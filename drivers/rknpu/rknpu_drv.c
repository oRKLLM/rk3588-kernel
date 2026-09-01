// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Felix Zeng <felix.zeng@rock-chips.com>
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/of_address.h>

#ifndef FPGA_PLATFORM
#include <soc/rockchip/rockchip_iommu.h>
#endif

#include <linux/io.h>
#include "rknpu_ioctl.h"
#include "rknpu_reset.h"
#include "rknpu_fence.h"
#include "rknpu_drv.h"
#include "rknpu_gem.h"
#include "rknpu_devfreq.h"
#include "rknpu_iommu.h"

#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
#include <drm/drm_device.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_file.h>
#include <drm/drm_drv.h>
#include "rknpu_gem.h"
#endif

#ifdef CONFIG_ROCKCHIP_RKNPU_DMA_HEAP
#include <linux/rk-dma-heap.h>
#include "rknpu_mem.h"
#endif

#define POWER_DOWN_FREQ 200000000
#define NPU_MMU_DISABLED_POLL_PERIOD_US 1000
#define NPU_MMU_DISABLED_POLL_TIMEOUT_US 20000

static int bypass_irq_handler;
module_param(bypass_irq_handler, int, 0644);
MODULE_PARM_DESC(bypass_irq_handler,
		 "bypass RKNPU irq handler if set it to 1, disabled by default");

static int bypass_soft_reset;
module_param(bypass_soft_reset, int, 0644);
MODULE_PARM_DESC(bypass_soft_reset,
		 "bypass RKNPU soft reset if set it to 1, disabled by default");

static const struct rknpu_irqs_data rknpu_irqs[] = {
	{ "npu_irq", rknpu_core0_irq_handler }
};

static const struct rknpu_irqs_data rk3576_npu_irqs[] = {
	{ "npu0_irq", rknpu_core0_irq_handler },
	{ "npu1_irq", rknpu_core1_irq_handler }
};

static const struct rknpu_irqs_data rk3588_npu_irqs[] = {
	{ "npu0_irq", rknpu_core0_irq_handler },
	{ "npu1_irq", rknpu_core1_irq_handler },
	{ "npu2_irq", rknpu_core2_irq_handler }
};

static const struct rknpu_amount_data rknpu_old_top_amount = {
	.offset_clr_all = 0x8010,
	.offset_dt_wr = 0x8034,
	.offset_dt_rd = 0x8038,
	.offset_wt_rd = 0x803c,
};

static const struct rknpu_amount_data rknpu_top_amount = {
	.offset_clr_all = 0x2210,
	.offset_dt_wr = 0x2234,
	.offset_dt_rd = 0x2238,
	.offset_wt_rd = 0x223c
};

static const struct rknpu_amount_data rknpu_core_amount = {
	.offset_clr_all = 0x2410,
	.offset_dt_wr = 0x2434,
	.offset_dt_rd = 0x2438,
	.offset_wt_rd = 0x243c,
};

static void rk3576_state_init(struct rknpu_device *rknpu_dev)
{
	void __iomem *rknpu_core_base = rknpu_dev->base[0];

	writel(0x1, rknpu_core_base + 0x10);
	writel(0, rknpu_core_base + 0x1004);
	writel(0x80000000, rknpu_core_base + 0x1024);
	writel(1, rknpu_core_base + 0x1004);
	writel(0x80000000, rknpu_core_base + 0x1024);
	writel(0x1e, rknpu_core_base + 0x1004);
}

static int rk3576_cache_sgt_init(struct rknpu_device *rknpu_dev)
{
	struct sg_table *sgt = NULL;
	struct scatterlist *sgl = NULL;
	uint64_t block_size_kb[4] = { 448, 64, 448, 64 };
	uint64_t block_offset_kb[4] = { 0, 896, 448, 960 };
	int core_num = rknpu_dev->config->num_irqs;
	int ret = 0, i = 0, j = 0;

	for (i = 0; i < core_num; i++) {
		sgt = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
		if (!sgt)
			goto out_free_table;
		ret = sg_alloc_table(sgt, core_num, GFP_KERNEL);
		if (ret) {
			kfree(sgt);
			goto out_free_table;
		}
		rknpu_dev->cache_sgt[i] = sgt;
		for_each_sgtable_sg(sgt, sgl, j) {
			sg_set_page(sgl, NULL,
				    block_size_kb[i * core_num + j] * 1024,
				    block_offset_kb[i * core_num + j] * 1024);
		}
	}
	return 0;

out_free_table:
	for (i = 0; i < core_num; i++) {
		if (rknpu_dev->cache_sgt[i]) {
			sg_free_table(rknpu_dev->cache_sgt[i]);
			kfree(rknpu_dev->cache_sgt[i]);
			rknpu_dev->cache_sgt[i] = NULL;
		}
	}

	return ret;
}

static const struct rknpu_config rk356x_rknpu_config = {
	.bw_priority_addr = 0xfe180008,
	.bw_priority_length = 0x10,
	.dma_mask = DMA_BIT_MASK(32),
	.pc_data_amount_scale = 1,
	.pc_task_number_bits = 12,
	.pc_task_number_mask = 0xfff,
	.pc_task_status_offset = 0x3c,
	.pc_dma_ctrl = 0,
	.irqs = rknpu_irqs,
	.num_irqs = ARRAY_SIZE(rknpu_irqs),
	.nbuf_phyaddr = 0,
	.nbuf_size = 0,
	.max_submit_number = (1 << 12) - 1,
	.core_mask = 0x1,
	.amount_top = &rknpu_old_top_amount,
	.amount_core = NULL,
	.state_init = NULL,
	.cache_sgt_init = NULL,
};

static const struct rknpu_config rk3588_rknpu_config = {
	.bw_priority_addr = 0x0,
	.bw_priority_length = 0x0,
	.dma_mask = DMA_BIT_MASK(40),
	.pc_data_amount_scale = 2,
	.pc_task_number_bits = 12,
	.pc_task_number_mask = 0xfff,
	.pc_task_status_offset = 0x3c,
	.pc_dma_ctrl = 0,
	.irqs = rk3588_npu_irqs,
	.num_irqs = ARRAY_SIZE(rk3588_npu_irqs),
	.nbuf_phyaddr = 0,
	.nbuf_size = 0,
	.max_submit_number = (1 << 12) - 1,
	.core_mask = 0x7,
	.amount_top = NULL,
	.amount_core = NULL,
	.state_init = NULL,
	.cache_sgt_init = NULL,
};

static const struct rknpu_config rk3583_rknpu_config = {
	.bw_priority_addr = 0x0,
	.bw_priority_length = 0x0,
	.dma_mask = DMA_BIT_MASK(40),
	.pc_data_amount_scale = 2,
	.pc_task_number_bits = 12,
	.pc_task_number_mask = 0xfff,
	.pc_task_status_offset = 0x3c,
	.pc_dma_ctrl = 0,
	.irqs = rk3588_npu_irqs,
	.num_irqs = 2,
	.nbuf_phyaddr = 0,
	.nbuf_size = 0,
	.max_submit_number = (1 << 12) - 1,
	.core_mask = 0x3,
	.amount_top = NULL,
	.amount_core = NULL,
	.state_init = NULL,
	.cache_sgt_init = NULL,
};

static const struct rknpu_config rv1106_rknpu_config = {
	.bw_priority_addr = 0x0,
	.bw_priority_length = 0x0,
	.dma_mask = DMA_BIT_MASK(32),
	.pc_data_amount_scale = 2,
	.pc_task_number_bits = 16,
	.pc_task_number_mask = 0xffff,
	.pc_task_status_offset = 0x3c,
	.pc_dma_ctrl = 0,
	.irqs = rknpu_irqs,
	.num_irqs = ARRAY_SIZE(rknpu_irqs),
	.nbuf_phyaddr = 0,
	.nbuf_size = 0,
	.max_submit_number = (1 << 16) - 1,
	.core_mask = 0x1,
	.amount_top = &rknpu_old_top_amount,
	.amount_core = NULL,
	.state_init = NULL,
	.cache_sgt_init = NULL,
};

static const struct rknpu_config rk3562_rknpu_config = {
	.bw_priority_addr = 0x0,
	.bw_priority_length = 0x0,
	.dma_mask = DMA_BIT_MASK(40),
	.pc_data_amount_scale = 2,
	.pc_task_number_bits = 16,
	.pc_task_number_mask = 0xffff,
	.pc_task_status_offset = 0x48,
	.pc_dma_ctrl = 1,
	.irqs = rknpu_irqs,
	.num_irqs = ARRAY_SIZE(rknpu_irqs),
	.nbuf_phyaddr = 0xfe400000,
	.nbuf_size = 256 * 1024,
	.max_submit_number = (1 << 16) - 1,
	.core_mask = 0x1,
	.amount_top = &rknpu_old_top_amount,
	.amount_core = NULL,
	.state_init = NULL,
	.cache_sgt_init = NULL,
};

static const struct rknpu_config rk3576_rknpu_config = {
	.bw_priority_addr = 0x0,
	.bw_priority_length = 0x0,
	.dma_mask = DMA_BIT_MASK(40),
	.pc_data_amount_scale = 2,
	.pc_task_number_bits = 16,
	.pc_task_number_mask = 0xffff,
	.pc_task_status_offset = 0x48,
	.pc_dma_ctrl = 1,
	.irqs = rk3576_npu_irqs,
	.num_irqs = ARRAY_SIZE(rk3576_npu_irqs),
	.nbuf_phyaddr = 0x3fe80000,
	.nbuf_size = 1024 * 1024,
	.max_submit_number = (1 << 16) - 1,
	.core_mask = 0x3,
	.amount_top = &rknpu_top_amount,
	.amount_core = &rknpu_core_amount,
	.state_init = rk3576_state_init,
	.cache_sgt_init = rk3576_cache_sgt_init,
};

/* driver probe and init */
static const struct of_device_id rknpu_of_match[] = {
	{
		.compatible = "rockchip,rknpu",
		.data = &rk356x_rknpu_config,
	},
	{
		.compatible = "rockchip,rk3568-rknpu",
		.data = &rk356x_rknpu_config,
	},
	{
		.compatible = "rockchip,rk3588-rknpu",
		.data = &rk3588_rknpu_config,
	},
	{
		.compatible = "rockchip,rv1106-rknpu",
		.data = &rv1106_rknpu_config,
	},
	{
		.compatible = "rockchip,rk3562-rknpu",
		.data = &rk3562_rknpu_config,
	},
	{
		.compatible = "rockchip,rk3576-rknpu",
		.data = &rk3576_rknpu_config,
	},
	{},
};

static int rknpu_get_drv_version(uint32_t *version)
{
	*version = RKNPU_GET_DRV_VERSION_CODE(DRIVER_MAJOR, DRIVER_MINOR,
					      DRIVER_PATCHLEVEL);
	return 0;
}

static int rknpu_power_on(struct rknpu_device *rknpu_dev);
static int rknpu_power_off(struct rknpu_device *rknpu_dev);

static void rknpu_power_off_delay_work(struct work_struct *power_off_work)
{
	int ret = 0;
	struct rknpu_device *rknpu_dev =
		container_of(to_delayed_work(power_off_work),
			     struct rknpu_device, power_off_work);
	mutex_lock(&rknpu_dev->power_lock);
	if (atomic_dec_if_positive(&rknpu_dev->power_refcount) == 0) {
		ret = rknpu_power_off(rknpu_dev);
		if (ret)
			atomic_inc(&rknpu_dev->power_refcount);
	}
	mutex_unlock(&rknpu_dev->power_lock);

	if (ret)
		rknpu_power_put_delay(rknpu_dev);
}

int rknpu_power_get(struct rknpu_device *rknpu_dev)
{
	int ret = 0;

	mutex_lock(&rknpu_dev->power_lock);
	if (atomic_inc_return(&rknpu_dev->power_refcount) == 1)
		ret = rknpu_power_on(rknpu_dev);
	mutex_unlock(&rknpu_dev->power_lock);

	return ret;
}

int rknpu_power_put(struct rknpu_device *rknpu_dev)
{
	int ret = 0;

	mutex_lock(&rknpu_dev->power_lock);
	if (atomic_dec_if_positive(&rknpu_dev->power_refcount) == 0) {
		ret = rknpu_power_off(rknpu_dev);
		if (ret)
			atomic_inc(&rknpu_dev->power_refcount);
	}
	mutex_unlock(&rknpu_dev->power_lock);

	if (ret)
		rknpu_power_put_delay(rknpu_dev);

	return ret;
}

int rknpu_power_put_delay(struct rknpu_device *rknpu_dev)
{
	if (rknpu_dev->power_put_delay == 0)
		return rknpu_power_put(rknpu_dev);

	mutex_lock(&rknpu_dev->power_lock);
	if (atomic_read(&rknpu_dev->power_refcount) == 1)
		queue_delayed_work(
			rknpu_dev->power_off_wq, &rknpu_dev->power_off_work,
			msecs_to_jiffies(rknpu_dev->power_put_delay));
	else
		atomic_dec_if_positive(&rknpu_dev->power_refcount);
	mutex_unlock(&rknpu_dev->power_lock);

	return 0;
}

static int rknpu_action(struct rknpu_device *rknpu_dev,
			struct rknpu_action *args)
{
	int ret = -EINVAL;

	switch (args->flags) {
	case RKNPU_GET_HW_VERSION:
		ret = rknpu_get_hw_version(rknpu_dev, &args->value);
		break;
	case RKNPU_GET_DRV_VERSION:
		ret = rknpu_get_drv_version(&args->value);
		break;
	case RKNPU_GET_FREQ:
#ifndef FPGA_PLATFORM
		args->value = clk_get_rate(rknpu_dev->clks[0].clk);
#endif
		ret = 0;
		break;
	case RKNPU_SET_FREQ:
		break;
	case RKNPU_GET_VOLT:
#ifndef FPGA_PLATFORM
		args->value = regulator_get_voltage(rknpu_dev->vdd);
#endif
		ret = 0;
		break;
	case RKNPU_SET_VOLT:
		break;
	case RKNPU_ACT_RESET_CORE: {
		/* #patch: per-core counterpart of RKNPU_ACT_RESET. Cheaper than the device-wide path — no
		 * msleep(100), only two reset lines — but it still needs the same quiesce for the IOMMU
		 * re-attach, which is unsafe asynchronously (it raced rknpu_iommu_dma_map_sg and panicked). */
		int _core = (int)args->value;
		if (_core < 0 || _core >= rknpu_dev->config->num_irqs)
			return -EINVAL;
		ret = rknpu_reset_core_and_remap(rknpu_dev, _core);
		break;
	}
	case RKNPU_ACT_SET_DOMAIN: {   /* #patchB11: make a domain live before an import's PRIME ioctl */
		int _id = (int)args->value;

		ret = rknpu_iommu_domain_get_and_switch(rknpu_dev, _id);
		if (!ret)
			rknpu_iommu_domain_put(rknpu_dev);   /* switch only; hold no reference */
		break;
	}
	case RKNPU_ACT_REINIT:   /* #patch61 */
		ret = rknpu_iommu_reinit(rknpu_dev);
		break;
	case RKNPU_ACT_RESET:
		ret = rknpu_soft_reset(rknpu_dev);
		break;
	case RKNPU_GET_BW_PRIORITY:
		ret = rknpu_get_bw_priority(rknpu_dev, &args->value, NULL,
					    NULL);
		break;
	case RKNPU_SET_BW_PRIORITY:
		ret = rknpu_set_bw_priority(rknpu_dev, args->value, 0, 0);
		break;
	case RKNPU_GET_BW_EXPECT:
		ret = rknpu_get_bw_priority(rknpu_dev, NULL, &args->value,
					    NULL);
		break;
	case RKNPU_SET_BW_EXPECT:
		ret = rknpu_set_bw_priority(rknpu_dev, 0, args->value, 0);
		break;
	case RKNPU_GET_BW_TW:
		ret = rknpu_get_bw_priority(rknpu_dev, NULL, NULL,
					    &args->value);
		break;
	case RKNPU_SET_BW_TW:
		ret = rknpu_set_bw_priority(rknpu_dev, 0, 0, args->value);
		break;
	case RKNPU_ACT_CLR_TOTAL_RW_AMOUNT:
		ret = rknpu_clear_rw_amount(rknpu_dev);
		break;
	case RKNPU_GET_DT_WR_AMOUNT:
		ret = rknpu_get_rw_amount(rknpu_dev, &args->value, NULL, NULL);
		break;
	case RKNPU_GET_DT_RD_AMOUNT:
		ret = rknpu_get_rw_amount(rknpu_dev, NULL, &args->value, NULL);
		break;
	case RKNPU_GET_WT_RD_AMOUNT:
		ret = rknpu_get_rw_amount(rknpu_dev, NULL, NULL, &args->value);
		break;
	case RKNPU_GET_TOTAL_RW_AMOUNT:
		ret = rknpu_get_total_rw_amount(rknpu_dev, &args->value);
		break;
	case RKNPU_GET_IOMMU_EN:
		args->value = rknpu_dev->iommu_en;
		ret = 0;
		break;
	case RKNPU_SET_PROC_NICE:
		set_user_nice(current, *(int32_t *)&args->value);
		ret = 0;
		break;
	case RKNPU_GET_TOTAL_SRAM_SIZE:
		if (rknpu_dev->sram_mm)
			args->value = rknpu_dev->sram_mm->total_chunks *
				      rknpu_dev->sram_mm->chunk_size;
		else
			args->value = 0;
		ret = 0;
		break;
	case RKNPU_GET_FREE_SRAM_SIZE:
		if (rknpu_dev->sram_mm)
			args->value = rknpu_dev->sram_mm->free_chunks *
				      rknpu_dev->sram_mm->chunk_size;
		else
			args->value = 0;
		ret = 0;
		break;
	case RKNPU_GET_IOMMU_DOMAIN_ID:
		args->value = rknpu_dev->iommu_domain_id;
		ret = 0;
		break;
	case RKNPU_SET_IOMMU_DOMAIN_ID: {
		ret = rknpu_iommu_domain_get_and_switch(
			rknpu_dev, *(int32_t *)&args->value);
		if (ret)
			break;
		rknpu_iommu_domain_put(rknpu_dev);
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#ifdef CONFIG_ROCKCHIP_RKNPU_DMA_HEAP
static int rknpu_open(struct inode *inode, struct file *file)
{
	struct rknpu_device *rknpu_dev =
		container_of(file->private_data, struct rknpu_device, miscdev);
	struct rknpu_session *session = NULL;

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session) {
		LOG_ERROR("rknpu session alloc failed\n");
		return -ENOMEM;
	}

	session->rknpu_dev = rknpu_dev;
	INIT_LIST_HEAD(&session->list);

	file->private_data = (void *)session;

	return nonseekable_open(inode, file);
}

static int rknpu_release(struct inode *inode, struct file *file)
{
	struct rknpu_mem_object *entry;
	struct rknpu_session *session = file->private_data;
	struct rknpu_device *rknpu_dev = session->rknpu_dev;
	LIST_HEAD(local_list);

	spin_lock(&rknpu_dev->lock);
	list_replace_init(&session->list, &local_list);
	file->private_data = NULL;
	spin_unlock(&rknpu_dev->lock);

	while (!list_empty(&local_list)) {
		entry = list_first_entry(&local_list, struct rknpu_mem_object,
					 head);

		LOG_DEBUG(
			"Fd close free rknpu_obj: %#llx, rknpu_obj->dma_addr: %#llx\n",
			(__u64)(uintptr_t)entry, (__u64)entry->dma_addr);

		vunmap(entry->kv_addr);
		entry->kv_addr = NULL;

		if (!entry->owner)
			dma_buf_put(entry->dmabuf);

		list_del(&entry->head);
		kfree(entry);
	}

	kfree(session);

	return 0;
}

static int rknpu_action_ioctl(struct rknpu_device *rknpu_dev,
			      unsigned long data)
{
	struct rknpu_action args;
	int ret = -EINVAL;

	if (unlikely(copy_from_user(&args, (struct rknpu_action *)data,
				    sizeof(struct rknpu_action)))) {
		LOG_ERROR("%s: copy_from_user failed\n", __func__);
		ret = -EFAULT;
		return ret;
	}

	ret = rknpu_action(rknpu_dev, &args);

	if (unlikely(copy_to_user((struct rknpu_action *)data, &args,
				  sizeof(struct rknpu_action)))) {
		LOG_ERROR("%s: copy_to_user failed\n", __func__);
		ret = -EFAULT;
		return ret;
	}

	return ret;
}

static long rknpu_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -EINVAL;
	struct rknpu_device *rknpu_dev = NULL;

	if (!file->private_data)
		return -EINVAL;

	rknpu_dev = ((struct rknpu_session *)file->private_data)->rknpu_dev;

	rknpu_power_get(rknpu_dev);

	switch (_IOC_NR(cmd)) {
	case RKNPU_ACTION:
		ret = rknpu_action_ioctl(rknpu_dev, arg);
		break;
	case RKNPU_SUBMIT:
		ret = rknpu_submit_ioctl(rknpu_dev, arg);
		break;
	case RKNPU_MEM_CREATE:
		ret = rknpu_mem_create_ioctl(rknpu_dev, file, cmd, arg);
		break;
	case RKNPU_MEM_MAP:
		break;
	case RKNPU_MEM_DESTROY:
		ret = rknpu_mem_destroy_ioctl(rknpu_dev, file, arg);
		break;
	case RKNPU_MEM_SYNC:
		ret = rknpu_mem_sync_ioctl(rknpu_dev, arg);
		break;
	default:
		break;
	}

	rknpu_power_put_delay(rknpu_dev);

	return ret;
}
const struct file_operations rknpu_fops = {
	.owner = THIS_MODULE,
	.open = rknpu_open,
	.release = rknpu_release,
	.unlocked_ioctl = rknpu_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = rknpu_ioctl,
#endif
};
#endif

#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
static const struct vm_operations_struct rknpu_gem_vm_ops = {
	.fault = rknpu_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};
#endif

static int rknpu_action_ioctl(struct drm_device *dev, void *data,
			      struct drm_file *file_priv)
{
	struct rknpu_device *rknpu_dev = dev_get_drvdata(dev->dev);

	return rknpu_action(rknpu_dev, (struct rknpu_action *)data);
}

#define RKNPU_IOCTL(func)                                                   \
	static int __##func(struct drm_device *dev, void *data,             \
			    struct drm_file *file_priv)                     \
	{                                                                   \
		struct rknpu_device *rknpu_dev = dev_get_drvdata(dev->dev); \
		int ret = -EINVAL;                                          \
		rknpu_power_get(rknpu_dev);                                 \
		ret = func(dev, data, file_priv);                           \
		rknpu_power_put_delay(rknpu_dev);                           \
		return ret;                                                 \
	}

RKNPU_IOCTL(rknpu_action_ioctl);
RKNPU_IOCTL(rknpu_submit_ioctl);
RKNPU_IOCTL(rknpu_gem_create_ioctl);
RKNPU_IOCTL(rknpu_gem_map_ioctl);
RKNPU_IOCTL(rknpu_gem_destroy_ioctl);
RKNPU_IOCTL(rknpu_gem_sync_ioctl);

static const struct drm_ioctl_desc rknpu_ioctls[] = {
	DRM_IOCTL_DEF_DRV(RKNPU_ACTION, __rknpu_action_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(RKNPU_SUBMIT, __rknpu_submit_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(RKNPU_MEM_CREATE, __rknpu_gem_create_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(RKNPU_MEM_MAP, __rknpu_gem_map_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(RKNPU_MEM_DESTROY, __rknpu_gem_destroy_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(RKNPU_MEM_SYNC, __rknpu_gem_sync_ioctl,
			  DRM_RENDER_ALLOW),
};

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
DEFINE_DRM_GEM_FOPS(rknpu_drm_driver_fops);
#else
static const struct file_operations rknpu_drm_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.mmap = rknpu_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.unlocked_ioctl = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.release = drm_release,
	.llseek = noop_llseek,
};
#endif

static struct drm_driver rknpu_drm_driver = {
#if KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE
	.driver_features = DRIVER_GEM | DRIVER_RENDER,
#else
	.driver_features = DRIVER_GEM | DRIVER_PRIME | DRIVER_RENDER,
#endif
#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
	.gem_free_object_unlocked = rknpu_gem_free_object,
	.gem_vm_ops = &rknpu_gem_vm_ops,
	.dumb_destroy = drm_gem_dumb_destroy,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_get_sg_table = rknpu_gem_prime_get_sg_table,
	.gem_prime_vmap = rknpu_gem_prime_vmap,
	.gem_prime_vunmap = rknpu_gem_prime_vunmap,
#endif
	.dumb_create = rknpu_gem_dumb_create,
#if KERNEL_VERSION(4, 19, 0) > LINUX_VERSION_CODE
	.dumb_map_offset = rknpu_gem_dumb_map_offset,
#else
	.dumb_map_offset = drm_gem_dumb_map_offset,
#endif
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
	.gem_prime_import = rknpu_gem_prime_import,
#else
	.gem_prime_import = drm_gem_prime_import,
#endif
	.gem_prime_import_sg_table = rknpu_gem_prime_import_sg_table,
#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	.gem_prime_mmap = drm_gem_prime_mmap,
#else
	.gem_prime_mmap = rknpu_gem_prime_mmap,
#endif
	.ioctls = rknpu_ioctls,
	.num_ioctls = ARRAY_SIZE(rknpu_ioctls),
	.fops = &rknpu_drm_driver_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

#endif

static enum hrtimer_restart hrtimer_handler(struct hrtimer *timer)
{
	struct rknpu_device *rknpu_dev =
		container_of(timer, struct rknpu_device, timer);
	struct rknpu_subcore_data *subcore_data = NULL;
	struct rknpu_job *job = NULL;
	ktime_t now;
	unsigned long flags;
	int i;

	for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
		subcore_data = &rknpu_dev->subcore_datas[i];

		spin_lock_irqsave(&rknpu_dev->irq_lock, flags);

		job = subcore_data->job;
		if (job) {
			now = ktime_get();
			subcore_data->timer.busy_time +=
				ktime_sub(now, job->hw_recoder_time);
			job->hw_recoder_time = now;
		}

		subcore_data->timer.total_busy_time =
			subcore_data->timer.busy_time;
		subcore_data->timer.busy_time = 0;

		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
	}

	hrtimer_forward_now(timer, rknpu_dev->kt);
	return HRTIMER_RESTART;
}

/* ================= PROGRESS WATCHDOG (#patch) ==========================================
 * WHY. rknpu_job_timeout_clean is not a watchdog: it runs only when ANOTHER submit arrives on
 * the same core, so a stalled job with no follow-up submit is never noticed (which is why
 * userspace has to fire "reap dummy" submits). It also measures TOTAL JOB AGE, meaningless for
 * a PC-chained job whose task list is extensible at runtime. "No task completed in N samples"
 * is well defined for a chain of any length.
 *
 * CONFIRMED FINDING: on a doorbell miss the PC completed-task counter reads 0 with the job many
 * ms old -- the job is accepted and NEVER DISPATCHES. (Control: at an 8 ms threshold a healthy
 * ~30 ms job does NOT report, so the counter genuinely advances and the offset is right.)
 *
 * POWER SAFETY -- the hard part, and v1/v2 got it wrong.
 *   v1 read NPU MMIO straight from the hrtimer. A register read on a powered-down block is an
 *   external abort => instant HARD hang (no ping, power cycle required). It hung the board on
 *   its first NPU workload.
 *   v2 guarded with pm_runtime_get_if_in_use(rknpu_dev->dev). STILL WRONG on RK3588: this driver
 *   keeps its OWN refcount (power_refcount/power_lock) and rknpu_power_off() puts the three
 *   PER-CORE genpds (genpd_dev_npu0/1/2) directly. A reference on the parent does not hold those
 *   up, so core i's domain can still drop between the check and the readl on base[i].
 *   v3 (this): sample in PROCESS context and hold power_lock across the read. rknpu_power_off()
 *   takes the same mutex, so the genpds cannot go down while we read. We deliberately do NOT call
 *   rknpu_power_get(): that would POWER THE NPU ON just to read a counter and pin it up forever.
 *   mutex_trylock means the watchdog can never delay a submit; a missed sample is simply skipped
 *   rather than counted as a no-advance.
 *
 * The hrtimer only decides cadence and queues the work; all MMIO happens in the worker. */
/* DEFAULT OFF. This watchdog has destabilised the board three times: v1 hard-hung it (unguarded MMIO,
 * fixed), and even the power_lock-guarded v3 COLD-RESET it twice mid-run (instant SoC reset, no panic,
 * no console, ramoops cleared). Discriminator, same workload: wd_period_us=1000 -> reset;
 * wd_period_us=0 -> 4000 reps clean, same boot_id. It is a DIAGNOSTIC: set wd_period_us=1000 only for
 * a deliberate investigation, accepting that the board may reset. */
/* #patch40: the progress watchdog is ON BY DEFAULT.
 *
 * It was opt-in because two earlier versions killed the board (v1 read NPU MMIO straight from the
 * hrtimer; v2's pm_runtime_get_if_in_use on the parent missed the per-core genpds). v3 samples in
 * process context holding power_lock and has since run clean. The other reason -- "it caused board
 * resets" -- was RETRACTED; that was the vendor IRQ bug fixed as change 6.
 *
 * Opt-in made it useless in practice: the one time it was needed, it was enabled by hand and
 * recorded wd_ticks=0 across 8 stalls, because the arm site was NONBLOCK-only (fixed in #patch39).
 * A stalled BLOCKING submit otherwise sits in rknpu_job_wait() for its whole timeout -- 60.6 s
 * measured -- with nothing reporting it. Detection at ~120 ms is a ~500x cut in time-to-visibility.
 *
 * Cost, measured A/B on this board (2000-rep int4 probe, ndom=1, interleaved x3, warmup discarded):
 *   OFF 133/133/133 commits/s   ON 131/133/129 commits/s   => ~1.5%
 * Paid on every submit since #patch39. Judged worth it: the stalls it exposes cost ~23% of 27B
 * prefill and were previously invisible.
 *
 * Detection latency is wd_samples * wd_period_us ~= 120 ms. Do NOT drop the period to 1000 without
 * also dropping wd_samples: 1 kHz sampling costs ~100x throughput (2.8 vs 283-490 commits/s). */
/* #patchB12: DEFAULT OFF — this is a DEVELOPER tool, not a production mechanism.
 *
 * The progress watchdog was written to chase "job committed, PC task counter 0x0, no interrupt",
 * which we believed was a hardware stall. It was not: it was our own multi-domain page-table bugs
 * (see the wiki, Kernel-Modifications changes 20-22 and Vendor-Kernel-Behaviour). Mainline
 * accel/rocket carries no stall detection at all, and does not need any.
 *
 * With those fixed the watchdog should never fire, so leaving it armed only costs a 100 Hz timer and
 * the risk of misreading a slow-but-healthy job: the detection threshold is wd_samples * wd_period_us
 * ~= 120 ms, and a cold weight-DMA can legitimately exceed that and still complete bit-exact
 * (measured 2026-08-27 — several detections per suite run with every output bit-exact).
 *
 * It stays useful for DEVELOPERS: a wedge-prone shape can still hang the NPU, and a hang costs a
 * reboot — or, for wide 27B prefill, a physical SPI reflash. So arm it while bringing up new shapes:
 *     echo 10000 > /sys/module/rknpu/parameters/wd_period_us   # detect + log
 *     echo 2000  > /sys/module/rknpu/parameters/wd_abort_ms    # AND fail the submit (opt-in on top)
 * The cheap counters (cnt_commit / cnt_done / cnt_irq) are always on and cost nothing — they are what
 * actually localised the bug this was built for. */
static unsigned int rknpu_wd_period_us;   /* 0 = off; see above */
module_param_named(wd_period_us, rknpu_wd_period_us, uint, 0644);
MODULE_PARM_DESC(wd_period_us, "progress watchdog sample period in us (0 = off, the default; set 10000 to arm developer stall detection)");

/* diagnostics: is the watchdog actually sampling, and why not when it isn't */
static unsigned long rknpu_wd_ticks, rknpu_wd_seen_job, rknpu_wd_pm_skip, rknpu_wd_stalls;
module_param_named(wd_ticks, rknpu_wd_ticks, ulong, 0444);
module_param_named(wd_seen_job, rknpu_wd_seen_job, ulong, 0444);
module_param_named(wd_pm_skip, rknpu_wd_pm_skip, ulong, 0444);
module_param_named(wd_stalls, rknpu_wd_stalls, ulong, 0444);

/* START-CONFIRM-AND-RETRY (wd_kick=1).
 * A stalled job is committed with byte-identical registers, a valid mapping, a fresh descriptor and
 * verified-correct regcmd bytes in DRAM — it simply never begins, and the completed-task counter stays 0.
 * The registers are still correct at that point, so rather than the current recovery (userspace waits out
 * a ~614 ms window, then issues a DEVICE-WIDE ACT_RESET that destroys warm state on all three cores, then
 * resubmits — ~3.6 s per miss), just RE-PULSE the start trigger for that one core.
 *
 * Cheap and targeted: two register writes, no reset, no warm-state loss, no resubmit. If the PC merely
 * missed its start, this restarts it; if it does not, we have lost nothing and learned that the fault is
 * not a missed start trigger. Fires ONCE per stall episode, on the same condition as the report, and only
 * with the counter still reading 0 (i.e. nothing has executed, so re-pulsing cannot disturb work in
 * flight). Opt-in while under evaluation. */
static unsigned long rknpu_cnt_kick;
module_param_named(cnt_kick, rknpu_cnt_kick, ulong, 0444);

static unsigned int rknpu_wd_samples = 12;
/* #patch41: FAST-ABORT. When a BLOCKING submit stalls, its caller sits in rknpu_job_wait() for
 * args->timeout x 3 -- 60.8 s measured on a real 1.5B-Q8 run -- before getting -ETIMEDOUT and
 * self-healing. Waking it as soon as the stall is established turns that into ~wd_abort_ms.
 *
 * DELIBERATELY NOT the logging threshold. The PC counter advances per COMPLETED TASK, so a single
 * long-running task is indistinguishable from a stall by this signal alone; aborting at the ~120 ms
 * log threshold would kill slow-but-healthy jobs. This is a separate, much larger bound: still
 * ~30x faster than 60 s while leaving any plausible single task room to finish.
 *
 * 0 disables. OPT-IN on purpose -- the previous automatic stall intervention (wd_kick) was deleted
 * for having no measurable benefit, so this one earns its default only once measured. */
static unsigned int rknpu_wd_abort_ms;
module_param_named(wd_abort_ms, rknpu_wd_abort_ms, uint, 0644);
MODULE_PARM_DESC(wd_abort_ms, "fail a stalled BLOCKING submit after this many ms (0 = off)");
/* #patch47: opt-in timing profiler. prof_mask != 0 enables it (bit0 = per-stall breakdown,
 * bit1 = also log healthy commits). Everything it reports is derived from timestamps the driver
 * already takes, so the cost when disabled is one branch and when enabled is one printk. */
/* #patch49: CAUSAL TEST for the change-16 correlation.
 *
 * Every stalled job was committed 0.24-0.9 ms after the previous commit on the same core, against a
 * healthy mean three orders of magnitude larger. That is correlation. This enforces a MINIMUM spacing
 * between commits on a core so the hypothesis can be tested directly: if stalls vanish as this rises,
 * back-to-back commits cause them; if they persist, the short gap is a symptom of something else.
 *
 * 0 = off (default). Clamped to 5000 us because this MUST busy-wait: rknpu_job_next() can be reached
 * from the completion IRQ, where sleeping is illegal. A diagnostic knob, not a shipping fix -- a real
 * fix would reshape submission rather than burn CPU in udelay(). */
static unsigned int rknpu_min_commit_gap_us;
module_param_named(min_commit_gap_us, rknpu_min_commit_gap_us, uint, 0644);
MODULE_PARM_DESC(min_commit_gap_us, "enforce >= this gap between commits on a core (0=off, max 5000)");
unsigned long rknpu_gap_enforced_n, rknpu_gap_enforced_us;
module_param_named(gap_enforced_n, rknpu_gap_enforced_n, ulong, 0444);
module_param_named(gap_enforced_us_sum, rknpu_gap_enforced_us, ulong, 0444);
unsigned int rknpu_min_commit_gap_get(void) { return rknpu_min_commit_gap_us; }

/* #patch55: SUB-BLOCK register dump — is the NPU stuck mid-read, or did it never issue one?
 *
 * At stall time the PC completed-task counter reads 0x0 and all four MMU banks read IDLE
 * (0x19 = paging|idle|replay-empty). Those two facts pull in opposite directions: MMU-idle says no
 * translation is outstanding, i.e. nothing was ever requested; but the job WAS triggered, so
 * something should have been. The missing instrument is the NPU's own sub-blocks — we only ever
 * dumped the PC block.
 *
 *   CNA  0x1000  does the convolution feeder think it is running?
 *   DPU  0x4000  output stage state
 *   CDMA 0x5000  IS A DMA READ OUTSTANDING?  <- the discriminator
 *
 * CDMA busy while the MMU is idle == a transaction stuck UPSTREAM of translation (a hung read).
 * CDMA idle too == the trigger never took and nothing was issued.
 *
 * Deliberately dumped BLIND rather than by name: mainline drivers/accel/rocket names these, but it
 * is a much newer kernel than this tree, and we do not need names to diff a stalled snapshot against
 * a healthy one. Names can follow once we know which words move.
 *
 * Safety: each core maps 0x10000 (rk3588s.dtsi), so 0x1000/0x4000/0x5000 are in range; and this runs
 * only from the watchdog worker, which is process context holding power_lock with power_refcount > 0
 * (patch06: an MMIO read of a powered-down block kills the machine instantly). Capped so a stall
 * storm cannot flood the log. */
/* #patch58: does the PC completed-task counter EVER read non-zero while a job is in flight?
 *
 * The cnt>0 healthy control never fired across a whole run despite tens of thousands of samples.
 * If cnt is always 0 for in-flight jobs then `cnt == wd_last` is trivially true and the watchdog is
 * not a progress detector at all — it is a "job present for > wd_samples*period" duration timer, and
 * the "counter 0x0" in every stall report is NOT evidence that the hardware never started, because
 * healthy jobs would report it too. That would invalidate the framing this whole investigation has
 * carried. Record the maximum ever observed and settle it. */
static unsigned int rknpu_wd_max_cnt;
module_param_named(wd_max_cnt, rknpu_wd_max_cnt, uint, 0444);
static unsigned long rknpu_wd_cnt_nonzero;
module_param_named(wd_cnt_nonzero, rknpu_wd_cnt_nonzero, ulong, 0444);

/* #patch67: TIME SERIES of CNA/DPU S_STATUS across in-flight jobs (prof_mask bit 3).
 *
 * The one-shot healthy dump showed CNA/DPU S_STATUS = 0x00000000 healthy vs 0x00000005 stalled, which
 * suggests the PC DID start the downstream units and they are parked. But `cnt > 0` only proves the
 * job executed a task — the single sample may have landed BETWEEN tasks, so "healthy = 0" could be a
 * quiet moment rather than the running state, which would invalidate the comparison.
 *
 * Sample every watchdog tick for every in-flight job and log wd_flat alongside, so healthy
 * (flat small, still progressing) and stalling (flat large) samples sit in one stream and can be
 * separated offline. If 0x5 ever appears transiently while healthy, it means RUNNING and the units are
 * merely stuck mid-op; if it never does, 0x5 is specific to the failure. */
static unsigned long rknpu_prof_stat_n;
module_param_named(prof_stat_n, rknpu_prof_stat_n, ulong, 0444);
static unsigned long rknpu_prof_stat_cap = 400;
module_param_named(prof_stat_cap, rknpu_prof_stat_cap, ulong, 0644);

static unsigned int rknpu_prof_blk_cap = 4;
module_param_named(prof_blk_cap, rknpu_prof_blk_cap, uint, 0644);
static unsigned int rknpu_prof_blk_done;
static unsigned int rknpu_prof_blk_healthy_done;

static void rknpu_blk_dump(void __iomem *base, int core, const char *tag)
{
	/* #patch65: include the PC block (0x0000) — the one we never compared.
	 *
	 * The CPU->NPU protocol is a doorbell plus a DMA-fetched program: write PC_DATA_ADDR (0x10)
	 * with the regcmd's IOVA, PC_DATA_AMOUNT (0x14), the interrupt regs, then PC_TASK_CONTROL
	 * (0x30) whose 0x6 field is the trigger; the NPU then DMA-reads the regcmd and executes it.
	 * At a stall the trigger IS consumed (0x7002 written, 0x1002 read back) but the MMU stays
	 * idle, so the fetch never happens — the doorbell is acknowledged and the program is never
	 * collected.
	 *
	 * A stall dump showed PC_OP_EN (0x08) reading 0x00000000 with INT_STATUS and INT_RAW_STATUS
	 * both 0. The commit sequence never writes PC_OP_EN, and one watchdog kick mode was
	 * "re-pulse PC_OP_EN", so it is plausibly the run-enable — but we have never dumped this
	 * block for a HEALTHY job, so there is nothing to compare against. Add it. */
	static const u32 blk[] = { 0x0000, 0x1000, 0x4000, 0x5000 };
	static const char *nm[] = { "PC", "CNA", "DPU", "CDMA" };
	int b, i;

	/* #patch57: 16 words per block, not 8. The first 8 were byte-identical between healthy and
	 * stalled, which may simply mean they are static config. Widen so a word that actually moves
	 * has a chance to appear. */
	for (b = 0; b < 4; b++) {
		u32 v[16];
		int h;

		for (i = 0; i < 16; i++)
			v[i] = readl(base + blk[b] + i * 4);
		for (h = 0; h < 2; h++)
			LOG_ERROR("RKNPU: BLK[%s] core%d %-4s +0x%04x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				  tag, core, nm[b], blk[b] + h * 32,
				  v[h*8+0], v[h*8+1], v[h*8+2], v[h*8+3],
				  v[h*8+4], v[h*8+5], v[h*8+6], v[h*8+7]);
	}
}

extern unsigned long rknpu_dbg_destroy_bailed;   /* #patch69 */
module_param_named(dbg_destroy_bailed, rknpu_dbg_destroy_bailed, ulong, 0444);

static unsigned int rknpu_prof_mask;
module_param_named(prof_mask, rknpu_prof_mask, uint, 0644);
MODULE_PARM_DESC(prof_mask, "timing profiler: 0=off, 1=stall breakdown, 3=+healthy commits");
/* #patch50: shape-axis profiling. bit1 = log every stall with its shape (already on),
 * bit2 = ALSO log every commit with its shape, so the HEALTHY distribution can be built.
 * ~1100 lines per run — trivial for the persistent disk log, and it keeps all the binning
 * offline in awk instead of hard-coding a histogram in the kernel. */
unsigned int rknpu_prof_mask_get(void) { return rknpu_prof_mask; }
/* #patch48: non-static — rknpu_job_done() (other TU) accumulates the healthy side */
unsigned long rknpu_prof_ok_n, rknpu_prof_ok_gap, rknpu_prof_ok_queue;
module_param_named(prof_ok_n, rknpu_prof_ok_n, ulong, 0444);
module_param_named(prof_ok_gap_us_sum, rknpu_prof_ok_gap, ulong, 0444);
module_param_named(prof_ok_queue_us_sum, rknpu_prof_ok_queue, ulong, 0444);
static unsigned long rknpu_prof_bad_n, rknpu_prof_bad_gap, rknpu_prof_bad_queue;
module_param_named(prof_bad_n, rknpu_prof_bad_n, ulong, 0444);
module_param_named(prof_bad_gap_us_sum, rknpu_prof_bad_gap, ulong, 0444);
module_param_named(prof_bad_queue_us_sum, rknpu_prof_bad_queue, ulong, 0444);

static unsigned long rknpu_wd_queued_aborts;
module_param_named(wd_queued_aborts, rknpu_wd_queued_aborts, ulong, 0444);
static unsigned long rknpu_wd_aborts;
module_param_named(wd_aborts, rknpu_wd_aborts, ulong, 0444);   /* #patch40: 12 x 10 ms ~= 120 ms detection */
module_param_named(wd_samples, rknpu_wd_samples, uint, 0644);
MODULE_PARM_DESC(wd_samples, "consecutive no-advance samples before reporting a stall");

/* Process context: safe to take power_lock, so safe to touch MMIO. */
static void rknpu_wd_work_fn(struct work_struct *work)
{
	struct rknpu_device *rknpu_dev =
		container_of(work, struct rknpu_device, wd_work);
	struct rknpu_subcore_data *subcore_data = NULL;
	struct rknpu_job *job = NULL;
	unsigned long flags;
	int i;

	/* never block a submit: power_lock is held by rknpu_power_get/put on the hot path */
	if (!mutex_trylock(&rknpu_dev->power_lock)) {
		rknpu_wd_pm_skip++;
		return;
	}
	/* powered off => nothing is running and MMIO is unsafe; skip without penalising wd_flat */
	if (atomic_read(&rknpu_dev->power_refcount) <= 0) {
		rknpu_wd_pm_skip++;
		mutex_unlock(&rknpu_dev->power_lock);
		return;
	}

	for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
		void __iomem *rknpu_core_base = rknpu_dev->base[i];
		u32 cnt;

		subcore_data = &rknpu_dev->subcore_datas[i];

		spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
		job = subcore_data->job;
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

		if (!job) {
			subcore_data->wd_flat = 0;
			subcore_data->wd_job = NULL;   /* #patch44 */
			continue;
		}
		rknpu_wd_seen_job++;

		cnt = readl(rknpu_core_base +
			    rknpu_dev->config->pc_task_status_offset) &
		      rknpu_dev->config->pc_task_number_mask;

		/* #patch44: a stall run must be attributed to a JOB, not just to a counter value.
		 *
		 * wd_flat only reset when the PC counter CHANGED. Consecutive jobs on one core each
		 * start their counter at 0, so across a job boundary the counter looks unchanged and
		 * wd_flat keeps climbing. Both the stall report and the fast-abort fire on EQUALITY
		 * (== wd_samples / == an), so only the FIRST stall in a run of them was ever seen and
		 * every later one was silently absorbed.
		 *
		 * Measured before this fix: 10 timed-out waits, 5 detected+aborted at 2.0 s, 5 missed
		 * entirely at ~61 s -- exactly 50% coverage, with the misses in a clean second mode.
		 * Reset the run whenever the job identity changes. */
		if (subcore_data->wd_job != job) {
			subcore_data->wd_job = job;
			subcore_data->wd_last = cnt;
			subcore_data->wd_flat = 0;
		}

		if ((rknpu_prof_mask & 8) &&
		    rknpu_prof_stat_n < rknpu_prof_stat_cap) {
			rknpu_prof_stat_n++;
			/* #patch68: INT_RAW_STATUS (PC block, 0x2c) in the same series.
			 *
			 * It read 0x00000008 healthy vs 0xc0000000 stalled — but from a SINGLE sample
			 * of each, the same one-shot method that made CNA/DPU 0x5 look like a failure
			 * signature when it is really the RUNNING state. Sample it across both
			 * populations before believing it. */
			LOG_ERROR("RKNPU: SSTAT core%d cnt=%u flat=%u CNA=%08x DPU=%08x RAW=%08x\n",
				  i, cnt, subcore_data->wd_flat,
				  readl(rknpu_core_base + 0x1000),
				  readl(rknpu_core_base + 0x4000),
				  readl(rknpu_core_base + 0x2c));
		}

		if (cnt > rknpu_wd_max_cnt)
			rknpu_wd_max_cnt = cnt;   /* #patch58 */
		if (cnt)
			rknpu_wd_cnt_nonzero++;

		/* #patch57: a VALID healthy control.
		 *
		 * The previous baseline fired at wd_flat == 0, which only means RECENTLY COMMITTED —
		 * plausibly the identical register state to a job that is about to be reported
		 * stalled. Comparing that against a stall compares a state with itself, and indeed
		 * every word matched. Gate on cnt > 0 instead: the PC completed-task counter has
		 * advanced, so this job has provably executed at least one task and cannot be in the
		 * failure state (whose defining symptom is counter stuck at 0x0). Also far more
		 * catchable than requiring the counter to CHANGE between two 10 ms samples. */
		if ((rknpu_prof_mask & 4) && cnt > 0 &&
		    rknpu_prof_blk_healthy_done < rknpu_prof_blk_cap) {
			rknpu_prof_blk_healthy_done++;
			rknpu_blk_dump(rknpu_core_base, i, "OK");
		}

		if (cnt == subcore_data->wd_last) {
			/* fire ONCE per stall episode (equality, not >=) */
			if (++subcore_data->wd_flat == rknpu_wd_samples) {
				rknpu_wd_stalls++;
				if ((rknpu_prof_mask & 4) &&
				    rknpu_prof_blk_done < rknpu_prof_blk_cap) {
					rknpu_prof_blk_done++;
					rknpu_blk_dump(rknpu_core_base, i, "STALL");   /* #patch55 */
				}
				/* #patch47: attribute this stall's timeline */
				rknpu_prof_bad_n++;
				rknpu_prof_bad_gap += (unsigned long)job->prof_gap_us;
				rknpu_prof_bad_queue += (unsigned long)job->prof_queue_us;
				if (rknpu_prof_mask & 1)
					LOG_ERROR("RKNPU: PROF stalled core %d: tn=%u cm=%#x dom=%d sinceswitch=%u tsince=%lldus gap_since_prev_commit=%lldus queue(submit->commit)=%lldus | healthy mean gap=%luus queue=%luus over n=%lu\n",
						  i, job->args->task_number,
						  job->args->core_mask,
						  job->iommu_domain_id,
						  job->prof_since_switch,
						  job->prof_tsince_us,
						  job->prof_gap_us, job->prof_queue_us,
						  rknpu_prof_ok_n ? rknpu_prof_ok_gap / rknpu_prof_ok_n : 0UL,
						  rknpu_prof_ok_n ? rknpu_prof_ok_queue / rknpu_prof_ok_n : 0UL,
						  rknpu_prof_ok_n);
				/* #patch: report COMPLETED vs SUBMITTED so the line is self-interpreting.
				 * The kernel has both: the PC counter (completed) and the submit's task
				 * counts. "0/21" says the job never started; "17/21" would say it stalled
				 * partway. core_mask is echoed so this pairs with the userspace
				 * ORK_KMSG_SUBMIT stamp (`ork: sub#N dom=D cm=0xM tasks=T`). */
				LOG_ERROR("RKNPU: core %d NO TASK PROGRESS for %u samples (%u us), tasks=%u/%u done (counter %#x), cm=0x%x dom=%u, job age %lld us\n",
					  i, subcore_data->wd_flat,
					  subcore_data->wd_flat * rknpu_wd_period_us,
					  cnt,
					  (job->args->subcore_task[i].task_number ?
					   job->args->subcore_task[i].task_number :
					   job->args->task_number),
					  cnt,
					  job->args->core_mask,
					  job->args->iommu_domain_id,
					  ktime_us_delta(ktime_get(), job->timestamp));
				if (job->dbg_valid)
					LOG_ERROR("RKNPU:   commit snapshot: seq=%u WROTE data_addr=%#x amount=%#x int_mask=%#x task_ctrl=%#x dma_base=%#x tasks=%u | READBACK data_addr=%#x amount=%#x int_mask=%#x task_ctrl=%#x dma_base=%#x status(pre)=%#x status(post)=%#x\n",
						  job->dbg_seq, job->dbg_wr[0], job->dbg_wr[1], job->dbg_wr[2],
						  job->dbg_wr[3], job->dbg_wr[4], job->dbg_wr[5],
						  job->dbg_rd[0], job->dbg_rd[1], job->dbg_rd[2],
						  job->dbg_rd[3], job->dbg_rd[4], job->dbg_rd[5],
						  job->dbg_rd[6]);
				LOG_ERROR("RKNPU:   PCBLK %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x | en=%08x\n",
					  job->dbg_blk[0], job->dbg_blk[1], job->dbg_blk[2], job->dbg_blk[3],
					  job->dbg_blk[4], job->dbg_blk[5], job->dbg_blk[6], job->dbg_blk[7],
					  job->dbg_blk[8], job->dbg_blk[9], job->dbg_blk[10], job->dbg_blk[11],
					  job->dbg_blk[12], job->dbg_blk[13], job->dbg_blk[14], job->dbg_blk[15],
					  job->dbg_blk[16]);
			}

			/* #patch41: fast-abort a stalled BLOCKING submit. ASYNC jobs have no
			 * waiter, so there is nothing to wake. Equality fires it once. */
			if (rknpu_wd_abort_ms && !(job->flags & RKNPU_JOB_ASYNC)) {
				unsigned int an = (rknpu_wd_abort_ms * 1000u) /
						  (rknpu_wd_period_us ?
							   rknpu_wd_period_us : 1u);

				if (an && subcore_data->wd_flat == an) {
					struct rknpu_job *entry = NULL;
					unsigned int qn = 0;

					spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
					job->flags |= RKNPU_JOB_STALLED;
					/* #patch43: also abort everything QUEUED behind this head.
					 *
					 * This is the detection-coverage gap: the sampler only ever
					 * inspects subcore_data->job, the COMMITTED job, so a submit
					 * waiting in todo_list is invisible and its waiter burns the
					 * whole timeout for a job that never even started. Measured:
					 * 14 userspace submit failures produced only 2 watchdog
					 * episodes, and fast-abort therefore only helped 2 of 14.
					 *
					 * Safe to declare these stalled: rknpu_job_next() promotes
					 * from todo_list ONLY when subcore_data->job is NULL, so while
					 * this head is stuck nothing behind it can run. A woken job
					 * that never committed has last_task == NULL, and
					 * rknpu_job_wait() already handles exactly that -- it unlinks
					 * itself from todo_list and returns. No new path needed. */
					list_for_each_entry(entry,
							    &subcore_data->todo_list,
							    head[i]) {
						if (entry->flags & RKNPU_JOB_ASYNC)
							continue;
						entry->flags |= RKNPU_JOB_STALLED;
						qn++;
					}
					spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
					/* one wake serves the whole queue: every waiter sleeps on this
					 * same job_done_wq and re-tests its OWN job->flags */
					wake_up(&subcore_data->job_done_wq);
					rknpu_wd_aborts++;
					rknpu_wd_queued_aborts += qn;
					LOG_ERROR("RKNPU: core %d FAST-ABORT after %u ms, +%u queued behind it (would have waited %u ms x3)\n",
						  i, rknpu_wd_abort_ms, qn,
						  job->args->timeout);
				}
			}
		} else {
			subcore_data->wd_last = cnt;
			subcore_data->wd_flat = 0;
		}
	}

	mutex_unlock(&rknpu_dev->power_lock);
}

/* Cadence only — NO MMIO here. Self-disarms once no core has an outstanding job. */
static enum hrtimer_restart rknpu_wd_handler(struct hrtimer *timer)
{
	struct rknpu_device *rknpu_dev =
		container_of(timer, struct rknpu_device, wd_timer);
	unsigned long flags;
	int i, active = 0;

	rknpu_wd_ticks++;

	spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
	for (i = 0; i < rknpu_dev->config->num_irqs; i++)
		if (rknpu_dev->subcore_datas[i].job)
			active++;
	spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

	if (!active) {
		rknpu_dev->wd_on = 0;
		return HRTIMER_NORESTART;
	}
	/* a no-op if the previous sample is still queued, which self-throttles */
	queue_work(system_wq, &rknpu_dev->wd_work);

	hrtimer_forward_now(timer, rknpu_dev->wd_kt);
	return HRTIMER_RESTART;
}

/* Armed from the submit path; self-disarms in the handler once no core has a job. Re-arming an
 * already-running hrtimer is harmless, so the wd_on race is benign. */
void rknpu_wd_arm(struct rknpu_device *rknpu_dev)
{
	if (!rknpu_wd_period_us || rknpu_dev->wd_on)
		return;
	rknpu_dev->wd_on = 1;
	rknpu_dev->wd_kt = ktime_set(0, (u64)rknpu_wd_period_us * 1000);
	hrtimer_start(&rknpu_dev->wd_timer, rknpu_dev->wd_kt, HRTIMER_MODE_REL);
}

static void rknpu_init_timer(struct rknpu_device *rknpu_dev)
{
	rknpu_dev->kt = ktime_set(0, RKNPU_LOAD_INTERVAL);
	hrtimer_init(&rknpu_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rknpu_dev->timer.function = hrtimer_handler;
	hrtimer_start(&rknpu_dev->timer, rknpu_dev->kt, HRTIMER_MODE_REL);
	/* #patch: watchdog timer is initialised but NOT started — armed on submit */
	hrtimer_init(&rknpu_dev->wd_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rknpu_dev->wd_timer.function = rknpu_wd_handler;
	rknpu_dev->wd_on = 0;
	INIT_WORK(&rknpu_dev->wd_work, rknpu_wd_work_fn);
}

static void rknpu_cancel_timer(struct rknpu_device *rknpu_dev)
{
	hrtimer_cancel(&rknpu_dev->timer);
	hrtimer_cancel(&rknpu_dev->wd_timer);   /* #patch */
	cancel_work_sync(&rknpu_dev->wd_work);  /* #patch */
}

static bool rknpu_is_iommu_enable(struct device *dev)
{
	struct device_node *iommu = NULL;

	iommu = of_parse_phandle(dev->of_node, "iommus", 0);
	if (!iommu) {
		LOG_DEV_INFO(
			dev,
			"rknpu iommu device-tree entry not found!, using non-iommu mode\n");
		return false;
	}

	if (!of_device_is_available(iommu)) {
		LOG_DEV_INFO(dev,
			     "rknpu iommu is disabled, using non-iommu mode\n");
		of_node_put(iommu);
		return false;
	}
	of_node_put(iommu);

	LOG_DEV_INFO(dev, "rknpu iommu is enabled, using iommu mode\n");

	return true;
}

#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
static int drm_fake_dev_register(struct rknpu_device *rknpu_dev)
{
	const struct platform_device_info rknpu_dev_info = {
		.name = "rknpu_dev",
		.id = PLATFORM_DEVID_AUTO,
		.dma_mask = rknpu_dev->config->dma_mask,
	};
	struct platform_device *pdev = NULL;
	int ret = -EINVAL;

	pdev = platform_device_register_full(&rknpu_dev_info);
	if (pdev) {
		ret = of_dma_configure(&pdev->dev, NULL, true);
		if (ret) {
			platform_device_unregister(pdev);
			pdev = NULL;
		}
	}

	rknpu_dev->fake_dev = pdev ? &pdev->dev : NULL;

	return ret;
}

static void drm_fake_dev_unregister(struct rknpu_device *rknpu_dev)
{
	struct platform_device *pdev = NULL;

	if (!rknpu_dev->fake_dev)
		return;

	pdev = to_platform_device(rknpu_dev->fake_dev);

	platform_device_unregister(pdev);
}

static int rknpu_drm_probe(struct rknpu_device *rknpu_dev)
{
	struct device *dev = rknpu_dev->dev;
	struct drm_device *drm_dev = NULL;
	int ret = -EINVAL;

	drm_dev = drm_dev_alloc(&rknpu_drm_driver, dev);
	if (IS_ERR(drm_dev))
		return PTR_ERR(drm_dev);

	/* register the DRM device */
	ret = drm_dev_register(drm_dev, 0);
	if (ret < 0)
		goto err_free_drm;

	drm_dev->dev_private = rknpu_dev;
	rknpu_dev->drm_dev = drm_dev;

	drm_fake_dev_register(rknpu_dev);

	return 0;

err_free_drm:
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	drm_dev_put(drm_dev);
#else
	drm_dev_unref(drm_dev);
#endif

	return ret;
}

static void rknpu_drm_remove(struct rknpu_device *rknpu_dev)
{
	struct drm_device *drm_dev = rknpu_dev->drm_dev;

	drm_fake_dev_unregister(rknpu_dev);

	drm_dev_unregister(drm_dev);

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	drm_dev_put(drm_dev);
#else
	drm_dev_unref(drm_dev);
#endif
}
#endif

static int rknpu_power_on(struct rknpu_device *rknpu_dev)
{
	struct device *dev = rknpu_dev->dev;
	int ret = -EINVAL;

#ifndef FPGA_PLATFORM
	if (rknpu_dev->vdd) {
		ret = regulator_enable(rknpu_dev->vdd);
		if (ret) {
			LOG_DEV_ERROR(
				dev,
				"failed to enable vdd reg for rknpu, ret: %d\n",
				ret);
			return ret;
		}
	}

	if (rknpu_dev->mem) {
		ret = regulator_enable(rknpu_dev->mem);
		if (ret) {
			LOG_DEV_ERROR(
				dev,
				"failed to enable mem reg for rknpu, ret: %d\n",
				ret);
			return ret;
		}
	}
#endif

	ret = clk_bulk_prepare_enable(rknpu_dev->num_clks, rknpu_dev->clks);
	if (ret) {
		LOG_DEV_ERROR(dev, "failed to enable clk for rknpu, ret: %d\n",
			      ret);
		return ret;
	}

#ifndef FPGA_PLATFORM
	rknpu_devfreq_lock(rknpu_dev);
#endif

	if (rknpu_dev->multiple_domains) {
		if (rknpu_dev->genpd_dev_npu0) {
#if KERNEL_VERSION(5, 5, 0) < LINUX_VERSION_CODE
			ret = pm_runtime_resume_and_get(
				rknpu_dev->genpd_dev_npu0);
#else
			ret = pm_runtime_get_sync(rknpu_dev->genpd_dev_npu0);
#endif
			if (ret < 0) {
				LOG_DEV_ERROR(
					dev,
					"failed to get pm runtime for npu0, ret: %d\n",
					ret);
				goto out;
			}
		}
		if (rknpu_dev->genpd_dev_npu1) {
#if KERNEL_VERSION(5, 5, 0) < LINUX_VERSION_CODE
			ret = pm_runtime_resume_and_get(
				rknpu_dev->genpd_dev_npu1);
#else
			ret = pm_runtime_get_sync(rknpu_dev->genpd_dev_npu1);
#endif
			if (ret < 0) {
				LOG_DEV_ERROR(
					dev,
					"failed to get pm runtime for npu1, ret: %d\n",
					ret);
				goto out;
			}
		}
		if (rknpu_dev->genpd_dev_npu2) {
#if KERNEL_VERSION(5, 5, 0) < LINUX_VERSION_CODE
			ret = pm_runtime_resume_and_get(
				rknpu_dev->genpd_dev_npu2);
#else
			ret = pm_runtime_get_sync(rknpu_dev->genpd_dev_npu2);
#endif
			if (ret < 0) {
				LOG_DEV_ERROR(
					dev,
					"failed to get pm runtime for npu2, ret: %d\n",
					ret);
				goto out;
			}
		}
	}
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		LOG_DEV_ERROR(dev,
			      "failed to get pm runtime for rknpu, ret: %d\n",
			      ret);
	}

	if (rknpu_dev->config->state_init != NULL)
		rknpu_dev->config->state_init(rknpu_dev);

out:
#ifndef FPGA_PLATFORM
	rknpu_devfreq_unlock(rknpu_dev);
#endif

	return ret;
}

static int rknpu_power_off(struct rknpu_device *rknpu_dev)
{
	struct device *dev = rknpu_dev->dev;

#ifndef FPGA_PLATFORM
	int ret;
	bool val;

	rknpu_devfreq_lock(rknpu_dev);
#endif

	pm_runtime_put_sync(dev);

	if (rknpu_dev->multiple_domains) {
#ifndef FPGA_PLATFORM
		/*
		 * Because IOMMU's runtime suspend callback is asynchronous,
		 * So it may be executed after the NPU is turned off after PD/CLK/VD,
		 * and the runtime suspend callback has a register access.
		 * If the PD/VD/CLK is closed, the register access will crash.
		 * As a workaround, it's safe to close pd stuff until iommu disabled.
		 * If pm runtime framework can handle this issue in the future, remove
		 * this.
		 */
		ret = readx_poll_timeout(rockchip_iommu_is_enabled, dev, val,
					 !val, NPU_MMU_DISABLED_POLL_PERIOD_US,
					 NPU_MMU_DISABLED_POLL_TIMEOUT_US);
		if (ret) {
			LOG_DEV_ERROR(dev, "iommu still enabled\n");
			pm_runtime_get_sync(dev);
			rknpu_devfreq_unlock(rknpu_dev);
			return ret;
		}
#else
		if (rknpu_dev->iommu_en)
			msleep(20);
#endif
		if (rknpu_dev->genpd_dev_npu2)
			pm_runtime_put_sync(rknpu_dev->genpd_dev_npu2);
		if (rknpu_dev->genpd_dev_npu1)
			pm_runtime_put_sync(rknpu_dev->genpd_dev_npu1);
		if (rknpu_dev->genpd_dev_npu0)
			pm_runtime_put_sync(rknpu_dev->genpd_dev_npu0);
	}

#ifndef FPGA_PLATFORM
	rknpu_devfreq_unlock(rknpu_dev);
#endif

	clk_bulk_disable_unprepare(rknpu_dev->num_clks, rknpu_dev->clks);

#ifndef FPGA_PLATFORM
	if (rknpu_dev->vdd)
		regulator_disable(rknpu_dev->vdd);

	if (rknpu_dev->mem)
		regulator_disable(rknpu_dev->mem);
#endif

	return 0;
}

static int rknpu_register_irq(struct platform_device *pdev,
			      struct rknpu_device *rknpu_dev)
{
	const struct rknpu_config *config = rknpu_dev->config;
	struct device *dev = &pdev->dev;
#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
	struct resource *res;
#endif
	int i, ret, irq;

#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					   config->irqs[0].name);
	if (res) {
		/* there are irq names in dts */
		for (i = 0; i < config->num_irqs; i++) {
			irq = platform_get_irq_byname(pdev,
						      config->irqs[i].name);
			if (irq < 0) {
				LOG_DEV_ERROR(dev, "no npu %s in dts\n",
					      config->irqs[i].name);
				return irq;
			}

			ret = devm_request_irq(dev, irq,
					       config->irqs[i].irq_hdl,
					       IRQF_SHARED, dev_name(dev),
					       rknpu_dev);
			if (ret < 0) {
				LOG_DEV_ERROR(dev, "request %s failed: %d\n",
					      config->irqs[i].name, ret);
				return ret;
			}
		}
	} else {
		/* no irq names in dts */
		irq = platform_get_irq(pdev, 0);
		if (irq < 0) {
			LOG_DEV_ERROR(dev, "no npu irq in dts\n");
			return irq;
		}

		ret = devm_request_irq(dev, irq, rknpu_core0_irq_handler,
				       IRQF_SHARED, dev_name(dev), rknpu_dev);
		if (ret < 0) {
			LOG_DEV_ERROR(dev, "request irq failed: %d\n", ret);
			return ret;
		}
	}
#else
	/* there are irq names in dts */
	for (i = 0; i < config->num_irqs; i++) {
		irq = platform_get_irq_byname(pdev, config->irqs[i].name);
		if (irq < 0) {
			irq = platform_get_irq(pdev, i);
			if (irq < 0) {
				LOG_DEV_ERROR(dev, "no npu %s in dts\n",
					      config->irqs[i].name);
				return irq;
			}
		}

		ret = devm_request_irq(dev, irq, config->irqs[i].irq_hdl,
				       IRQF_SHARED, dev_name(dev), rknpu_dev);
		if (ret < 0) {
			LOG_DEV_ERROR(dev, "request %s failed: %d\n",
				      config->irqs[i].name, ret);
			return ret;
		}
	}
#endif

	return 0;
}

static int rknpu_find_sram_resource(struct rknpu_device *rknpu_dev)
{
	struct device *dev = rknpu_dev->dev;
	struct device_node *sram_node = NULL;
	struct resource sram_res;
	uint32_t sram_size = 0;
	int ret = -EINVAL;

	/* get sram device node */
	sram_node = of_parse_phandle(dev->of_node, "rockchip,sram", 0);
	rknpu_dev->sram_size = 0;
	if (!sram_node)
		return -EINVAL;

	/* get sram start and size */
	ret = of_address_to_resource(sram_node, 0, &sram_res);
	of_node_put(sram_node);
	if (ret)
		return ret;

	/* check sram start and size is PAGE_SIZE align */
	rknpu_dev->sram_start = round_up(sram_res.start, PAGE_SIZE);
	rknpu_dev->sram_end = round_down(
		sram_res.start + resource_size(&sram_res), PAGE_SIZE);
	if (rknpu_dev->sram_end <= rknpu_dev->sram_start) {
		LOG_DEV_WARN(
			dev,
			"invalid sram resource, sram start %pa, sram end %pa\n",
			&rknpu_dev->sram_start, &rknpu_dev->sram_end);
		return -EINVAL;
	}

	sram_size = rknpu_dev->sram_end - rknpu_dev->sram_start;

	rknpu_dev->sram_base_io =
		devm_ioremap(dev, rknpu_dev->sram_start, sram_size);
	if (IS_ERR(rknpu_dev->sram_base_io)) {
		LOG_DEV_ERROR(dev, "failed to remap sram base io!\n");
		rknpu_dev->sram_base_io = NULL;
	}

	rknpu_dev->sram_size = sram_size;

	LOG_DEV_INFO(dev, "sram region: [%pa, %pa), sram size: %#x\n",
		     &rknpu_dev->sram_start, &rknpu_dev->sram_end,
		     rknpu_dev->sram_size);

	return 0;
}

static int rknpu_find_nbuf_resource(struct rknpu_device *rknpu_dev)
{
	struct device *dev = rknpu_dev->dev;

	if (rknpu_dev->config->nbuf_size == 0)
		return -EINVAL;

	rknpu_dev->nbuf_start = rknpu_dev->config->nbuf_phyaddr;
	rknpu_dev->nbuf_size = rknpu_dev->config->nbuf_size;
	rknpu_dev->nbuf_base_io =
		devm_ioremap(dev, rknpu_dev->nbuf_start, rknpu_dev->nbuf_size);
	if (IS_ERR(rknpu_dev->nbuf_base_io)) {
		LOG_DEV_ERROR(dev, "failed to remap nbuf base io!\n");
		rknpu_dev->nbuf_base_io = NULL;
	}

	rknpu_dev->nbuf_end = rknpu_dev->nbuf_start + rknpu_dev->nbuf_size;

	LOG_DEV_INFO(dev, "nbuf region: [%pa, %pa), nbuf size: %#x\n",
		     &rknpu_dev->nbuf_start, &rknpu_dev->nbuf_end,
		     rknpu_dev->nbuf_size);

	return 0;
}

static int rknpu_get_invalid_core_mask(struct device *dev)
{
	int ret = 0;
	u8 invalid_core_mask = 0;

	if (of_property_match_string(dev->of_node, "nvmem-cell-names",
				     "cores") >= 0) {
		ret = rockchip_nvmem_cell_read_u8(dev->of_node, "cores",
						  &invalid_core_mask);
		/* The default valid npu cores for RK3583 are core0 and core1 */
		invalid_core_mask |= RKNPU_CORE2_MASK;
		if (ret) {
			LOG_DEV_ERROR(
				dev,
				"failed to get specification_serial_number\n");
			return invalid_core_mask;
		}
	}

	return (int)invalid_core_mask;
}

static int rknpu_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct rknpu_device *rknpu_dev = NULL;
	struct device *dev = &pdev->dev;
	struct device *virt_dev = NULL;
	const struct of_device_id *match = NULL;
	const struct rknpu_config *config = NULL;
	int ret = -EINVAL, i = 0;

	if (!pdev->dev.of_node) {
		LOG_DEV_ERROR(dev, "rknpu device-tree data is missing!\n");
		return -ENODEV;
	}

	match = of_match_device(rknpu_of_match, dev);
	if (!match) {
		LOG_DEV_ERROR(dev, "rknpu device-tree entry is missing!\n");
		return -ENODEV;
	}

	rknpu_dev = devm_kzalloc(dev, sizeof(*rknpu_dev), GFP_KERNEL);
	if (!rknpu_dev) {
		LOG_DEV_ERROR(dev, "failed to allocate rknpu device!\n");
		return -ENOMEM;
	}

	config = of_device_get_match_data(dev);
	if (!config)
		return -EINVAL;

	if (match->data == (void *)&rk3588_rknpu_config) {
		int invalid_core_mask = rknpu_get_invalid_core_mask(dev);
		/* The default valid npu cores for RK3583 are core0 and core1 */
		if (invalid_core_mask & RKNPU_CORE2_MASK) {
			if ((invalid_core_mask & RKNPU_CORE0_MASK) ||
			    (invalid_core_mask & RKNPU_CORE1_MASK)) {
				LOG_DEV_ERROR(
					dev,
					"rknpu core invalid, invalid core mask: %#x\n",
					invalid_core_mask);
				return -ENODEV;
			}
			config = &rk3583_rknpu_config;
		}
	}

	rknpu_dev->config = config;
	rknpu_dev->dev = dev;
	dev_set_drvdata(dev, rknpu_dev);

	rknpu_dev->iommu_en = rknpu_is_iommu_enable(dev);
	if (rknpu_dev->iommu_en) {
		rknpu_dev->iommu_group = iommu_group_get(dev);
		if (!rknpu_dev->iommu_group)
			return -EINVAL;
	} else {
		/* Initialize reserved memory resources */
		ret = of_reserved_mem_device_init(dev);
		if (!ret) {
			LOG_DEV_INFO(
				dev,
				"initialize reserved memory for rknpu device!\n");
		}
	}

	rknpu_dev->bypass_irq_handler = bypass_irq_handler;
	rknpu_dev->bypass_soft_reset = bypass_soft_reset;

	rknpu_reset_get(rknpu_dev);

	rknpu_dev->num_clks = devm_clk_bulk_get_all(dev, &rknpu_dev->clks);
	if (rknpu_dev->num_clks < 1) {
		LOG_DEV_ERROR(dev, "failed to get clk source for rknpu\n");
#ifndef FPGA_PLATFORM
		return -ENODEV;
#endif
	}

#ifndef FPGA_PLATFORM
	rknpu_dev->vdd = devm_regulator_get_optional(dev, "rknpu");
	if (IS_ERR(rknpu_dev->vdd)) {
		if (PTR_ERR(rknpu_dev->vdd) != -ENODEV) {
			ret = PTR_ERR(rknpu_dev->vdd);
			LOG_DEV_ERROR(
				dev,
				"failed to get vdd regulator for rknpu: %d\n",
				ret);
			return ret;
		}
		rknpu_dev->vdd = NULL;
	}

	rknpu_dev->mem = devm_regulator_get_optional(dev, "mem");
	if (IS_ERR(rknpu_dev->mem)) {
		if (PTR_ERR(rknpu_dev->mem) != -ENODEV) {
			ret = PTR_ERR(rknpu_dev->mem);
			LOG_DEV_ERROR(
				dev,
				"failed to get mem regulator for rknpu: %d\n",
				ret);
			return ret;
		}
		rknpu_dev->mem = NULL;
	}
#endif

	spin_lock_init(&rknpu_dev->lock);
	spin_lock_init(&rknpu_dev->irq_lock);
	mutex_init(&rknpu_dev->power_lock);
	mutex_init(&rknpu_dev->reset_lock);
	mutex_init(&rknpu_dev->domain_lock);
	for (i = 0; i < config->num_irqs; i++) {
		INIT_LIST_HEAD(&rknpu_dev->subcore_datas[i].todo_list);
		init_waitqueue_head(&rknpu_dev->subcore_datas[i].job_done_wq);
		rknpu_dev->subcore_datas[i].task_num = 0;
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			LOG_DEV_ERROR(
				dev,
				"failed to get memory resource for rknpu\n");
			return -ENXIO;
		}

		rknpu_dev->base[i] = devm_ioremap_resource(dev, res);
		if (PTR_ERR(rknpu_dev->base[i]) == -EBUSY) {
			rknpu_dev->base[i] = devm_ioremap(dev, res->start,
							  resource_size(res));
		}

		if (IS_ERR(rknpu_dev->base[i])) {
			LOG_DEV_ERROR(dev,
				      "failed to remap register for rknpu\n");
			return PTR_ERR(rknpu_dev->base[i]);
		}
	}

	if (config->bw_priority_length > 0) {
		rknpu_dev->bw_priority_base =
			devm_ioremap(dev, config->bw_priority_addr,
				     config->bw_priority_length);
		if (IS_ERR(rknpu_dev->bw_priority_base)) {
			LOG_DEV_ERROR(
				rknpu_dev->dev,
				"failed to remap bw priority register for rknpu\n");
			rknpu_dev->bw_priority_base = NULL;
		}
	}

	if (!rknpu_dev->bypass_irq_handler) {
		ret = rknpu_register_irq(pdev, rknpu_dev);
		if (ret)
			return ret;
	} else {
		LOG_DEV_WARN(dev, "bypass irq handler!\n");
	}

#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
	ret = rknpu_drm_probe(rknpu_dev);
	if (ret) {
		LOG_DEV_ERROR(dev, "failed to probe device for rknpu\n");
		return ret;
	}
#endif
#ifdef CONFIG_ROCKCHIP_RKNPU_DMA_HEAP
	rknpu_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	rknpu_dev->miscdev.name = "rknpu";
	rknpu_dev->miscdev.fops = &rknpu_fops;

	ret = misc_register(&rknpu_dev->miscdev);
	if (ret) {
		LOG_DEV_ERROR(dev, "cannot register miscdev (%d)\n", ret);
		return ret;
	}

	rknpu_dev->heap = rk_dma_heap_find("rk-dma-heap-cma");
	if (!rknpu_dev->heap) {
		LOG_DEV_ERROR(dev, "failed to find cma heap\n");
		return -ENOMEM;
	}
	rk_dma_heap_set_dev(dev);
	LOG_DEV_INFO(dev, "Initialized %s: v%d.%d.%d for %s\n", DRIVER_DESC,
		     DRIVER_MAJOR, DRIVER_MINOR, DRIVER_PATCHLEVEL,
		     DRIVER_DATE);
#endif

#ifdef CONFIG_ROCKCHIP_RKNPU_FENCE
	ret = rknpu_fence_context_alloc(rknpu_dev);
	if (ret) {
		LOG_DEV_ERROR(dev,
			      "failed to allocate fence context for rknpu\n");
		goto err_remove_drv;
	}
#endif

	platform_set_drvdata(pdev, rknpu_dev);

	pm_runtime_enable(dev);

	if (of_count_phandle_with_args(dev->of_node, "power-domains",
				       "#power-domain-cells") > 1) {
		virt_dev = dev_pm_domain_attach_by_name(dev, "npu0");
		if (!IS_ERR(virt_dev))
			rknpu_dev->genpd_dev_npu0 = virt_dev;
		virt_dev = dev_pm_domain_attach_by_name(dev, "npu1");
		if (!IS_ERR(virt_dev))
			rknpu_dev->genpd_dev_npu1 = virt_dev;
		if (config->num_irqs > 2) {
			virt_dev = dev_pm_domain_attach_by_name(dev, "npu2");
			if (!IS_ERR(virt_dev))
				rknpu_dev->genpd_dev_npu2 = virt_dev;
		}
		rknpu_dev->multiple_domains = true;
	}

	ret = rknpu_power_on(rknpu_dev);
	if (ret)
		goto err_remove_drv;

#ifndef FPGA_PLATFORM
	rknpu_devfreq_init(rknpu_dev);
#endif

	// set default power put delay to 3s
	rknpu_dev->power_put_delay = 3000;
	rknpu_dev->power_off_wq =
		create_freezable_workqueue("rknpu_power_off_wq");
	if (!rknpu_dev->power_off_wq) {
		LOG_DEV_ERROR(dev, "rknpu couldn't create power_off workqueue");
		ret = -ENOMEM;
		goto err_devfreq_remove;
	}
	INIT_DEFERRABLE_WORK(&rknpu_dev->power_off_work,
			     rknpu_power_off_delay_work);

	if (IS_ENABLED(CONFIG_NO_GKI) &&
	    IS_ENABLED(CONFIG_ROCKCHIP_RKNPU_SRAM) && rknpu_dev->iommu_en) {
		if (!rknpu_find_sram_resource(rknpu_dev)) {
			ret = rknpu_mm_create(rknpu_dev->sram_size, PAGE_SIZE,
					      &rknpu_dev->sram_mm);
			if (ret != 0)
				goto err_remove_wq;
		} else {
			LOG_DEV_WARN(dev, "could not find sram resource!\n");
		}
	}

	if (IS_ENABLED(CONFIG_NO_GKI) && rknpu_dev->iommu_en &&
	    rknpu_dev->config->nbuf_size > 0) {
		rknpu_find_nbuf_resource(rknpu_dev);
		if (rknpu_dev->config->cache_sgt_init != NULL)
			rknpu_dev->config->cache_sgt_init(rknpu_dev);
	}

	if (rknpu_dev->iommu_en)
		rknpu_iommu_init_domain(rknpu_dev);

	rknpu_power_off(rknpu_dev);
	atomic_set(&rknpu_dev->power_refcount, 0);
	atomic_set(&rknpu_dev->cmdline_power_refcount, 0);
	atomic_set(&rknpu_dev->iommu_domain_refcount, 0);

	rknpu_debugger_init(rknpu_dev);
	rknpu_init_timer(rknpu_dev);

	return 0;

err_remove_wq:
	destroy_workqueue(rknpu_dev->power_off_wq);

err_devfreq_remove:
#ifndef FPGA_PLATFORM
	rknpu_devfreq_remove(rknpu_dev);
#endif

err_remove_drv:
#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
	rknpu_drm_remove(rknpu_dev);
#endif
#ifdef CONFIG_ROCKCHIP_RKNPU_DMA_HEAP
	misc_deregister(&(rknpu_dev->miscdev));
#endif

	return ret;
}

static int rknpu_remove(struct platform_device *pdev)
{
	struct rknpu_device *rknpu_dev = platform_get_drvdata(pdev);
	int i = 0;

	cancel_delayed_work_sync(&rknpu_dev->power_off_work);
	destroy_workqueue(rknpu_dev->power_off_wq);

	rknpu_debugger_remove(rknpu_dev);
	rknpu_cancel_timer(rknpu_dev);

	if (rknpu_dev->config->cache_sgt_init != NULL) {
		for (i = 0; i < RKNPU_CACHE_SG_TABLE_NUM; i++) {
			if (rknpu_dev->cache_sgt[i]) {
				sg_free_table(rknpu_dev->cache_sgt[i]);
				kfree(rknpu_dev->cache_sgt[i]);
				rknpu_dev->cache_sgt[i] = NULL;
			}
		}
	}

	for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
		WARN_ON(rknpu_dev->subcore_datas[i].job);
		WARN_ON(!list_empty(&rknpu_dev->subcore_datas[i].todo_list));
	}

	if (IS_ENABLED(CONFIG_ROCKCHIP_RKNPU_SRAM) && rknpu_dev->sram_mm)
		rknpu_mm_destroy(rknpu_dev->sram_mm);

	if (rknpu_dev->iommu_en) {
		rknpu_iommu_free_domains(rknpu_dev);
		iommu_group_put(rknpu_dev->iommu_group);
	}

#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
	rknpu_drm_remove(rknpu_dev);
#endif
#ifdef CONFIG_ROCKCHIP_RKNPU_DMA_HEAP
	misc_deregister(&(rknpu_dev->miscdev));
#endif

#ifndef FPGA_PLATFORM
	rknpu_devfreq_remove(rknpu_dev);
#endif

	mutex_lock(&rknpu_dev->power_lock);
	if (atomic_read(&rknpu_dev->power_refcount) > 0)
		rknpu_power_off(rknpu_dev);
	mutex_unlock(&rknpu_dev->power_lock);

	if (rknpu_dev->multiple_domains) {
		if (rknpu_dev->genpd_dev_npu0)
			dev_pm_domain_detach(rknpu_dev->genpd_dev_npu0, true);
		if (rknpu_dev->genpd_dev_npu1)
			dev_pm_domain_detach(rknpu_dev->genpd_dev_npu1, true);
		if (rknpu_dev->genpd_dev_npu2)
			dev_pm_domain_detach(rknpu_dev->genpd_dev_npu2, true);
	}

	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifndef FPGA_PLATFORM
#ifdef CONFIG_PM_SLEEP
static int rknpu_suspend(struct device *dev)
{
	struct rknpu_device *rknpu_dev = dev_get_drvdata(dev);

	rknpu_power_get(rknpu_dev);

	return pm_runtime_force_suspend(dev);
}

static int rknpu_resume(struct device *dev)
{
	struct rknpu_device *rknpu_dev = dev_get_drvdata(dev);

	rknpu_power_put_delay(rknpu_dev);

	return pm_runtime_force_resume(dev);
}
#endif

static int rknpu_runtime_suspend(struct device *dev)
{
	return rknpu_devfreq_runtime_suspend(dev);
}

static int rknpu_runtime_resume(struct device *dev)
{
	return rknpu_devfreq_runtime_resume(dev);
}

static const struct dev_pm_ops rknpu_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rknpu_suspend, rknpu_resume) SET_RUNTIME_PM_OPS(
		rknpu_runtime_suspend, rknpu_runtime_resume, NULL)
};
#endif

static struct platform_driver rknpu_driver = {
	.probe = rknpu_probe,
	.remove = rknpu_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "RKNPU",
#ifndef FPGA_PLATFORM
		.pm = &rknpu_pm_ops,
#endif
		.of_match_table = of_match_ptr(rknpu_of_match),
	},
};

static int rknpu_init(void)
{
	return platform_driver_register(&rknpu_driver);
}

static void rknpu_exit(void)
{
	platform_driver_unregister(&rknpu_driver);
}

late_initcall(rknpu_init);
module_exit(rknpu_exit);

MODULE_DESCRIPTION("RKNPU driver");
MODULE_AUTHOR("Felix Zeng <felix.zeng@rock-chips.com>");
MODULE_ALIAS("rockchip-rknpu");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(RKNPU_GET_DRV_VERSION_STRING(DRIVER_MAJOR, DRIVER_MINOR,
					    DRIVER_PATCHLEVEL));
#if KERNEL_VERSION(5, 16, 0) < LINUX_VERSION_CODE
MODULE_IMPORT_NS(DMA_BUF);
#endif
