// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Felix Zeng <felix.zeng@rock-chips.com>
 */

#include <linux/delay.h>
#include <linux/iommu.h>

#include "rknpu_job.h"
#include "rknpu_reset.h"

#ifndef FPGA_PLATFORM
static inline struct reset_control *rknpu_reset_control_get(struct device *dev,
							    const char *name)
{
	struct reset_control *rst = NULL;

	rst = devm_reset_control_get(dev, name);
	if (IS_ERR(rst))
		LOG_DEV_ERROR(dev,
			      "failed to get rknpu reset control: %s, %ld\n",
			      name, PTR_ERR(rst));

	return rst;
}
#endif

int rknpu_reset_get(struct rknpu_device *rknpu_dev)
{
#ifndef FPGA_PLATFORM
	int i = 0;
	int num_srsts = 0;

	num_srsts = of_count_phandle_with_args(rknpu_dev->dev->of_node,
					       "resets", "#reset-cells");
	if (num_srsts <= 0) {
		LOG_DEV_ERROR(rknpu_dev->dev,
			      "failed to get rknpu resets from dtb\n");
		return num_srsts;
	}

	rknpu_dev->srsts = devm_kcalloc(rknpu_dev->dev, num_srsts,
					sizeof(*rknpu_dev->srsts), GFP_KERNEL);
	if (!rknpu_dev->srsts)
		return -ENOMEM;

	for (i = 0; i < num_srsts; ++i) {
		rknpu_dev->srsts[i] = devm_reset_control_get_exclusive_by_index(
			rknpu_dev->dev, i);
		if (IS_ERR(rknpu_dev->srsts[i])) {
			rknpu_dev->num_srsts = i;
			return PTR_ERR(rknpu_dev->srsts[i]);
		}
	}

	rknpu_dev->num_srsts = num_srsts;

	return num_srsts;
#endif

	return 0;
}

#ifndef FPGA_PLATFORM
static int rknpu_reset_assert(struct reset_control *rst)
{
	int ret = -EINVAL;

	if (!rst)
		return -EINVAL;

	ret = reset_control_assert(rst);
	if (ret < 0) {
		LOG_ERROR("failed to assert rknpu reset: %d\n", ret);
		return ret;
	}

	return 0;
}

static int rknpu_reset_deassert(struct reset_control *rst)
{
	int ret = -EINVAL;

	if (!rst)
		return -EINVAL;

	ret = reset_control_deassert(rst);
	if (ret < 0) {
		LOG_ERROR("failed to deassert rknpu reset: %d\n", ret);
		return ret;
	}

	return 0;
}
#endif

int rknpu_soft_reset(struct rknpu_device *rknpu_dev)
{
#ifndef FPGA_PLATFORM
	struct iommu_domain *domain = NULL;
	struct rknpu_subcore_data *subcore_data = NULL;
	int ret = 0, i = 0;

	if (rknpu_dev->bypass_soft_reset) {
		LOG_WARN("bypass soft reset\n");
		return 0;
	}

	if (!mutex_trylock(&rknpu_dev->reset_lock))
		return 0;

	rknpu_dev->soft_reseting = true;

	msleep(100);

	for (i = 0; i < rknpu_dev->config->num_irqs; ++i) {
		subcore_data = &rknpu_dev->subcore_datas[i];
		wake_up(&subcore_data->job_done_wq);
	}

	LOG_INFO("soft reset, num: %d\n", rknpu_dev->num_srsts);

	for (i = 0; i < rknpu_dev->num_srsts; ++i)
		ret |= rknpu_reset_assert(rknpu_dev->srsts[i]);

	udelay(10);

	for (i = 0; i < rknpu_dev->num_srsts; ++i)
		ret |= rknpu_reset_deassert(rknpu_dev->srsts[i]);

	udelay(10);

	if (ret) {
		LOG_DEV_ERROR(rknpu_dev->dev,
			      "failed to soft reset for rknpu: %d\n", ret);
		/*
		 * Clear the flag on the way out as well. Leaving it set disables dispatch for the whole
		 * device permanently -- rknpu_job_next() returns immediately for every core, so a failed
		 * reset does not degrade the NPU, it silently stops it.
		 */
		rknpu_dev->soft_reseting = false;
		mutex_unlock(&rknpu_dev->reset_lock);
		rknpu_job_redrive(rknpu_dev);
		return ret;
	}

	if (rknpu_dev->iommu_en)
		domain = iommu_get_domain_for_dev(rknpu_dev->dev);

	if (domain) {
		iommu_detach_device(domain, rknpu_dev->dev);
		iommu_attach_device(domain, rknpu_dev->dev);
	}

	rknpu_dev->soft_reseting = false;

	if (rknpu_dev->config->state_init != NULL)
		rknpu_dev->config->state_init(rknpu_dev);

	mutex_unlock(&rknpu_dev->reset_lock);

	/*
	 * Anything submitted while dispatch was disabled is still sitting on todo_list with nothing left
	 * to promote it. Do it here, after state_init has reconfigured the block -- committing a job
	 * before that would program a half-initialised device -- and after dropping reset_lock, because
	 * a commit can lead back into rknpu_soft_reset(), whose mutex_trylock() would then fail and
	 * silently skip a reset that was needed.
	 */
	rknpu_job_redrive(rknpu_dev);
#endif

	return 0;
}
