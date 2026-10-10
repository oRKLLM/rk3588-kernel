// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Felix Zeng <felix.zeng@rock-chips.com>
 */

#include <linux/delay.h>
#include <linux/iommu.h>

#include "rknpu_reset.h"
#include "rknpu_iommu.h"

/* #patchB8: provided by drivers/iommu/rockchip-iommu.c */
extern int rk_iommu_reprogram(struct device *dev);

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

/* #patch: PER-CORE soft reset. rknpu_soft_reset() asserts ALL SIX DT reset lines
 * ("srst_a0","srst_a1","srst_a2","srst_h0","srst_h1","srst_h2" — AXI+AHB per core),
 * sleeps 100 ms, and sets the global soft_reseting flag that makes rknpu_job_next()
 * bail for EVERY core. Recovering one stalled core therefore resets the whole NPU and
 * destroys the other two cores' warm state — most of the ~3.6 s a doorbell miss costs.
 *
 * The lines are per core, so reset only core N: indices N (AXI) and N+num_irqs (AHB).
 * No msleep, no global flag, no IOMMU re-attach — the other cores keep running. */
int rknpu_soft_reset_core(struct rknpu_device *rknpu_dev, int core)
{
#ifndef FPGA_PLATFORM
	int ret = 0, ia, ih;

	if (rknpu_dev->bypass_soft_reset)
		return 0;

	ia = core;
	ih = core + rknpu_dev->config->num_irqs;
	if (core < 0 || ih >= rknpu_dev->num_srsts)
		return -EINVAL;

	ret |= rknpu_reset_assert(rknpu_dev->srsts[ia]);
	ret |= rknpu_reset_assert(rknpu_dev->srsts[ih]);
	udelay(10);
	ret |= rknpu_reset_deassert(rknpu_dev->srsts[ia]);
	ret |= rknpu_reset_deassert(rknpu_dev->srsts[ih]);
	udelay(10);

	wake_up(&rknpu_dev->subcore_datas[core].job_done_wq);
	return ret;
#else
	return 0;
#endif
}

/* #patch: per-core reset + IOMMU re-programming — the minimal sequence measured to actually recover a
 * stalled core. Shared by the RKNPU_ACT_RESET_CORE ioctl and the progress watchdog so both paths behave
 * identically, mirroring how rknpu_soft_reset serves the device-wide action. */
int rknpu_reset_core_and_remap(struct rknpu_device *rknpu_dev, int core)
{
#ifndef FPGA_PLATFORM
	struct iommu_domain *domain;
	int ret;

	ret = rknpu_soft_reset_core(rknpu_dev, core);
	if (ret)
		return ret;

	if (!rknpu_dev->iommu_en)
		return 0;

	/* the re-attach must not race a concurrent dma_map; quiesce as rknpu_soft_reset does */
	if (!mutex_trylock(&rknpu_dev->reset_lock))
		return -EBUSY;
	rknpu_dev->soft_reseting = true;
	/* #patchB9: reprogram the LIVE domain, exactly as rknpu_soft_reset() now does (#patchB8).
	 *
	 * This is the PER-CORE twin of that path (RKNPU_ACT_RESET_CORE + the progress watchdog) and it
	 * carried the identical bug: iommu_get_domain_for_dev() returns the group's DEFAULT domain
	 * under the light switch, so the detach/attach below re-attached domain 0 while the driver
	 * still believed domain N was live. B8 fixed the device-wide reset and missed this one -- and
	 * this is the path the watchdog drives during a multi-domain run. */
	domain = rknpu_iommu_live_domain(rknpu_dev->dev);
	if (domain) {
		/* #patch46: serialize against domain switching.
		 *
		 * rknpu_soft_reset() holds reset_lock; rknpu_iommu_domain_get_and_switch() holds
		 * domain_lock. Different locks, so this detach/attach had NO mutual exclusion against
		 * a concurrent switch -- and a switch that reads iommu_get_domain_for_dev() while we
		 * are detached sees a domain that does not match rknpu_dev->iommu_domains[], bails
		 * with "mismatch domain get from iommu_get_domain_for_dev", and fails the caller.
		 *
		 * Measured: 1600 mismatch lines in a single 300 s run once the fast-abort raised the
		 * reset rate ~30x. Low reset rates hid it; it was always latent.
		 *
		 * Ordering is reset_lock -> domain_lock. Safe: no path takes reset_lock while holding
		 * domain_lock (rknpu_iommu_switch_domain does not reset). */
		mutex_lock(&rknpu_dev->domain_lock);
		{
			int rp = rk_iommu_reprogram(rknpu_dev->dev);

			if (rp)
				LOG_DEV_ERROR(rknpu_dev->dev,
					      "failed to reprogram iommu after per-core reset: %d\n", rp);
		}
		mutex_unlock(&rknpu_dev->domain_lock);
	}
	rknpu_dev->soft_reseting = false;
	mutex_unlock(&rknpu_dev->reset_lock);
	/* #patch72: promote anything queued while dispatch was disabled -- see rknpu_job_redrive.
	 * AFTER the unlock: a commit can lead back into a reset, whose mutex_trylock would then fail
	 * and silently skip a reset that was needed. */
	rknpu_job_redrive(rknpu_dev);
	return 0;
#else
	return 0;
#endif
}

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

	/* #patch: a failed trylock used to `return 0` — reporting SUCCESS without resetting anything.
	 * The caller (RKNPU_ACT_RESET from userspace recovery) then believes the NPU was reset when it
	 * was not, and spins on a still-wedged core. That was harmless while this was the only resetter;
	 * it stopped being harmless once the per-core recovery path also takes reset_lock, at which point
	 * userspace recovery silently became a no-op and the workload ran ~10x slower.
	 * Wait for the in-flight reset instead, so an ACT_RESET always means a reset actually happened.
	 * Legal: this function already msleep()s, so it is process context. */
	if (!mutex_trylock(&rknpu_dev->reset_lock))
		mutex_lock(&rknpu_dev->reset_lock);

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
		/* #patch72: clear the flag on the way out too. Leaving it set disables dispatch for the
		 * WHOLE DEVICE permanently -- rknpu_job_next returns immediately for every core -- so a
		 * failed reset does not degrade the NPU, it silently stops it. */
		rknpu_dev->soft_reseting = false;
		mutex_unlock(&rknpu_dev->reset_lock);
		rknpu_job_redrive(rknpu_dev);
		return ret;
	}

	/* #patchB8: reprogram the MMU for the LIVE domain, not the core's default.
	 *
	 * This used to be iommu_detach_device(iommu_get_domain_for_dev(dev)) + iommu_attach_device(),
	 * which was only ever right because the old multi-domain code overwrote
	 * iommu_group->default_domain on every switch. With the light switch (#patchB1/B2) the IOMMU
	 * core is not in the loop, so iommu_get_domain_for_dev() returns the group's DEFAULT domain
	 * (domain 0) -- and this sequence re-attached domain 0 while rknpu_dev->iommu_domain_id still
	 * said domain N. Every IOVA the next job used then resolved against the wrong page table, so
	 * the job was committed and never completed: no IRQ, no error, an output buffer left holding
	 * its seed. That is the "first int8 op after an ACT_RESET is silently dropped" bug.
	 *
	 * rk_iommu_reprogram() re-enables the MMU from iommu->domain, which the light switch keeps
	 * pointed at the live domain, so the correct page table is restored. */
	if (rknpu_dev->iommu_en) {
		int rp = rk_iommu_reprogram(rknpu_dev->dev);

		if (rp)
			LOG_DEV_ERROR(rknpu_dev->dev,
				      "failed to reprogram iommu after soft reset: %d\n", rp);
	}
	(void)domain;

	rknpu_dev->soft_reseting = false;

	if (rknpu_dev->config->state_init != NULL)
		rknpu_dev->config->state_init(rknpu_dev);

	mutex_unlock(&rknpu_dev->reset_lock);

	/* #patch72: promote anything queued while dispatch was disabled -- see rknpu_job_redrive.
	 * AFTER state_init, because committing a job before it would program a half-initialised
	 * block, and after the unlock, because a commit can lead back into rknpu_soft_reset() whose
	 * mutex_trylock would then fail and silently skip a reset that was needed. */
	rknpu_job_redrive(rknpu_dev);
#endif

	return 0;
}
