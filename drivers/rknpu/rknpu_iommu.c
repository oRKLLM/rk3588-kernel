// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Felix Zeng <felix.zeng@rock-chips.com>
 */

#include <linux/dma-map-ops.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include "rknpu_iommu.h"

/* #patch30 diagnostics */
unsigned long rknpu_dbg_switch_inflight;
unsigned long rknpu_dbg_switch_idle;
int rknpu_dbg_switch_verbose;
module_param_named(dbg_switch_inflight, rknpu_dbg_switch_inflight, ulong, 0444);
module_param_named(dbg_switch_idle, rknpu_dbg_switch_idle, ulong, 0444);
module_param_named(dbg_switch_verbose, rknpu_dbg_switch_verbose, int, 0644);

#define RKNPU_SWITCH_DOMAIN_WAIT_TIME_MS 6000


/* #patch59: what RATCHETS across runs?
 *
 * The mismatch wedge is cumulative — it appears on roughly the 4th run of a boot and clears only on
 * reboot, so something is persistent kernel state. dbg_domain_underflow stays 0, so it is not the
 * refcount going negative. The two remaining candidates are a leaked domain REFERENCE (refcount
 * never returns to 0, so every switch has to wait) and a leaked MAPPING (IOVA space exhausted, so
 * dma map eventually fails — which is the observed proximate cause,
 * "rknpu_gem_get_pages: dma map <size> fail").
 *
 * Sample all three after each run: whichever monotonically climbs is the leak. refcount_now is a
 * shadow of the atomic, updated wherever it is touched, so it can be read from sysfs. */
unsigned long rknpu_dbg_refcnt_now;
module_param_named(dbg_refcnt_now, rknpu_dbg_refcnt_now, ulong, 0444);
/* #patch69: outstanding IOVA, and the destroy-path leak that is suspected of holding it.
 *
 * REINIT frees domains 1..15 but NOT domain 0 (the IOMMU core's default), so anything leaked there
 * survives everything short of a reboot — and the wedge presents as dma map failures in domain 0.
 * The suspected holder is rknpu_gem_object_destroy(), which bails after 3 failed domain switches and
 * returns WITHOUT freeing, leaking the object and its IOVA. Count the bytes in and out, and the bails,
 * and see whether they ratchet together across runs. */
unsigned long rknpu_dbg_iova_alloc_kb, rknpu_dbg_iova_free_kb;
module_param_named(dbg_iova_alloc_kb, rknpu_dbg_iova_alloc_kb, ulong, 0444);
module_param_named(dbg_iova_free_kb, rknpu_dbg_iova_free_kb, ulong, 0444);
unsigned long rknpu_dbg_map_n, rknpu_dbg_unmap_n;
module_param_named(dbg_map_n, rknpu_dbg_map_n, ulong, 0444);
module_param_named(dbg_unmap_n, rknpu_dbg_unmap_n, ulong, 0444);


/* #patch51: distinguish "this DOMAIN is bad" from "the first commit after a SWITCH is bad".
 * Per-domain stall rate alone cannot separate them: a domain that is visited briefly has a much
 * larger FRACTION of its commits sitting immediately after a switch, so it would look bad either
 * way. Reset on every switch, incremented per commit; logged per commit so the two can be
 * disentangled by controlling for it. */
unsigned int rknpu_commits_since_switch;
/* #patch52: COUNT vs TIME. sinceswitch alone cannot tell "the first N commits are poisoned" from
 * "there is a settling window of T microseconds", because the first jobs into a new domain arrive in
 * a burst -- the two describe the same events. Stamp the switch so each commit can also record how
 * long ago it happened; then the stall boundary can be checked against both axes independently.
 * They imply different fixes: a count means prime/quiesce, a time means a cooldown. */
ktime_t rknpu_last_switch_time;
/* #patch53 */
/* #patch54: ESCALATION for the domain-switch livelock.
 *
 * get_and_switch() only proceeds when iommu_domain_refcount reads 0. A job that never completes
 * pins that reference, so every switch then times out after RKNPU_SWITCH_DOMAIN_WAIT_TIME_MS (6 s)
 * and returns -EINVAL -- and userspace simply retries, forever. Nothing breaks the loop:
 * rknpu_job_timeout_clean() is only invoked when a NEW submit arrives, which is exactly what cannot
 * happen while allocation is failing.
 *
 * Measured cost of having no bound: a run wedged the board hard enough to need a power cycle, with
 * "switch iommu domain time out" repeating every 6 s and one job abort at 184 s. None of the kernel's
 * own detectors can see it -- panic_on_oops needs a fault, softlockup needs a spinning CPU, and
 * hung_task needs CONTINUOUS uninterruptible sleep whereas this loop wakes every 6-20 s. It is a
 * livelock, invisible by construction.
 *
 * So bound it here: after N consecutive timeouts, reap the stuck jobs, which releases the references
 * they hold. Reaping (not forcing the refcount to 0) is deliberate -- if a job really is still live,
 * zeroing the count would let a switch happen underneath it and corrupt its translations. */
static unsigned int rknpu_switch_escalate = 3;
module_param_named(switch_escalate, rknpu_switch_escalate, uint, 0644);
MODULE_PARM_DESC(switch_escalate, "consecutive domain-switch timeouts before reaping stuck jobs (0=off)");
static unsigned int rknpu_switch_consec;
unsigned long rknpu_switch_escalations;
module_param_named(switch_escalations, rknpu_switch_escalations, ulong, 0444);
void rknpu_reap_all_cores(struct rknpu_device *rknpu_dev);

unsigned int rknpu_dom_cooldown_us;
module_param_named(dom_cooldown_us, rknpu_dom_cooldown_us, uint, 0644);
MODULE_PARM_DESC(dom_cooldown_us, "settle this many us after an iommu domain switch (0=off)");
unsigned long rknpu_dom_cooldowns;
module_param_named(dom_cooldowns, rknpu_dom_cooldowns, ulong, 0444);

/* #patch33: never dereference a NULL iommu domain.
 *
 * rknpu_iommu_switch_domain() does detach(src) then attach(dst); if the attach fails it tries to
 * re-attach src, and if THAT fails too the device is left attached to NO domain. The switch does
 * fail in practice: a stalled job cannot be drained, so the switch times out
 * ("switch iommu domain time out") and the abort path runs. The next GEM allocation then called
 * iommu_get_domain_for_dev() and dereferenced the NULL result one line later:
 *
 *   Unable to handle kernel NULL pointer dereference at virtual address 0000000000000040
 *   pc : rknpu_iommu_dma_map_sg+0x48   Comm: i4_widek_stall_
 *     rknpu_gem_object_create / rknpu_gem_create_ioctl
 *
 * which kills the board (this is a strong candidate for the long-standing "NPU wedge needs a
 * power cycle"). Turn an unrecoverable panic into an ordinary error return. */
/* #patchB2: provided by drivers/iommu/rockchip-iommu.c (#patchB1). Reprograms the MMU page-table
 * base in place -- stall / write DTE_ADDR / ZAP_CACHE / unstall -- with no force-reset and WITHOUT
 * involving the IOMMU core, so no detach/attach pair and no need to overwrite the group's default
 * domain. Consequence: iommu_get_domain_for_dev() no longer tracks our switches. */
extern int rk_iommu_switch_domain(struct device *dev, struct iommu_domain *domain);

/* #patchB2: THE LIVE DOMAIN, from the driver's own bookkeeping.
 *
 * This replaces every iommu_get_domain_for_dev() call in this driver. Since the light switch keeps
 * the IOMMU core out of the loop, the core keeps reporting the group's default domain (domain 0)
 * forever -- which was only ever "right" before because the vendor code overwrote
 * iommu_group->default_domain on each switch. That overwrite is gone; the driver owns the answer. */
struct iommu_domain *rknpu_iommu_live_domain(struct device *dev)
{
	struct rknpu_device *rknpu_dev = dev_get_drvdata(dev);
	int id;

	if (!rknpu_dev)
		return NULL;
	id = rknpu_dev->iommu_domain_id;
	if (id < 0 || id >= RKNPU_MAX_IOMMU_DOMAIN_NUM)
		return NULL;
	return rknpu_dev->iommu_domains[id];
}

static struct rknpu_iommu_dma_cookie *rknpu_iommu_cookie(struct device *dev,
							 const char *who)
{
	struct iommu_domain *domain = rknpu_iommu_live_domain(dev);

	if (!domain) {
		dev_err_ratelimited(dev, "%s: NPU attached to NO iommu domain (a domain switch left it detached)\n",
				    who);
		return NULL;
	}
	if (!domain->iova_cookie) {
		dev_err_ratelimited(dev, "%s: iommu domain has no iova cookie\n", who);
		return NULL;
	}
	return (struct rknpu_iommu_dma_cookie *)domain->iova_cookie;
}


dma_addr_t rknpu_iommu_dma_alloc_iova(struct iommu_domain *domain, size_t size,
				      u64 dma_limit, struct device *dev,
				      bool size_aligned)
{
	struct rknpu_iommu_dma_cookie *cookie;
	struct iova_domain *iovad;
	unsigned long shift, iova_len, iova = 0;
	unsigned long limit_pfn;
	struct iova *new_iova = NULL;
	bool alloc_fast = size_aligned;

	if (!domain || !domain->iova_cookie) {   /* #patch34 */
		dev_err_ratelimited(dev, "%s: no iommu domain/cookie\n", __func__);
		return 0;
	}
	cookie = (struct rknpu_iommu_dma_cookie *)domain->iova_cookie;
	iovad = &cookie->iovad;

	shift = iova_shift(iovad);
	iova_len = size >> shift;
	rknpu_dbg_iova_alloc_kb += (unsigned long)(size >> 10);   /* #patch69 */

#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
	/*
	 * Freeing non-power-of-two-sized allocations back into the IOVA caches
	 * will come back to bite us badly, so we have to waste a bit of space
	 * rounding up anything cacheable to make sure that can't happen. The
	 * order of the unadjusted size will still match upon freeing.
	 */
	if (iova_len < (1 << (IOVA_RANGE_CACHE_MAX_SIZE - 1)))
		iova_len = roundup_pow_of_two(iova_len);
#endif

#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
	dma_limit = min_not_zero(dma_limit, dev->bus_dma_limit);
#else
	if (dev->bus_dma_mask)
		dma_limit &= dev->bus_dma_mask;
#endif

	if (domain->geometry.force_aperture)
		dma_limit =
			min_t(u64, dma_limit, domain->geometry.aperture_end);

#if (KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE)
	limit_pfn = dma_limit >> shift;
#else
	limit_pfn = min_t(dma_addr_t, dma_limit >> shift, iovad->end_pfn);
#endif

	if (alloc_fast) {
		iova = alloc_iova_fast(iovad, iova_len, limit_pfn, true);
	} else {
		new_iova = alloc_iova(iovad, iova_len, limit_pfn, size_aligned);
		if (!new_iova)
			return 0;
		iova = new_iova->pfn_lo;
	}

	return (dma_addr_t)iova << shift;
}

void rknpu_iommu_dma_free_iova(struct rknpu_iommu_dma_cookie *cookie,
			       dma_addr_t iova, size_t size, bool size_aligned)
{
	rknpu_dbg_iova_free_kb += (unsigned long)(size >> 10);   /* #patch69 */
	struct iova_domain *iovad = &cookie->iovad;
	bool alloc_fast = size_aligned;

	if (alloc_fast)
		free_iova_fast(iovad, iova_pfn(iovad, iova),
			       size >> iova_shift(iovad));
	else
		free_iova(iovad, iova_pfn(iovad, iova));
}

static int rknpu_dma_info_to_prot(enum dma_data_direction dir, bool coherent)
{
	int prot = coherent ? IOMMU_CACHE : 0;

	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return prot | IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return prot | IOMMU_READ;
	case DMA_FROM_DEVICE:
		return prot | IOMMU_WRITE;
	default:
		return 0;
	}
}

/*
 * Prepare a successfully-mapped scatterlist to give back to the caller.
 *
 * At this point the segments are already laid out by iommu_dma_map_sg() to
 * avoid individually crossing any boundaries, so we merely need to check a
 * segment's start address to avoid concatenating across one.
 */
static int __finalise_sg(struct device *dev, struct scatterlist *sg, int nents,
			 dma_addr_t dma_addr)
{
	struct scatterlist *s, *cur = sg;
	unsigned long seg_mask = dma_get_seg_boundary(dev);
	unsigned int cur_len = 0, max_len = dma_get_max_seg_size(dev);
	int i, count = 0;

	for_each_sg(sg, s, nents, i) {
		/* Restore this segment's original unaligned fields first */
#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		dma_addr_t s_dma_addr = sg_dma_address(s);
#endif
		unsigned int s_iova_off = sg_dma_address(s);
		unsigned int s_length = sg_dma_len(s);
		unsigned int s_iova_len = s->length;

		sg_dma_address(s) = DMA_MAPPING_ERROR;
		sg_dma_len(s) = 0;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		if (sg_is_dma_bus_address(s)) {
			if (i > 0)
				cur = sg_next(cur);

			sg_dma_unmark_bus_address(s);
			sg_dma_address(cur) = s_dma_addr;
			sg_dma_len(cur) = s_length;
			sg_dma_mark_bus_address(cur);
			count++;
			cur_len = 0;
			continue;
		}
#endif

		s->offset += s_iova_off;
		s->length = s_length;

		/*
		 * Now fill in the real DMA data. If...
		 * - there is a valid output segment to append to
		 * - and this segment starts on an IOVA page boundary
		 * - but doesn't fall at a segment boundary
		 * - and wouldn't make the resulting output segment too long
		 */
		if (cur_len && !s_iova_off && (dma_addr & seg_mask) &&
		    (max_len - cur_len >= s_length)) {
			/* ...then concatenate it with the previous one */
			cur_len += s_length;
		} else {
			/* Otherwise start the next output segment */
			if (i > 0)
				cur = sg_next(cur);
			cur_len = s_length;
			count++;

			sg_dma_address(cur) = dma_addr + s_iova_off;
		}

		sg_dma_len(cur) = cur_len;
		dma_addr += s_iova_len;

		if (s_length + s_iova_off < s_iova_len)
			cur_len = 0;
	}
	return count;
}

/*
 * If mapping failed, then just restore the original list,
 * but making sure the DMA fields are invalidated.
 */
static void __invalidate_sg(struct scatterlist *sg, int nents)
{
	struct scatterlist *s;
	int i;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	for_each_sg(sg, s, nents, i) {
		if (sg_is_dma_bus_address(s)) {
			sg_dma_unmark_bus_address(s);
		} else {
			if (sg_dma_address(s) != DMA_MAPPING_ERROR)
				s->offset += sg_dma_address(s);
			if (sg_dma_len(s))
				s->length = sg_dma_len(s);
		}
		sg_dma_address(s) = DMA_MAPPING_ERROR;
		sg_dma_len(s) = 0;
	}
#else
	for_each_sg(sg, s, nents, i) {
		if (sg_dma_address(s) != DMA_MAPPING_ERROR)
			s->offset += sg_dma_address(s);
		if (sg_dma_len(s))
			s->length = sg_dma_len(s);
		sg_dma_address(s) = DMA_MAPPING_ERROR;
		sg_dma_len(s) = 0;
	}
#endif
}

int rknpu_iommu_dma_map_sg(struct device *dev, struct scatterlist *sg,
			   int nents, enum dma_data_direction dir,
			   bool iova_aligned)
{
	struct iommu_domain *domain = rknpu_iommu_live_domain(dev);
	struct rknpu_iommu_dma_cookie *cookie =
		rknpu_iommu_cookie(dev, __func__);   /* #patch33 */
	struct iova_domain *iovad;
	struct scatterlist *s = NULL, *prev = NULL;
	int prot = rknpu_dma_info_to_prot(dir, dev_is_dma_coherent(dev));
	dma_addr_t iova;
	unsigned long iova_len = 0;
	unsigned long mask = dma_get_seg_boundary(dev);
	ssize_t ret = -EINVAL;
	int i = 0;

	if (!cookie)
		return 0;   /* #patch33: 0 == mapping failed, per this function's convention */
	rknpu_dbg_map_n++;   /* #patch59 */
	iovad = &cookie->iovad;

	if (iova_aligned)
		return dma_map_sg(dev, sg, nents, dir);

	/*
	 * Work out how much IOVA space we need, and align the segments to
	 * IOVA granules for the IOMMU driver to handle. With some clever
	 * trickery we can modify the list in-place, but reversibly, by
	 * stashing the unaligned parts in the as-yet-unused DMA fields.
	 */
	for_each_sg(sg, s, nents, i) {
		size_t s_iova_off = iova_offset(iovad, s->offset);
		size_t s_length = s->length;
		size_t pad_len = (mask - iova_len + 1) & mask;

		sg_dma_address(s) = s_iova_off;
		sg_dma_len(s) = s_length;
		s->offset -= s_iova_off;
		s_length = iova_align(iovad, s_length + s_iova_off);
		s->length = s_length;

		/*
		 * Due to the alignment of our single IOVA allocation, we can
		 * depend on these assumptions about the segment boundary mask:
		 * - If mask size >= IOVA size, then the IOVA range cannot
		 *   possibly fall across a boundary, so we don't care.
		 * - If mask size < IOVA size, then the IOVA range must start
		 *   exactly on a boundary, therefore we can lay things out
		 *   based purely on segment lengths without needing to know
		 *   the actual addresses beforehand.
		 * - The mask must be a power of 2, so pad_len == 0 if
		 *   iova_len == 0, thus we cannot dereference prev the first
		 *   time through here (i.e. before it has a meaningful value).
		 */
		if (pad_len && pad_len < s_length - 1) {
			prev->length += pad_len;
			iova_len += pad_len;
		}

		iova_len += s_length;
		prev = s;
	}

	if (!iova_len) {
		ret = __finalise_sg(dev, sg, nents, 0);
		goto out;
	}

	iova = rknpu_iommu_dma_alloc_iova(domain, iova_len, dma_get_mask(dev),
					  dev, iova_aligned);
	if (!iova) {
		ret = -ENOMEM;
		LOG_ERROR("failed to allocate IOVA: %zd\n", ret);
		goto out_restore_sg;
	}

	ret = iommu_map_sg(domain, iova, sg, nents, prot);
	if (ret < 0 || ret < iova_len) {
		LOG_ERROR("failed to map SG: %zd\n", ret);
		goto out_free_iova;
	}

	return __finalise_sg(dev, sg, nents, iova);

out_free_iova:
	rknpu_iommu_dma_free_iova(cookie, iova, iova_len, iova_aligned);
out_restore_sg:
	__invalidate_sg(sg, nents);
out:

	if (ret < 0)
		ret = 0;

	return ret;
}

void rknpu_iommu_dma_unmap_sg(struct device *dev, struct scatterlist *sg,
			      int nents, enum dma_data_direction dir,
			      bool iova_aligned)
{
	struct iommu_domain *domain = rknpu_iommu_live_domain(dev);
	struct rknpu_iommu_dma_cookie *cookie =
		rknpu_iommu_cookie(dev, __func__);   /* #patch34 */
	struct iova_domain *iovad;
	size_t iova_off = 0;
	dma_addr_t end = 0, start = 0;
	struct scatterlist *tmp = NULL;
	dma_addr_t dma_addr = 0;
	size_t size = 0;
	int i = 0;

	if (!cookie)
		return;   /* #patch34: nothing mapped through a domain that is gone */
	rknpu_dbg_unmap_n++;   /* #patch59 */
	iovad = &cookie->iovad;

	if (iova_aligned)
		return dma_unmap_sg(dev, sg, nents, dir);

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	/*
	 * The scatterlist segments are mapped into a single
	 * contiguous IOVA allocation, the start and end points
	 * just have to be determined.
	 */
	for_each_sg(sg, tmp, nents, i) {
		if (sg_is_dma_bus_address(tmp)) {
			sg_dma_unmark_bus_address(tmp);
			continue;
		}

		if (sg_dma_len(tmp) == 0)
			break;

		start = sg_dma_address(tmp);
		break;
	}

	nents -= i;
	for_each_sg(tmp, tmp, nents, i) {
		if (sg_is_dma_bus_address(tmp)) {
			sg_dma_unmark_bus_address(tmp);
			continue;
		}

		if (sg_dma_len(tmp) == 0)
			break;

		end = sg_dma_address(tmp) + sg_dma_len(tmp);
	}
#else
	start = sg_dma_address(sg);
	for_each_sg(sg_next(sg), tmp, nents - 1, i) {
		if (sg_dma_len(tmp) == 0)
			break;
		sg = tmp;
	}
	end = sg_dma_address(sg) + sg_dma_len(sg);
#endif

	dma_addr = start;
	size = end - start;
	iova_off = iova_offset(iovad, start);

	if (end) {
		dma_addr -= iova_off;
		size = iova_align(iovad, size + iova_off);
		iommu_unmap(domain, dma_addr, size);
		rknpu_iommu_dma_free_iova(cookie, dma_addr, size, iova_aligned);
	}
}

#if defined(CONFIG_IOMMU_API) && defined(CONFIG_NO_GKI)

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
struct iommu_group {
	struct kobject kobj;
	struct kobject *devices_kobj;
	struct list_head devices;
#ifdef __ANDROID_COMMON_KERNEL__
	struct xarray pasid_array;
#endif
	struct mutex mutex;
	void *iommu_data;
	void (*iommu_data_release)(void *iommu_data);
	char *name;
	int id;
	struct iommu_domain *default_domain;
	struct iommu_domain *blocking_domain;
	struct iommu_domain *domain;
	struct list_head entry;
	unsigned int owner_cnt;
	void *owner;
};
#else
struct iommu_group {
	struct kobject kobj;
	struct kobject *devices_kobj;
	struct list_head devices;
	struct mutex mutex;
	struct blocking_notifier_head notifier;
	void *iommu_data;
	void (*iommu_data_release)(void *iommu_data);
	char *name;
	int id;
	struct iommu_domain *default_domain;
	struct iommu_domain *domain;
	struct list_head entry;
};
#endif

/* #patchB10: temporarily point the IOMMU core's DEFAULT domain at the LIVE domain.
 *
 * The DMA API resolves every mapping through iommu_get_dma_domain() == group->default_domain, and
 * dma_buf_map_attachment() -- which DRM core runs during PRIME_FD_TO_HANDLE, before any rknpu hook
 * -- goes through the DMA API. So an imported weight's sg is mapped into domain 0 while the object
 * is recorded as domain N: the NPU then resolves an IOVA that exists only in domain 0's page table,
 * and the job is committed but never completes.
 *
 * This is deliberately NOT the vendor's overwrite. That one was PERMANENT -- every switch left the
 * core's default pointing at a domain the driver might later free, so any core path that reattaches
 * "the default" (reset, release, error recovery) used it, which is where the mismatch-domain class
 * came from. Here the override lives only for the duration of one attachment map/unmap, under
 * domain_lock so no switch can move the live domain underneath it, and is restored immediately.
 *
 * Returns the previous default so the caller can restore it. */
struct iommu_domain *rknpu_iommu_default_swap(struct device *dev, struct iommu_domain *dom)
{
	struct rknpu_device *rknpu_dev = dev_get_drvdata(dev);
	struct iommu_domain *old;

	if (!rknpu_dev || !rknpu_dev->iommu_group || !dom)
		return NULL;
	old = rknpu_dev->iommu_group->default_domain;
	rknpu_dev->iommu_group->default_domain = dom;
	return old;
}

int rknpu_iommu_init_domain(struct rknpu_device *rknpu_dev)
{
	// init domain 0
	if (!rknpu_dev->iommu_domains[0]) {
		rknpu_dev->iommu_domain_id = 0;
		rknpu_dev->iommu_domains[rknpu_dev->iommu_domain_id] =
			iommu_get_domain_for_dev(rknpu_dev->dev);
		rknpu_dev->iommu_domain_num = 1;
	}
	return 0;
}

int rknpu_iommu_switch_domain(struct rknpu_device *rknpu_dev, int domain_id)
{
	struct iommu_domain *src_domain = NULL;
	struct iommu_domain *dst_domain = NULL;
	struct bus_type *bus = NULL;
	int src_domain_id = 0;
	int ret = -EINVAL;

	if (!rknpu_dev->iommu_en)
		return -EINVAL;

	if (domain_id < 0 || domain_id > (RKNPU_MAX_IOMMU_DOMAIN_NUM - 1)) {
		LOG_DEV_ERROR(
			rknpu_dev->dev,
			"invalid iommu domain id: %d, reuse domain id: %d\n",
			domain_id, rknpu_dev->iommu_domain_id);
		return -EINVAL;
	}

	bus = rknpu_dev->dev->bus;
	if (!bus)
		return -EFAULT;

	src_domain_id = rknpu_dev->iommu_domain_id;
	if (domain_id == src_domain_id) {
		return 0;
	}

	/* #patch30 (diagnostic): a domain switch calls iommu_attach_device(), and for
	 * rockchip-iommu that runs rk_iommu_enable() -> enable_stall + FORCE_RESET of every
	 * MMU bank + ZAP_CACHE + paging off/on. If an NPU job is still in flight, its
	 * descriptor/regcmd fetches are cut mid-dispatch: the job is committed but the PC
	 * completed-task counter never advances, and nothing faults. Count switches that
	 * race in-flight work so it can be correlated with the observed dispatch stalls. */
	{
		int _i, _busy = 0;
		unsigned long _f;
		spin_lock_irqsave(&rknpu_dev->irq_lock, _f);
		for (_i = 0; _i < rknpu_dev->config->num_irqs; _i++)
			if (rknpu_dev->subcore_datas[_i].job)
				_busy++;
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, _f);
		if (_busy) {
			rknpu_dbg_switch_inflight++;
			if (rknpu_dbg_switch_verbose)
				LOG_WARN("switch domain %d->%d with %d job(s) IN FLIGHT (n=%lu)\n",
					 src_domain_id, domain_id, _busy,
					 rknpu_dbg_switch_inflight);
		} else {
			rknpu_dbg_switch_idle++;
		}
	}

	/* #patchB2: the "mismatch domain" check is gone with the core detach/attach it guarded.
	 * iommu_get_domain_for_dev() reports the group's default domain, which no longer tracks our
	 * switches -- so the check could only ever fire spuriously. The driver's own
	 * iommu_domains[src_domain_id] IS the source of truth now. */
	src_domain = rknpu_dev->iommu_domains[src_domain_id];

	dst_domain = rknpu_dev->iommu_domains[domain_id];
	if (dst_domain != NULL) {
		ret = rk_iommu_switch_domain(rknpu_dev->dev, dst_domain);   /* #patchB2 */
		if (ret) {
			LOG_DEV_ERROR(
				rknpu_dev->dev,
				"failed to attach dst iommu domain, id: %d, ret: %d\n",
				domain_id, ret);
			/* #patchB3: recover with the light switch too.
			 *
			 * Note the failure mode is much weaker than it used to be: the old code had
			 * ALREADY executed iommu_detach_device(src) before the failing attach, so a
			 * failure left the device attached to NO domain and panicked the box on the
			 * next map (#patch34). rk_iommu_switch_domain() is atomic from the caller's
			 * point of view -- it either reprogrammed DTE_ADDR or it did not -- so on
			 * failure the previous page table is still live and this is belt-and-braces. */
			if (rk_iommu_switch_domain(rknpu_dev->dev, src_domain)) {
				int _d;

				LOG_DEV_ERROR(
					rknpu_dev->dev,
					"failed to restore src iommu domain, id: %d\n",
					src_domain_id);
				for (_d = 0; _d < RKNPU_MAX_IOMMU_DOMAIN_NUM; _d++) {
					if (!rknpu_dev->iommu_domains[_d])
						continue;
					if (!rk_iommu_switch_domain(
						    rknpu_dev->dev,
						    rknpu_dev->iommu_domains[_d])) {
						rknpu_dev->iommu_domain_id = _d;
						LOG_DEV_ERROR(rknpu_dev->dev,
							      "recovered onto iommu domain %d\n",
							      _d);
						break;
					}
				}
			}
			return ret;
		}
		rknpu_dev->iommu_domain_id = domain_id;
	} else {
		uint64_t dma_limit = 1ULL << 32;

		dst_domain = iommu_domain_alloc(bus);
		if (!dst_domain) {
			LOG_DEV_ERROR(rknpu_dev->dev,
				      "failed to allocate iommu domain\n");
			return -EIO;
		}
		// init domain iova_cookie
		iommu_get_dma_cookie(dst_domain);

		/* #patchB2: initialise OUR OWN IOVA allocator for this domain.
		 *
		 * The vendor code got an iovad by faking a DMA-API domain: it set
		 * __IOMMU_DOMAIN_DMA_API on the type and called iommu_setup_dma_ops(), whose only
		 * useful effect here was iommu_dma_init_domain() initialising the cookie's iovad.
		 * That route needs iommu_get_domain_for_dev() to report this domain, i.e. it needs
		 * the default_domain overwrite. Do the two init calls directly instead -- same
		 * iovad, no lie, and this domain stays a plain UNMANAGED domain that iommu_map_sg()
		 * and iommu_unmap() are happy to serve.
		 *
		 * Granule/start_pfn mirror iommu_dma_init_domain(): 4 KiB pages, base_pfn 1 so that
		 * IOVA 0 is never handed out. */
		{
			struct rknpu_iommu_dma_cookie *ck =
				(struct rknpu_iommu_dma_cookie *)dst_domain->iova_cookie;

			if (!ck) {
				LOG_DEV_ERROR(rknpu_dev->dev,
					      "no iova cookie for domain %d\n", domain_id);
				iommu_domain_free(dst_domain);
				return -ENOMEM;
			}
			init_iova_domain(&ck->iovad, SZ_4K, 1);
			if (iova_domain_init_rcaches(&ck->iovad)) {
				LOG_DEV_ERROR(rknpu_dev->dev,
					      "failed to init iova rcaches, domain %d\n",
					      domain_id);
				iommu_domain_free(dst_domain);
				return -ENOMEM;
			}
		}

		ret = rk_iommu_switch_domain(rknpu_dev->dev, dst_domain);   /* #patchB2 */
		if (ret) {
			LOG_DEV_ERROR(
				rknpu_dev->dev,
				"failed to switch to iommu domain, id: %d, ret: %d\n",
				domain_id, ret);
			iommu_domain_free(dst_domain);
			return ret;
		}
		(void)dma_limit;

		rknpu_dev->iommu_domain_id = domain_id;
		rknpu_dev->iommu_domains[domain_id] = dst_domain;
		rknpu_dev->iommu_domain_num++;
	}

	/* #patchB2: the default_domain overwrite is DELETED.
	 *
	 * It existed because dma-iommu resolves every DMA-API mapping through
	 * iommu_get_dma_domain() == dev->iommu_group->default_domain, so the only way to make
	 * dma_map_sg() land in domain N was to tell the core that N was the default. That lie is
	 * the origin of the "mismatch domain" errors and of attaching a domain the driver may
	 * later free. Nothing here uses the DMA API for NPU buffers any more: allocations are
	 * NON_CONTIGUOUS + IOMMU_LIMIT_IOVA_ALIGNMENT, which take the explicit
	 * iommu_map_sg()/iommu_unmap() path against the domain the driver names. */

	/* #patch53: COOLDOWN after a domain switch.
	 *
	 * Measured, replicated across two unrelated code paths (blocking colsplit on a 1.5B model,
	 * and the async int4 doorbell probe): EVERY observed stall was a commit landing within
	 * ~1 ms of an IOMMU domain switch, and there were ZERO stalls beyond that window across
	 * ~10,000 commits. Being early by COUNT is not the predictor -- 9004 commits at position
	 * 0-2 produced 5 stalls, all of them in the sub-1 ms subset while 8993 early-but->=1ms
	 * commits were clean. Time is the operative variable.
	 *
	 * Sleeping here is legal and correct: rknpu_iommu_switch_domain() is only ever reached from
	 * process context (gem create/destroy/sync ioctls and the submit path), never from the
	 * completion IRQ. That is why this can usleep_range() where the earlier per-commit spacing
	 * experiment was forced into udelay() -- and it is charged once per SWITCH, not once per
	 * commit, so the cost is ~1 ms x the switch count (~56 in a 1.5B run) rather than per job.
	 *
	 * NOT a claim about mechanism: the window is necessary but not sufficient (inside it the
	 * rate ranges 0.2%-90.9% by workload), so something else modulates it. This removes the
	 * necessary condition, which is enough to eliminate every stall observed so far.
	 *
	 * 0 = off (default, opt-in until measured). */
	if (rknpu_dom_cooldown_us) {
		usleep_range(rknpu_dom_cooldown_us, rknpu_dom_cooldown_us + 100);
		rknpu_dom_cooldowns++;
	}

	rknpu_commits_since_switch = 0;   /* #patch51 */
	rknpu_last_switch_time = ktime_get();   /* #patch52 -- stamp AFTER the cooldown */
	LOG_INFO("switch iommu domain from %d to %d\n", src_domain_id,
		 domain_id);

	return ret;
}

/* #patchB5: RECLAIM-AND-RETRY on a domain-switch timeout (default ON).
 *
 * A stalled job (dispatch stall: committed, PC task counter stuck at 0, no interrupt) holds its
 * iommu domain reference forever. rknpu_iommu_domain_get_and_switch() only proceeds when the
 * refcount reads 0, so from the first stall onward EVERY switch burns its full 6 s and fails --
 * and because rknpu_gem_object_create() switches domains, every later allocation fails -EINVAL
 * ("MEM_CREATE errno=22" at zero IOVA used). One stall therefore kills the whole run, and for a
 * >4 GiB model spread over several domains that is fatal rather than merely slow.
 *
 * patch54 already reaped on timeout, but it reaped and then RETURNED -EINVAL anyway, so the
 * caller that triggered the reap still failed; only some later call could benefit. It also
 * required `switch_escalate` CONSECUTIVE timeouts (3 x 6 s = 18 s), and any success in between
 * reset the counter. Measured: the reproducer still died with escalate=3.
 *
 * So: reap on the FIRST timeout, then RETRY the switch. If the reap cleared every in-flight job
 * but the refcount is still non-zero, that count is by definition leaked -- no job owns it -- so
 * force it to 0 rather than stay wedged forever.
 *
 * Note the reap only runs after the full 6 s wait has already elapsed, so anything still in
 * flight has been running for >= 6 s. A real NPU job here is microseconds to milliseconds, so
 * that set is exactly the stuck jobs. A submit landing in the last instant before the reap could
 * be caught with them; that is a far better outcome than a permanently dead domain. */
unsigned int rknpu_dom_reclaim = 1;
module_param_named(dom_reclaim, rknpu_dom_reclaim, uint, 0644);
MODULE_PARM_DESC(dom_reclaim, "on a domain-switch timeout, reap stuck jobs and retry (0=off)");
unsigned long rknpu_dbg_reclaim_tries, rknpu_dbg_reclaim_ok, rknpu_dbg_reclaim_forced,
	rknpu_dbg_reclaim_fail;
module_param_named(dbg_reclaim_tries, rknpu_dbg_reclaim_tries, ulong, 0444);
module_param_named(dbg_reclaim_ok, rknpu_dbg_reclaim_ok, ulong, 0444);
module_param_named(dbg_reclaim_forced, rknpu_dbg_reclaim_forced, ulong, 0444);
module_param_named(dbg_reclaim_fail, rknpu_dbg_reclaim_fail, ulong, 0444);

int rknpu_iommu_domain_get_and_switch(struct rknpu_device *rknpu_dev,
				      int domain_id)
{
	unsigned long timeout_jiffies =
		msecs_to_jiffies(RKNPU_SWITCH_DOMAIN_WAIT_TIME_MS);
	unsigned long start = jiffies;
	int ret = -EINVAL;
	int reclaimed = 0;   /* #patchB5 */

	while (true) {
		mutex_lock(&rknpu_dev->domain_lock);

		if (domain_id == rknpu_dev->iommu_domain_id) {
			rknpu_switch_consec = 0;   /* #patch54 */
			atomic_inc(&rknpu_dev->iommu_domain_refcount);
			rknpu_dbg_refcnt_now = atomic_read(&rknpu_dev->iommu_domain_refcount);   /* #patch59 */
			mutex_unlock(&rknpu_dev->domain_lock);
			break;
		}

		if (atomic_read(&rknpu_dev->iommu_domain_refcount) == 0) {
			ret = rknpu_iommu_switch_domain(rknpu_dev, domain_id);
			if (ret) {
				LOG_DEV_ERROR(
					rknpu_dev->dev,
					"failed to switch iommu domain, id: %d, ret: %d\n",
					domain_id, ret);
				mutex_unlock(&rknpu_dev->domain_lock);
				return ret;
			}
			rknpu_switch_consec = 0;   /* #patch54 */
			atomic_inc(&rknpu_dev->iommu_domain_refcount);
			rknpu_dbg_refcnt_now = atomic_read(&rknpu_dev->iommu_domain_refcount);   /* #patch59 */
			mutex_unlock(&rknpu_dev->domain_lock);
			break;
		}

		mutex_unlock(&rknpu_dev->domain_lock);

		usleep_range(10, 100);
		if (time_after(jiffies, start + timeout_jiffies)) {
			LOG_DEV_ERROR(
				rknpu_dev->dev,
				"switch iommu domain time out, failed to switch iommu domain, id: %d\n",
				domain_id);
			/* #patchB5: reap the stuck job(s), then RETRY -- do not fail the caller
			 * that is trying to make progress. Once only: if the switch still times
			 * out after a successful reclaim, something is wrong beyond a stuck job. */
			if (rknpu_dom_reclaim && !reclaimed) {
				unsigned long f;
				int k, busy = 0;

				reclaimed = 1;
				rknpu_dbg_reclaim_tries++;
				rknpu_reap_all_cores(rknpu_dev);

				spin_lock_irqsave(&rknpu_dev->irq_lock, f);
				for (k = 0; k < rknpu_dev->config->num_irqs; k++)
					if (rknpu_dev->subcore_datas[k].job)
						busy++;
				spin_unlock_irqrestore(&rknpu_dev->irq_lock, f);

				if (!busy) {
					int rc = atomic_read(&rknpu_dev->iommu_domain_refcount);

					if (rc != 0) {
						/* nothing is in flight, so this count is leaked */
						LOG_DEV_ERROR(rknpu_dev->dev,
							      "reclaim: no jobs in flight but refcount=%d -- forcing to 0\n",
							      rc);
						atomic_set(&rknpu_dev->iommu_domain_refcount, 0);
						rknpu_dbg_refcnt_now = 0;
						rknpu_dbg_reclaim_forced++;
					}
					LOG_DEV_ERROR(rknpu_dev->dev,
						      "reclaim: domain reference released, retrying switch to id %d\n",
						      domain_id);
					rknpu_dbg_reclaim_ok++;
					start = jiffies;
					continue;
				}

				rknpu_dbg_reclaim_fail++;
				LOG_DEV_ERROR(rknpu_dev->dev,
					      "reclaim: %d job(s) survived the reap -- cannot release the domain\n",
					      busy);
			}

			/* #patch54: break the livelock rather than returning into an infinite retry */
			if (rknpu_switch_escalate &&
			    ++rknpu_switch_consec >= rknpu_switch_escalate) {
				rknpu_switch_consec = 0;
				rknpu_switch_escalations++;
				LOG_DEV_ERROR(rknpu_dev->dev,
					      "ESCALATING after %u consecutive switch timeouts: reaping stuck jobs to release the domain reference\n",
					      rknpu_switch_escalate);
				rknpu_reap_all_cores(rknpu_dev);
			}
			return -EINVAL;
		}
	}

	return 0;
}

/* #patch45 diagnostics: the live refcount, and how often a put tried to take it negative. */
unsigned long rknpu_dbg_premature_zero;
module_param_named(dbg_premature_zero, rknpu_dbg_premature_zero, ulong, 0444);
unsigned long rknpu_dbg_domain_underflow;
module_param_named(dbg_domain_underflow, rknpu_dbg_domain_underflow, ulong, 0444);

int rknpu_iommu_domain_put(struct rknpu_device *rknpu_dev)
{
	/* #patch45: NEVER let the refcount go negative.
	 *
	 * rknpu_iommu_domain_get_and_switch() proceeds only when this reads exactly 0
	 * (`if (atomic_read(&iommu_domain_refcount) == 0)`). A bare atomic_dec() therefore turns a
	 * single unbalanced put into a PERMANENT wedge: the count never reads 0 again, every domain
	 * switch burns its 6 s RKNPU_SWITCH_DOMAIN_WAIT_TIME_MS and fails, and because
	 * rknpu_gem_object_create() switches domains, EVERY allocation then fails with -EINVAL
	 * ("rknpu_gem_object_create error") until reboot.
	 *
	 * Unbalanced puts are a live risk here, not a hypothetical: our own kernel change 02 ADDED a
	 * put to rknpu_job_timeout_clean() to reclaim a leaked reference, which double-puts on any
	 * path that already released it. That is precisely the wedge seen repeatedly on this board
	 * (59 'mismatch domain'/'switch iommu domain time out' lines in a single 300 s run), and it
	 * invalidated several measurements before it was understood.
	 *
	 * Clamping is strictly better than wedging: an over-put becomes a counted anomaly instead of
	 * a dead NPU. The clamp is racy against a concurrent get, but the alternative it replaces is
	 * an unrecoverable device, so the trade is not close. dbg_domain_underflow != 0 means a real
	 * refcount bug remains to be found -- the clamp treats the symptom, not the cause. */
	/* #patch60: NAME the over-putting caller.
	 *
	 * Six runs showed dbg_domain_underflow reaching 5 — excess puts — but the counter alone
	 * cannot say WHICH of the ~10 call sites is unbalanced, and change 02 (our added put in
	 * rknpu_job_timeout_clean) is only a suspect, not a finding. %pS resolves the return address
	 * to a symbol, so the offending site names itself.
	 *
	 * Two conditions are reported, and the SECOND is the one that actually causes damage:
	 *   - underflow: the count went below zero. Loud, but already too late.
	 *   - premature zero: the count hit exactly 0 while jobs are still IN FLIGHT. No underflow,
	 *     no warning, but rknpu_iommu_domain_get_and_switch() tests only `refcount == 0`, so it
	 *     will now switch domains out from under live work — which is the mismatch-domain
	 *     signature. These are invisible to the underflow counter and must be more numerous. */
	{
		void *caller = __builtin_return_address(0);
		int after;

		rknpu_dbg_refcnt_now = atomic_read(&rknpu_dev->iommu_domain_refcount) - 1;
		after = atomic_dec_return(&rknpu_dev->iommu_domain_refcount);
		if (after < 0) {
			atomic_set(&rknpu_dev->iommu_domain_refcount, 0);
			rknpu_dbg_refcnt_now = 0;
			rknpu_dbg_domain_underflow++;
			LOG_DEV_ERROR(rknpu_dev->dev,
				      "domain_put UNDERFLOW (clamped) from %pS\n", caller);
		} else if (after == 0) {
			unsigned long f;
			int k, busy = 0;

			spin_lock_irqsave(&rknpu_dev->irq_lock, f);
			for (k = 0; k < rknpu_dev->config->num_irqs; k++)
				if (rknpu_dev->subcore_datas[k].job)
					busy++;
			spin_unlock_irqrestore(&rknpu_dev->irq_lock, f);
			if (busy) {
				rknpu_dbg_premature_zero++;
				LOG_DEV_ERROR(rknpu_dev->dev,
					      "domain refcount hit 0 with %d job(s) IN FLIGHT — put from %pS\n",
					      busy, caller);
			}
		}
	}

	return 0;
}

/* #patch61: explicit state reset — the software equivalent of the reboot.
 *
 * Refuses while any core has a job, because freeing a domain out from under live work would strand
 * its translations. Beyond freeing the domains it also zeroes the refcount and the diagnostic
 * counters, so a fresh run starts from a known state rather than inheriting a skewed one. Note the
 * caller must treat every existing buffer AND every regcmd it has built as invalid afterwards: the
 * IOVAs change, and userspace bakes IOVAs into its programs. */
int rknpu_iommu_reinit(struct rknpu_device *rknpu_dev)
{
	unsigned long flags;
	int i, busy = 0;

	spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
	for (i = 0; i < rknpu_dev->config->num_irqs; i++)
		if (rknpu_dev->subcore_datas[i].job)
			busy++;
	spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
	/* #patch62: ABORT in-flight work rather than refusing.
	 *
	 * The first cut returned -EBUSY whenever any core had a job. That made the call useless for
	 * its entire purpose: a device in the bad state almost always HAS a stuck job — that is what
	 * being stuck means — so REINIT refused with -EBUSY at exactly the moment it was needed
	 * (measured: idle device -> ok; wedged device -> -EBUSY). The guard was protecting the caller
	 * from the case the caller was trying to fix.
	 *
	 * So reap first, then re-check. rknpu_reap_all_cores() runs the driver's own timeout_clean
	 * path, which aborts the jobs and releases the references they hold. If a core STILL reports a
	 * job after that, something is wedged beyond what this call can safely unpick, and -EBUSY is
	 * then a truthful answer rather than an unhelpful one. */
	if (busy) {
		LOG_INFO("RKNPU: REINIT — %d job(s) in flight, reaping before teardown\n", busy);
		rknpu_reap_all_cores(rknpu_dev);

		busy = 0;
		spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
		for (i = 0; i < rknpu_dev->config->num_irqs; i++)
			if (rknpu_dev->subcore_datas[i].job)
				busy++;
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

		if (busy) {
			LOG_DEV_ERROR(rknpu_dev->dev,
				      "REINIT: %d job(s) survived the reap — refusing\n",
				      busy);
			return -EBUSY;
		}
	}

	LOG_INFO("RKNPU: REINIT — freeing all iommu domains and resetting state (was: refcnt=%d underflow=%lu premature_zero=%lu)\n",
		 atomic_read(&rknpu_dev->iommu_domain_refcount),
		 rknpu_dbg_domain_underflow, rknpu_dbg_premature_zero);

	rknpu_iommu_free_domains(rknpu_dev);
	atomic_set(&rknpu_dev->iommu_domain_refcount, 0);
	rknpu_dbg_refcnt_now = 0;
	rknpu_dbg_domain_underflow = 0;
	rknpu_dbg_premature_zero = 0;
	rknpu_dbg_map_n = 0;
	rknpu_dbg_unmap_n = 0;
	rknpu_switch_consec = 0;

	return 0;
}

void rknpu_iommu_free_domains(struct rknpu_device *rknpu_dev)
{
	int i = 0;

	if (rknpu_iommu_domain_get_and_switch(rknpu_dev, 0)) {
		LOG_DEV_ERROR(rknpu_dev->dev, "%s error\n", __func__);
		return;
	}

	for (i = 1; i < RKNPU_MAX_IOMMU_DOMAIN_NUM; i++) {
		struct iommu_domain *domain = rknpu_dev->iommu_domains[i];

		if (domain == NULL)
			continue;

		/* #patchB3: no iommu_detach_device() -- the core never attached these domains.
		 * The switch to domain 0 above already moved the hardware off this page table, so
		 * freeing it here is safe. Release the IOVA allocator we initialised in the switch
		 * (init_iova_domain + iova_domain_init_rcaches) before the domain goes. */
		if (domain->iova_cookie) {
			struct rknpu_iommu_dma_cookie *ck =
				(struct rknpu_iommu_dma_cookie *)domain->iova_cookie;

			put_iova_domain(&ck->iovad);
		}
		iommu_domain_free(domain);

		rknpu_dev->iommu_domains[i] = NULL;
	}

	rknpu_iommu_domain_put(rknpu_dev);
}

#else

int rknpu_iommu_init_domain(struct rknpu_device *rknpu_dev)
{
	return 0;
}

int rknpu_iommu_switch_domain(struct rknpu_device *rknpu_dev, int domain_id)
{
	return 0;
}

int rknpu_iommu_domain_get_and_switch(struct rknpu_device *rknpu_dev,
				      int domain_id)
{
	return 0;
}

int rknpu_iommu_domain_put(struct rknpu_device *rknpu_dev)
{
	return 0;
}

void rknpu_iommu_free_domains(struct rknpu_device *rknpu_dev)
{
}

#endif
