/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Felix Zeng <felix.zeng@rock-chips.com>
 */

#ifndef __LINUX_RKNPU_JOB_H_
#define __LINUX_RKNPU_JOB_H_

#include <linux/spinlock.h>
#include <linux/dma-fence.h>
#include <linux/irq.h>

#include <drm/drm_device.h>

#include "rknpu_ioctl.h"

#define RKNPU_MAX_CORES 3

#define RKNPU_JOB_DONE (1 << 0)
#define RKNPU_JOB_ASYNC (1 << 1)
/* #patch41: the progress watchdog has declared this job stalled. INTERNAL to job->flags (which is
 * never assigned from the uABI args->flags), so it cannot collide with RKNPU_JOB_PC/NONBLOCK/etc. */
#define RKNPU_JOB_STALLED (1 << 8)

/* #patch47 profiler fields (filled at commit; free -- derived from existing timestamps) */
#define RKNPU_JOB_DETACHED (1 << 2)

#define RKNPU_CORE_AUTO_MASK 0x00
#define RKNPU_CORE0_MASK 0x01
#define RKNPU_CORE1_MASK 0x02
#define RKNPU_CORE2_MASK 0x04

struct rknpu_job {
	struct rknpu_device *rknpu_dev;
	struct list_head head[RKNPU_MAX_CORES];
	struct work_struct cleanup_work;
	bool irq_entry[RKNPU_MAX_CORES];
	unsigned int flags;
	/* #patch64: does THIS job currently hold the iommu domain reference?
	 *
	 * The reference is acquired exactly once (rknpu_job_commit) but THREE teardown paths release
	 * it -- rknpu_job_done(), rknpu_job_abort() and rknpu_job_timeout_clean() -- with nothing
	 * recording whether this job still holds one. done-then-aborted, or aborted-then-reaped,
	 * releases twice, driving the device-wide count to zero while OTHER cores are still executing;
	 * rknpu_iommu_domain_get_and_switch() then reads zero as "safe to switch" and reprograms the
	 * IOMMU underneath live work. Caller-tagging measured 17 such premature zeros in one run,
	 * attributed to rknpu_job_abort+0x40. Upstream: rockchip-linux/kernel#387.
	 *
	 * Its own word (not a bit in ->flags) because the releases run from both IRQ and process
	 * context, so the test-and-clear must be atomic. */
	unsigned long dom_held;
	/* #patch73: set while this job holds a power reference of its own. An async
	 * (RKNPU_JOB_NONBLOCK) job outlives the ioctl that created it, so it cannot rely on the
	 * ioctl reference to keep the block powered -- see rknpu_job_submit(). */
	unsigned long pwr_held;
	/* #patch73: this ASYNC job holds a POWER reference. RKNPU_IOCTL scopes power_get/put_delay to the
	 * IOCTL, which covers a blocking submit (it waits for completion inside) but NOT a NONBLOCK one: that
	 * returns as soon as the job is scheduled, the put_delay countdown starts while the job is still on the
	 * hardware, and the completion IRQ then arrives with the block powered down. */
	int ret;
	struct rknpu_submit *args;
	bool args_owner;
	struct rknpu_task *first_task;
	struct rknpu_task *last_task;
	uint32_t int_mask[RKNPU_MAX_CORES];
	uint32_t int_status[RKNPU_MAX_CORES];
	struct dma_fence *fence;
	ktime_t timestamp;
	uint32_t use_core_num;
	atomic_t run_count;
	atomic_t interrupt_count;
	ktime_t hw_commit_time;
	ktime_t hw_recoder_time;
	ktime_t hw_elapse_time;
	atomic_t submit_count[RKNPU_MAX_CORES];
	int iommu_domain_id;
	s64 prof_gap_us;    /* #patch47: since previous commit on this core */
	s64 prof_queue_us;
	unsigned int prof_since_switch;
	s64 prof_tsince_us;   /* #patch52: us since the last domain switch */   /* #patch51 */  /* #patch47: submit -> commit */
	/* #patch (experiment): PC register snapshot taken at commit — what we WROTE and what
	 * reads BACK immediately after. Reported by the progress watchdog when a job stalls, so
	 * a stalled commit can be compared against a healthy one with no per-commit logging. */
	uint32_t dbg_blk[17];   /* #patch: full PC/INT register block sampled after the start pulse */
	ktime_t dbg_kick_t;   /* #patch: when recovery kicked this job (0 = never) */
	uint32_t dbg_seq;   /* #patch: userspace task-descriptor stamp seen at commit */
	uint32_t dbg_wr[6];
	uint32_t dbg_rd[7];
	bool     dbg_valid;
};

irqreturn_t rknpu_core0_irq_handler(int irq, void *data);
irqreturn_t rknpu_core1_irq_handler(int irq, void *data);
irqreturn_t rknpu_core2_irq_handler(int irq, void *data);

#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
int rknpu_submit_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *file_priv);
#endif
#ifdef CONFIG_ROCKCHIP_RKNPU_DMA_HEAP
int rknpu_submit_ioctl(struct rknpu_device *rknpu_dev, unsigned long data);
#endif

int rknpu_get_hw_version(struct rknpu_device *rknpu_dev, uint32_t *version);

int rknpu_get_bw_priority(struct rknpu_device *rknpu_dev, uint32_t *priority,
			  uint32_t *expect, uint32_t *tw);

int rknpu_set_bw_priority(struct rknpu_device *rknpu_dev, uint32_t priority,
			  uint32_t expect, uint32_t tw);

int rknpu_clear_rw_amount(struct rknpu_device *rknpu_dev);

int rknpu_get_rw_amount(struct rknpu_device *rknpu_dev, uint32_t *dt_wr,
			uint32_t *dt_rd, uint32_t *wd_rd);

int rknpu_get_total_rw_amount(struct rknpu_device *rknpu_dev, uint32_t *amount);

void rknpu_job_redrive(struct rknpu_device *rknpu_dev);   /* #patch72 */

#endif /* __LINUX_RKNPU_JOB_H_ */

void rknpu_reap_all_cores(struct rknpu_device *rknpu_dev);   /* #patch54 */
