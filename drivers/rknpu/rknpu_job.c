/* #patch40: the wd_kick auto-recovery was deleted -- see the note in rknpu_drv.c. */
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Felix Zeng <felix.zeng@rock-chips.com>
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sync_file.h>
#include <linux/io.h>

#include "rknpu_ioctl.h"
#include "rknpu_drv.h"
#include "rknpu_reset.h"
#include "rknpu_gem.h"

/* #patch71 counters: defined in rknpu_drv.c, exported via /sys/module/rknpu/parameters/ */
extern unsigned long rknpu_dbg_next_blocked, rknpu_dbg_next_blocked_logged, rknpu_dbg_blocked_slow;
extern long rknpu_dbg_blocked_age_us;
#include "rknpu_fence.h"
#include "rknpu_mem.h"
#include <linux/iommu.h>
#include "rknpu_iommu.h"
#include "rknpu_job.h"

/* #patch66: UNMASK the bits the hardware sets when a job fails to dispatch.
 *
 * Comparing the PC block for a healthy job against a stalled one, the one register that differs is
 * INT_RAW_STATUS (0x2c): 0x00000008 healthy, 0xc0000000 stalled — bits 30 and 31 latched. INT_MASK is
 * 0x300 and does not cover them, so the NPU detects the condition, records it, and delivers no
 * interrupt. Everything else in the block is identical and correct at the stall (PC_DATA_ADDR holds
 * the regcmd IOVA, the 0x6 trigger has been consumed, INT_STATUS is 0).
 *
 * Setting this ORs extra bits into INT_MASK so the condition can actually be delivered. Two hazards
 * handled here:
 *   - RKNPU_INT_CLEAR is 0x1ffff, i.e. bits 0..16 only, so the new bits would NOT be clearable and a
 *     level-triggered source would storm forever. The same extra bits are therefore OR'd into every
 *     INT_CLEAR write as well.
 *   - if it storms anyway, the guard below disarms itself rather than hanging the board.
 *
 * 0 = off (default). Try 0xc0000000. */
unsigned int rknpu_int_mask_extra;
module_param_named(int_mask_extra, rknpu_int_mask_extra, uint, 0644);
MODULE_PARM_DESC(int_mask_extra, "OR these bits into INT_MASK/INT_CLEAR (0=off; try 0xc0000000)");
unsigned long rknpu_int_extra_fired;
module_param_named(int_extra_fired, rknpu_int_extra_fired, ulong, 0444);

/* #patch47 */
ktime_t rknpu_prof_last_commit[3];
/* #patch48 */
extern unsigned long rknpu_prof_ok_n, rknpu_prof_ok_gap, rknpu_prof_ok_queue;
/* #patch49 */
extern unsigned long rknpu_gap_enforced_n, rknpu_gap_enforced_us;
unsigned int rknpu_min_commit_gap_get(void);
unsigned int rknpu_prof_mask_get(void);   /* #patch50 */
extern unsigned int rknpu_commits_since_switch;   /* #patch51 */
extern ktime_t rknpu_last_switch_time;   /* #patch52 */



/* #patch (experiment): does the QUEUE stall behind a lost completion interrupt?
 * rknpu_job_next() only runs from the completion path, so a lost IRQ leaves subcore_data->job
 * set and the next submit sits on todo_list, never committed -> PC never starts -> the
 * "accepted but never dispatched" doorbell miss. A deficit of completions vs commits proves it.
 *   commit  = rknpu_job_subcore_commit() calls  (jobs actually handed to the PC)
 *   irq     = rknpu_irq_handler() entries       (interrupts that reached the handler)
 *   nojob   = handler found no job              (spurious / already-reaped)
 *   unpow   = handler declined, block unpowered (Change 6 guard)
 *   done    = rknpu_job_done() final completion (interrupt_count reached 0)
 * Healthy steady state: commit == done, and irq >= commit. */
static unsigned int rknpu_dbg_healthy_n;
/* #patchB7: catch a BLOCKING submit that reports success for a job that never completed.
 * Measured contradiction: the first int8 op after an ACT_RESET has every one of its submits
 * committed with NONE completing (cnt_done/cnt_irq do not move), yet the ioctl returns 0 and
 * userspace consumes an unwritten output buffer. Only one site sets RKNPU_JOB_DONE and it
 * increments cnt_done in the same block, so a success return without JOB_DONE is a real leak.
 * Record it and name the flags -- in particular whether RKNPU_JOB_PC was set, since the blocking
 * path only calls rknpu_job_wait() when it is. */
unsigned long rknpu_dbg_false_ok;
module_param_named(dbg_false_ok, rknpu_dbg_false_ok, ulong, 0444);
unsigned long rknpu_dbg_nopc;
module_param_named(dbg_nopc, rknpu_dbg_nopc, ulong, 0444);   /* #patch: matched-control samples logged */
/* #patchB4: which task_ctrl signature the HEALTHY control samples. The control MUST match the
 * stalled job's signature or the comparison is meaningless -- a 1-task pp-on stall (0x7001)
 * compared against a 2-task pp-on success (0x7002) tells you nothing. Made a live param so the
 * control can be re-aimed at whatever signature is actually stalling, without a rebuild. */
unsigned int rknpu_dbg_healthy_ctrl = 0x7001;
module_param_named(dbg_healthy_ctrl, rknpu_dbg_healthy_ctrl, uint, 0644);
/* #patch: how often rknpu_job_timeout_clean SOFT-RESETS the NPU and drops the RUNNING job.
 * It compares ktime_us_delta(now, job->timestamp) >= args->timeout -- MICROseconds against a value every
 * other site treats as MILLIseconds -- so our 1500 ("ms") is really a 1.5 ms threshold while our jobs run
 * a median 5.7 ms. Any job still executing when the next submit lands on its core is therefore older than
 * the threshold, gets the whole NPU reset under it, and is dropped: committed, never completed, no
 * interrupt. That predicts commit-done == stall count, which is what we measure.
 *   treap     = timeout_clean reset+dropped a job
 *   treap_age = age (us) of the job it killed, last occurrence */
static unsigned long rknpu_cnt_treap;
static unsigned long rknpu_cnt_treap_age;
module_param_named(cnt_treap, rknpu_cnt_treap, ulong, 0444);
module_param_named(cnt_treap_age, rknpu_cnt_treap_age, ulong, 0444);
unsigned long rknpu_cnt_commit, rknpu_cnt_irq, rknpu_cnt_nojob,
		      rknpu_cnt_unpow, rknpu_cnt_done;
module_param_named(cnt_commit, rknpu_cnt_commit, ulong, 0444);
module_param_named(cnt_irq,    rknpu_cnt_irq,    ulong, 0444);
module_param_named(cnt_nojob,  rknpu_cnt_nojob,  ulong, 0444);
module_param_named(cnt_unpow,  rknpu_cnt_unpow,  ulong, 0444);
module_param_named(cnt_done,   rknpu_cnt_done,   ulong, 0444);

#define _REG_READ(base, offset) readl(base + (offset))
#define _REG_WRITE(base, value, offset) writel(value, base + (offset))

#define REG_READ(offset) _REG_READ(rknpu_core_base, offset)
#define REG_WRITE(value, offset) _REG_WRITE(rknpu_core_base, value, offset)

static int rknpu_wait_core_index(int core_mask)
{
	int index = 0;

	switch (core_mask) {
	case RKNPU_CORE0_MASK:
	case RKNPU_CORE0_MASK | RKNPU_CORE1_MASK:
	case RKNPU_CORE0_MASK | RKNPU_CORE1_MASK | RKNPU_CORE2_MASK:
		index = 0;
		break;
	case RKNPU_CORE1_MASK:
		index = 1;
		break;
	case RKNPU_CORE2_MASK:
		index = 2;
		break;
	default:
		break;
	}

	return index;
}

static int rknpu_core_mask(int core_index)
{
	int core_mask = RKNPU_CORE_AUTO_MASK;

	switch (core_index) {
	case 0:
		core_mask = RKNPU_CORE0_MASK;
		break;
	case 1:
		core_mask = RKNPU_CORE1_MASK;
		break;
	case 2:
		core_mask = RKNPU_CORE2_MASK;
		break;
	default:
		break;
	}

	return core_mask;
}

static int rknpu_get_task_number(struct rknpu_job *job, int core_index)
{
	struct rknpu_device *rknpu_dev = job->rknpu_dev;
	int task_num = job->args->task_number;

	if (core_index >= RKNPU_MAX_CORES || core_index < 0) {
		LOG_ERROR("invalid rknpu core index: %d", core_index);
		return 0;
	}

	if (rknpu_dev->config->num_irqs > 1) {
		if (job->use_core_num == 1 || job->use_core_num == 2)
			task_num =
				job->args->subcore_task[core_index].task_number;
		else if (job->use_core_num == 3)
			task_num = job->args->subcore_task[core_index + 2]
					   .task_number;
	}

	return task_num;
}

extern unsigned long rknpu_cnt_reset_declined;
extern u64 rknpu_hw_ns_sum;
extern unsigned long rknpu_hw_n;

static void rknpu_job_free(struct rknpu_job *job)
{
#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
	struct rknpu_gem_object *task_obj = NULL;

	task_obj =
		(struct rknpu_gem_object *)(uintptr_t)job->args->task_obj_addr;
	if (task_obj)
		rknpu_gem_object_put(&task_obj->base);
#endif

	/*
	 * Release the job's own power reference here rather than in rknpu_job_done() or
	 * rknpu_job_abort(): this is the one point every job passes through exactly once,
	 * including the rknpu_job_timeout_clean() path, which reaches cleanup directly
	 * without going through either. It also has to be here for context -- job_done()
	 * runs in hard IRQ, and rknpu_power_put_delay() takes power_lock, a mutex.
	 */
	if (test_and_clear_bit(0, &job->pwr_held))
		rknpu_power_put_delay(job->rknpu_dev);

	if (job->fence) {
		/* #patch: a job torn down WITHOUT completing (rknpu_job_timeout_clean /
		 * rknpu_job_abort) never reaches the RKNPU_JOB_DONE path, so its fence is
		 * never signalled and every waiter blocks until its OWN timeout expires --
		 * which defeats the point of waiting on a fence to detect a stalled job.
		 * Signal it with an error instead, so a waiter wakes immediately and can
		 * tell a stall from a completion via dma_fence_get_status(). No-op on the
		 * success path, where the completion IRQ already signalled it. */
		if (!dma_fence_is_signaled(job->fence)) {
			dma_fence_set_error(job->fence, -ETIMEDOUT);
			dma_fence_signal(job->fence);
		}
		dma_fence_put(job->fence);
	}

	if (job->args_owner)
		kfree(job->args);

	kfree(job);
}

static int rknpu_job_cleanup(struct rknpu_job *job)
{
	rknpu_job_free(job);

	return 0;
}

static void rknpu_job_cleanup_work(struct work_struct *work)
{
	struct rknpu_job *job =
		container_of(work, struct rknpu_job, cleanup_work);

	rknpu_job_cleanup(job);
}

static inline struct rknpu_job *rknpu_job_alloc(struct rknpu_device *rknpu_dev,
						struct rknpu_submit *args)
{
	struct rknpu_job *job = NULL;
	int i = 0;
#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
	struct rknpu_gem_object *task_obj = NULL;
#endif

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (job) {
		/* #patch67: head[] must be a valid empty node from the moment
		 * the job exists. kzalloc leaves it {NULL,NULL}, and
		 * rknpu_job_schedule() can bail before queueing (a failed
		 * domain switch sets job->ret and returns) yet still reach
		 * rknpu_job_abort() -- where the unlink above would then be a
		 * NULL deref instead of a no-op. */
		int c;

		for (c = 0; c < RKNPU_MAX_CORES; c++)
			INIT_LIST_HEAD(&job->head[c]);
	}
	if (!job)
		return NULL;

	job->timestamp = ktime_get();
	job->rknpu_dev = rknpu_dev;
	job->use_core_num = (args->core_mask & RKNPU_CORE0_MASK) +
			    ((args->core_mask & RKNPU_CORE1_MASK) >> 1) +
			    ((args->core_mask & RKNPU_CORE2_MASK) >> 2);
	atomic_set(&job->run_count, job->use_core_num);
	atomic_set(&job->interrupt_count, job->use_core_num);
	job->iommu_domain_id = args->iommu_domain_id;
	for (i = 0; i < rknpu_dev->config->num_irqs; i++)
		atomic_set(&job->submit_count[i], 0);
#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
	task_obj = (struct rknpu_gem_object *)(uintptr_t)args->task_obj_addr;
	if (task_obj)
		rknpu_gem_object_get(&task_obj->base);
#endif

	if (!(args->flags & RKNPU_JOB_NONBLOCK)) {
		job->args = args;
		job->args_owner = false;
		return job;
	}

	job->args = kzalloc(sizeof(*args), GFP_KERNEL);
	if (!job->args) {
		kfree(job);
		return NULL;
	}
	*job->args = *args;
	job->args_owner = true;

	INIT_WORK(&job->cleanup_work, rknpu_job_cleanup_work);

	return job;
}

static inline int rknpu_job_wait(struct rknpu_job *job)
{
	struct rknpu_device *rknpu_dev = job->rknpu_dev;
	struct rknpu_submit *args = job->args;
	struct rknpu_task *last_task = NULL;
	struct rknpu_subcore_data *subcore_data = NULL;
	struct rknpu_job *entry, *q;
	void __iomem *rknpu_core_base = NULL;
	int core_index = rknpu_wait_core_index(job->args->core_mask);
	unsigned long flags;
	int wait_count = 0;
	bool continue_wait = false;
	int ret = -EINVAL;
	int i = 0;

	subcore_data = &rknpu_dev->subcore_datas[core_index];

	do {
		ret = wait_event_timeout(subcore_data->job_done_wq,
					 job->flags & (RKNPU_JOB_DONE |
						       RKNPU_JOB_STALLED) ||
						 rknpu_dev->soft_reseting,
					 msecs_to_jiffies(args->timeout));

		/* #patch41 (fast-abort): the watchdog woke us because this job has made no
		 * progress. Force ret=0 and leave the retry loop so we fall into the SAME
		 * `if (ret <= 0)` path a natural timeout takes -- it samples the PC counter,
		 * logs, and returns -ETIMEDOUT. Identical outcome to the old behaviour, just
		 * without waiting args->timeout x 3 for it (measured: 60.6 s). */
		if (job->flags & RKNPU_JOB_STALLED) {
			ret = 0;
			break;
		}

		if (++wait_count >= 3)
			break;

		if (ret == 0) {
			int64_t elapse_time_us = 0;
			spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
			elapse_time_us = ktime_us_delta(ktime_get(),
							job->hw_commit_time);
			continue_wait =
				job->hw_commit_time == 0 ?
					true :
					(elapse_time_us < args->timeout * 1000);
			spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
			LOG_ERROR(
				"job: %p, mask: %#x, job iommu domain id: %d, dev iommu domain id: %d, wait_count: %d, continue wait: %d, commit elapse time: %lldus, wait time: %lldus, timeout: %uus\n",
				job, args->core_mask, job->iommu_domain_id,
				rknpu_dev->iommu_domain_id, wait_count,
				continue_wait,
				(job->hw_commit_time == 0 ? 0 : elapse_time_us),
				ktime_us_delta(ktime_get(), job->timestamp),
				args->timeout * 1000);
		}
	} while (ret == 0 && continue_wait);

	last_task = job->last_task;
	if (!last_task) {
		spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
		for (i = 0; i < job->use_core_num; i++) {
			subcore_data = &rknpu_dev->subcore_datas[i];
			list_for_each_entry_safe(
				entry, q, &subcore_data->todo_list, head[i]) {
				if (entry == job) {
					list_del(&job->head[i]);
					break;
				}
			}
		}
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

		LOG_ERROR("job commit failed\n");
		return ret < 0 ? ret : -EINVAL;
	}

	last_task->int_status = job->int_status[core_index];

	if (ret <= 0) {
		args->task_counter = 0;
		rknpu_core_base = rknpu_dev->base[core_index];
		if (args->flags & RKNPU_JOB_PC) {
			uint32_t task_status = REG_READ(
				rknpu_dev->config->pc_task_status_offset);
			args->task_counter =
				(task_status &
				 rknpu_dev->config->pc_task_number_mask);
		}

		LOG_ERROR(
			"failed to wait job, task counter: %d, flags: %#x, ret = %d, elapsed time: %lldus\n",
			args->task_counter, args->flags, ret,
			ktime_us_delta(ktime_get(), job->timestamp));

		return ret < 0 ? ret : -ETIMEDOUT;
	}

	if (!(job->flags & RKNPU_JOB_DONE))
		return -EINVAL;

	args->task_counter = args->task_number;
	args->hw_elapse_time = job->hw_elapse_time;

	return 0;
}

static inline int rknpu_job_subcore_commit_pc(struct rknpu_job *job,
					      int core_index)
{
	struct rknpu_device *rknpu_dev = job->rknpu_dev;
	struct rknpu_submit *args = job->args;
#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
	struct rknpu_gem_object *task_obj =
		(struct rknpu_gem_object *)(uintptr_t)args->task_obj_addr;
#endif
#ifdef CONFIG_ROCKCHIP_RKNPU_DMA_HEAP
	struct rknpu_mem_object *task_obj =
		(struct rknpu_mem_object *)(uintptr_t)args->task_obj_addr;
#endif
	struct rknpu_task *task_base = NULL;
	struct rknpu_task *first_task = NULL;
	struct rknpu_task *last_task = NULL;
	void __iomem *rknpu_core_base = rknpu_dev->base[core_index];
	int task_start = args->task_start;
	int task_end;
	int task_number = args->task_number;
	int task_pp_en = args->flags & RKNPU_JOB_PINGPONG ? 1 : 0;
	int pc_data_amount_scale = rknpu_dev->config->pc_data_amount_scale;
	int pc_task_number_bits = rknpu_dev->config->pc_task_number_bits;
	int i = 0;
	int submit_index = atomic_read(&job->submit_count[core_index]);

	if (static_branch_unlikely(&rknpu_dbg_key))
		rknpu_cnt_commit++;   /* #patch: hot path -- see `dbg` in rknpu_drv.c */
	int max_submit_number = rknpu_dev->config->max_submit_number;
	unsigned long flags;

	if (!task_obj) {
		job->ret = -EINVAL;
		return job->ret;
	}

	if (rknpu_dev->config->num_irqs > 1) {
		for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
			if (i == core_index) {
				REG_WRITE((0xe + 0x10000000 * i), 0x1004);
				REG_WRITE((0xe + 0x10000000 * i), 0x3004);
			}
		}

		switch (job->use_core_num) {
		case 1:
		case 2:
			task_start = args->subcore_task[core_index].task_start;
			task_number =
				args->subcore_task[core_index].task_number;
			break;
		case 3:
			task_start =
				args->subcore_task[core_index + 2].task_start;
			task_number =
				args->subcore_task[core_index + 2].task_number;
			break;
		default:
			LOG_ERROR("Unknown use core num %d\n",
				  job->use_core_num);
			break;
		}
	}

	task_start = task_start + submit_index * max_submit_number;
	task_number = task_number - submit_index * max_submit_number;
	task_number = task_number > max_submit_number ? max_submit_number :
							task_number;
	task_end = task_start + task_number - 1;

	task_base = task_obj->kv_addr;

	first_task = &task_base[task_start];
	last_task = &task_base[task_end];

	if (rknpu_dev->config->pc_dma_ctrl) {
		spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
		REG_WRITE(first_task->regcmd_addr, RKNPU_OFFSET_PC_DATA_ADDR);
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
	} else {
		REG_WRITE(first_task->regcmd_addr, RKNPU_OFFSET_PC_DATA_ADDR);
	}

	REG_WRITE((first_task->regcfg_amount + RKNPU_PC_DATA_EXTRA_AMOUNT +
		   pc_data_amount_scale - 1) /
				  pc_data_amount_scale -
			  1,
		  RKNPU_OFFSET_PC_DATA_AMOUNT);

	REG_WRITE(last_task->int_mask | rknpu_int_mask_extra,   /* #patch66 */
		  RKNPU_OFFSET_INT_MASK);

	REG_WRITE(first_task->int_mask | rknpu_int_mask_extra,  /* #patch66 */
		  RKNPU_OFFSET_INT_CLEAR);

	REG_WRITE(((0x6 | task_pp_en) << pc_task_number_bits) | task_number,
		  RKNPU_OFFSET_PC_TASK_CONTROL);

	REG_WRITE(args->task_base_addr, RKNPU_OFFSET_PC_DMA_BASE_ADDR);

	job->first_task = first_task;
	job->last_task = last_task;
	job->int_mask[core_index] = last_task->int_mask;

	/* #patch: snapshot what we WROTE vs what reads BACK, before and after the start pulse.
	 * Distinguishes "the write never landed" (readback zero/garbage -> ordering, barrier or
	 * gated clocks) from "it landed and the PC ignored it" (hardware state machine). */
	job->dbg_wr[0] = first_task->regcmd_addr;
	job->dbg_wr[1] = (first_task->regcfg_amount + RKNPU_PC_DATA_EXTRA_AMOUNT +
			  pc_data_amount_scale - 1) / pc_data_amount_scale - 1;
	job->dbg_wr[2] = last_task->int_mask;
	job->dbg_wr[3] = ((0x6 | task_pp_en) << pc_task_number_bits) | task_number;
	job->dbg_wr[4] = (uint32_t)args->task_base_addr;
	job->dbg_wr[5] = task_number;
	/* #patch: the sequence userspace stamped into this descriptor (rknpu_task.op_idx, a field the
	 * driver otherwise ignores). If a STALLED commit reports an OLDER stamp than the ones healthy
	 * commits are reporting, the kernel read a STALE task array through its kernel mapping. */
	job->dbg_seq = first_task->op_idx;
	job->dbg_rd[0] = REG_READ(RKNPU_OFFSET_PC_DATA_ADDR);
	job->dbg_rd[1] = REG_READ(RKNPU_OFFSET_PC_DATA_AMOUNT);
	job->dbg_rd[2] = REG_READ(RKNPU_OFFSET_INT_MASK);
	job->dbg_rd[3] = REG_READ(RKNPU_OFFSET_PC_TASK_CONTROL);
	job->dbg_rd[4] = REG_READ(RKNPU_OFFSET_PC_DMA_BASE_ADDR);
	job->dbg_rd[5] = REG_READ(rknpu_dev->config->pc_task_status_offset);

	REG_WRITE(0x1, RKNPU_OFFSET_PC_OP_EN);
	REG_WRITE(0x0, RKNPU_OFFSET_PC_OP_EN);

	job->dbg_rd[6] = REG_READ(rknpu_dev->config->pc_task_status_offset);
	/* #patch: full PC/INT block AFTER the start pulse. Map (vendor rknpu_ioctl.h):
	 *   0x00 VERSION      0x04 VERSION_NUM  0x08 PC_OP_EN     0x10 PC_DATA_ADDR
	 *   0x14 PC_DATA_AMT  0x20 INT_MASK     0x24 INT_CLEAR    0x28 INT_STATUS
	 *   0x2c INT_RAW      0x30 PC_TASK_CTRL 0x34 PC_DMA_BASE  0x3c PC_TASK_STATUS
	 * plus ENABLE_MASK at 0xf008. Everything we WRITE is already verified identical between a
	 * healthy and a stalled commit; this samples the registers we do NOT write, to see whether the
	 * PC's own state differs. If nothing differs, the fault is not visible at this level. */
	{ int _i; for (_i = 0; _i < 16; _i++) job->dbg_blk[_i] = REG_READ(_i * 4);
	  job->dbg_blk[16] = REG_READ(RKNPU_OFFSET_ENABLE_MASK); }
	job->dbg_valid = true;

	return 0;
}

static inline int rknpu_job_subcore_commit(struct rknpu_job *job,
					   int core_index)
{
	struct rknpu_device *rknpu_dev = job->rknpu_dev;
	struct rknpu_submit *args = job->args;
	void __iomem *rknpu_core_base = rknpu_dev->base[core_index];
	unsigned long flags;

	// switch to slave mode
	if (rknpu_dev->config->pc_dma_ctrl) {
		spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
		REG_WRITE(0x1, RKNPU_OFFSET_PC_DATA_ADDR);
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
	} else {
		REG_WRITE(0x1, RKNPU_OFFSET_PC_DATA_ADDR);
	}

	if (!(args->flags & RKNPU_JOB_PC)) {
		job->ret = -EINVAL;
		return job->ret;
	}

	return rknpu_job_subcore_commit_pc(job, core_index);
}

static void rknpu_job_commit(struct rknpu_job *job)
{
	switch (job->args->core_mask) {
	case RKNPU_CORE0_MASK:
		rknpu_job_subcore_commit(job, 0);
		break;
	case RKNPU_CORE1_MASK:
		rknpu_job_subcore_commit(job, 1);
		break;
	case RKNPU_CORE2_MASK:
		rknpu_job_subcore_commit(job, 2);
		break;
	case RKNPU_CORE0_MASK | RKNPU_CORE1_MASK:
		rknpu_job_subcore_commit(job, 0);
		rknpu_job_subcore_commit(job, 1);
		break;
	case RKNPU_CORE0_MASK | RKNPU_CORE1_MASK | RKNPU_CORE2_MASK:
		rknpu_job_subcore_commit(job, 0);
		rknpu_job_subcore_commit(job, 1);
		rknpu_job_subcore_commit(job, 2);
		break;
	default:
		LOG_ERROR("Unknown core mask: %d\n", job->args->core_mask);
		break;
	}
}


static void rknpu_job_next(struct rknpu_device *rknpu_dev, int core_index)
{
	struct rknpu_job *job = NULL;
	struct rknpu_subcore_data *subcore_data = NULL;
	unsigned long flags;

	if (rknpu_dev->soft_reseting) {
		/*
		 * Dispatch declined because a reset is in progress. If the caller was the completion
		 * path, subcore_data->job has just been cleared, so this core is now idle with a
		 * possibly non-empty queue and nothing else will come along to promote it -- see
		 * rknpu_job_redrive(). Same rare path as the reset itself, so counting is free.
		 */
		rknpu_cnt_reset_declined++;
		return;
	}

	subcore_data = &rknpu_dev->subcore_datas[core_index];

	spin_lock_irqsave(&rknpu_dev->irq_lock, flags);

	/* #patch71: THE "COMMITTED, NEVER COMPLETES" DECISION POINT.
	 *
	 * A submit that userspace saw succeed can still never reach the hardware: if this core is already
	 * owned (subcore_data->job != NULL) the new job stays on todo_list and nothing re-drives it until
	 * that owner retires. When the owner is a job that TIMED OUT and was aborted but never reaped, it
	 * never retires -- so every later submit on this core is accepted and silently never dispatched.
	 * Userspace sees only "doorbell sentinel never landed", with no kernel error, which is exactly the
	 * signature this project has repeatedly mistaken for silicon.
	 *
	 * Record it: count the blocks, remember how long the owner has been sitting there, and log the first
	 * few with the owner's identity. Age is the discriminator -- a few hundred microseconds is a healthy
	 * core that is simply busy; seconds means a stuck owner and a permanently undispatchable queue. */
	if (subcore_data->job) {
		struct rknpu_job *own = subcore_data->job;
		s64 age_us = own->hw_commit_time ?
				     ktime_us_delta(ktime_get(), own->hw_commit_time) : -1;
		if (static_branch_unlikely(&rknpu_dbg_key))
			rknpu_dbg_next_blocked++;
		/* Only the SLOW blocks are interesting; a busy core blocks constantly and harmlessly. */
		if (age_us > 1000000) {
			rknpu_dbg_blocked_slow++;
			rknpu_dbg_blocked_age_us = (long)age_us;
		}
		if (age_us > 1000000 && rknpu_dbg_next_blocked_logged < 16) {
			rknpu_dbg_next_blocked_logged++;
			LOG_ERROR("core %d: NOT dispatching -- still owned by job %p (flags %#x, age %lldus, "
				  "int_cnt %d, run_cnt %d); queued work will NOT run until it retires\n",
				  core_index, own, own->flags, age_us,
				  atomic_read(&own->interrupt_count), atomic_read(&own->run_count));
		}
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
		return;
	}
	if (list_empty(&subcore_data->todo_list)) {
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
		return;
	}

	job = list_first_entry(&subcore_data->todo_list, struct rknpu_job,
			       head[core_index]);

	list_del_init(&job->head[core_index]);
	subcore_data->job = job;
	job->hw_commit_time = ktime_get();
	/* #patch47 (profiler): record the gap since the PREVIOUS commit on this core, so a stalled
	 * job's timeline can be compared against a healthy one's. Two questions this answers:
	 * does a stall correlate with back-to-back commits (too little settle between jobs), or
	 * with a long queue delay (submit -> commit)? Both are free here -- job->timestamp and
	 * hw_commit_time already exist; only the per-core previous-commit time is new. */
	job->prof_gap_us = rknpu_prof_last_commit[core_index] ?
		ktime_us_delta(job->hw_commit_time,
			       rknpu_prof_last_commit[core_index]) : 0;
	job->prof_queue_us = ktime_us_delta(job->hw_commit_time, job->timestamp);
	rknpu_prof_last_commit[core_index] = job->hw_commit_time;
	job->hw_recoder_time = job->hw_commit_time;
	spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

	/* #patch50: shape axis. The timing hypothesis failed its causal test (enforcing spacing
	 * gave no dose-response and inverted at 1500us), so the working theory is that a short gap
	 * is a SYMPTOM of which op is running. Log the shape of every commit so the stalled and
	 * healthy shape distributions can be compared directly. */
	job->prof_since_switch = rknpu_commits_since_switch++;   /* #patch51 */
	job->prof_tsince_us = rknpu_last_switch_time ?   /* #patch52 */
		ktime_us_delta(job->hw_commit_time, rknpu_last_switch_time) : -1;
	if (rknpu_prof_mask_get() & 2)
		LOG_ERROR("RKNPU: PROF commit core=%d tn=%u cm=%#x dom=%d sinceswitch=%u tsince=%lldus gap=%lldus\n",
			  core_index, job->args->task_number,
			  job->args->core_mask, job->iommu_domain_id,
			  job->prof_since_switch, job->prof_tsince_us, job->prof_gap_us);

	/* #patch49: enforce a minimum spacing between commits on this core.
	 *
	 * Placed here so the enforcement matches where #patch47 MEASURES the gap -- same path, same
	 * core, apples to apples. udelay() not usleep_range(): this can be reached from the
	 * completion IRQ. Clamped, and skipped entirely when the knob is 0. */
	{
		unsigned int minus = rknpu_min_commit_gap_get();

		if (minus && job->prof_gap_us >= 0 &&
		    job->prof_gap_us < (s64)minus) {
			unsigned int need = minus - (unsigned int)job->prof_gap_us;

			if (need > 5000)
				need = 5000;
			udelay(need);
			rknpu_gap_enforced_n++;
			rknpu_gap_enforced_us += need;
		}
	}

	if (atomic_dec_and_test(&job->run_count))
		rknpu_job_commit(job);
}

/* #patch72: RE-DRIVE THE QUEUE AFTER DISPATCH WAS DISABLED.
 *
 * rknpu_job_next() bails for every core while soft_reseting is set, and its only other callers are the
 * completion path, rknpu_job_schedule() and timeout_clean. So a job submitted DURING a soft-reset window
 * is put on todo_list, declined once, and then never re-driven: the reset clears the flag and nothing
 * promotes the queue. The core is idle, no completion IRQ will ever arrive to call job_next again, and the
 * job sits there forever.
 *
 * Userspace sees a submit that SUCCEEDED and then an output that never lands -- with no kernel error,
 * because nothing failed. MEASURED from userspace at the moment of the stall: cnt_commit+0, cnt_irq+0,
 * cnt_done+0, submit_dom == weight_dom (so not a page-table mismatch), core_mask=0x1 -- i.e. the job never
 * reached the hardware at all.
 *
 * Call this after clearing soft_reseting. Idempotent: job_next is a no-op for a core that is already
 * owned or whose queue is empty. */
unsigned long rknpu_cnt_reset_declined;
module_param_named(cnt_reset_declined, rknpu_cnt_reset_declined, ulong, 0444);

/* #patch74: see rknpu_job_done. Writable (0644) so a userspace probe can zero them around a timed
 * region and get a clean per-job hardware average. */
u64 rknpu_hw_ns_sum;
unsigned long rknpu_hw_n;
module_param_named(hw_ns_sum, rknpu_hw_ns_sum, ullong, 0644);
module_param_named(hw_n, rknpu_hw_n, ulong, 0644);
MODULE_PARM_DESC(cnt_reset_declined,
		 "dispatch attempts declined because a soft reset was in progress");

unsigned long rknpu_cnt_stranded;
module_param_named(cnt_stranded, rknpu_cnt_stranded, ulong, 0444);
MODULE_PARM_DESC(cnt_stranded,
		 "jobs found queued on an idle core after dispatch was re-enabled (would have hung)");

void rknpu_job_redrive(struct rknpu_device *rknpu_dev)
{
	unsigned long flags;
	int i;

	for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
		struct rknpu_subcore_data *subcore_data =
			&rknpu_dev->subcore_datas[i];
		bool stranded;

		/*
		 * An idle core with a non-empty queue is precisely the state this function exists to
		 * fix: nothing owns the core, so no completion interrupt is coming to promote the
		 * queue. Count it -- on a driver without this call every one of these was a job that
		 * never ran, and the count is the only direct evidence the window is real.
		 * Cheap: soft resets are rare, and this runs once per reset per core.
		 */
		spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
		stranded = !subcore_data->job &&
			   !list_empty(&subcore_data->todo_list);
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

		if (stranded)
			rknpu_cnt_stranded++;

		rknpu_job_next(rknpu_dev, i);
	}
}

static void rknpu_job_done(struct rknpu_job *job, int ret, int core_index)
{
	struct rknpu_device *rknpu_dev = job->rknpu_dev;
	struct rknpu_subcore_data *subcore_data = NULL;
	ktime_t now;
	unsigned long flags;
	int max_submit_number = rknpu_dev->config->max_submit_number;

	if (atomic_inc_return(&job->submit_count[core_index]) <
	    (rknpu_get_task_number(job, core_index) + max_submit_number - 1) /
		    max_submit_number) {
		rknpu_job_subcore_commit(job, core_index);
		return;
	}

	subcore_data = &rknpu_dev->subcore_datas[core_index];

	spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
	subcore_data->job = NULL;
	subcore_data->task_num -= rknpu_get_task_number(job, core_index);
	now = ktime_get();
	job->hw_elapse_time = ktime_sub(now, job->hw_commit_time);
	/* #patch74: HARDWARE time per job, summed in the kernel. Userspace measures completion by polling a
	 * DRAM sentinel, so its "poll time" is hardware time PLUS however long it takes to NOTICE. A probe and
	 * a real consumer disagree 2x on the same shape with everything measurable held equal (twelve causes
	 * disproven), and this is the one split userspace cannot make: if hw_ns_sum/hw_n agrees across the two,
	 * the hardware is fine and the gap is detection latency; if it disagrees, the hardware is genuinely
	 * slower. One add per completion, on the normal path. */
	rknpu_hw_ns_sum += (u64)ktime_to_ns(job->hw_elapse_time);
	rknpu_hw_n++;
	subcore_data->timer.busy_time += ktime_sub(now, job->hw_recoder_time);
	spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

	if (atomic_dec_and_test(&job->interrupt_count)) {
		int use_core_num = job->use_core_num;

		if (static_branch_unlikely(&rknpu_dbg_key))
			rknpu_cnt_done++;   /* #patch: hot path */
		if (job->dbg_kick_t)
			LOG_ERROR("RKNPU:   KICKED JOB COMPLETED %lldus after the kick\n",
				  ktime_us_delta(ktime_get(), job->dbg_kick_t));

		/* #patch (experiment control): dump a SUCCESSFUL job's commit snapshot every 2000th
		 * completion, so the stalled-job dump has a same-run baseline to compare against.
		 * Without this the stalled readback is uninterpretable — we cannot tell an abnormal
		 * value from simply how the register reads back. */
		/* MATCHED control: only sample successful jobs with the SAME signature as the stalled
		 * ones (task_ctrl 0x7002 = ping-pong on, 2 tasks). An unmatched control compared a
		 * 1-task pp-off job against a 2-task pp-on stall and was uninterpretable. */
		/* Log the FIRST few matching successes, never a modulo. Sampling every 500th
		 * completion aliased: the workload repeats a 4-job cycle (one real 0x7002 op plus
		 * three 0x6001 drain dummies) and 500 %% 4 == 0, so the sampler locked to one phase
		 * and captured ZERO matching jobs. */
		if (job->dbg_valid && job->dbg_wr[3] == rknpu_dbg_healthy_ctrl &&
	    rknpu_dbg_healthy_n < 3 &&
		    ++rknpu_dbg_healthy_n) {
			LOG_ERROR("RKNPU: HEALTHY commit snapshot: WROTE data_addr=%#x amount=%#x int_mask=%#x task_ctrl=%#x dma_base=%#x tasks=%u | READBACK data_addr=%#x amount=%#x int_mask=%#x task_ctrl=%#x dma_base=%#x status(pre)=%#x status(post)=%#x\n",
				  job->dbg_wr[0], job->dbg_wr[1], job->dbg_wr[2],
				  job->dbg_wr[3], job->dbg_wr[4], job->dbg_wr[5],
				  job->dbg_rd[0], job->dbg_rd[1], job->dbg_rd[2],
				  job->dbg_rd[3], job->dbg_rd[4], job->dbg_rd[5],
				  job->dbg_rd[6]);
			LOG_ERROR("RKNPU: HEALTHY PCBLK %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x | en=%08x\n",
				  job->dbg_blk[0], job->dbg_blk[1], job->dbg_blk[2], job->dbg_blk[3],
				  job->dbg_blk[4], job->dbg_blk[5], job->dbg_blk[6], job->dbg_blk[7],
				  job->dbg_blk[8], job->dbg_blk[9], job->dbg_blk[10], job->dbg_blk[11],
				  job->dbg_blk[12], job->dbg_blk[13], job->dbg_blk[14], job->dbg_blk[15],
				  job->dbg_blk[16]);
		}

		if (test_and_clear_bit(0, &job->dom_held))   /* #patch64 */
			rknpu_iommu_domain_put(rknpu_dev);

		job->flags |= RKNPU_JOB_DONE;
		/* #patch48: healthy baseline for the profiler — same two quantities the stall path
		 * records, so the comparison is like-for-like. */
		if (!(job->flags & RKNPU_JOB_STALLED)) {
			rknpu_prof_ok_n++;
			rknpu_prof_ok_gap += (unsigned long)job->prof_gap_us;
			rknpu_prof_ok_queue += (unsigned long)job->prof_queue_us;
		}
		job->ret = ret;

		if (job->fence)
			dma_fence_signal(job->fence);

		if (job->flags & RKNPU_JOB_ASYNC)
			schedule_work(&job->cleanup_work);

		if (use_core_num > 1)
			wake_up(&(&rknpu_dev->subcore_datas[0])->job_done_wq);
		else
			wake_up(&subcore_data->job_done_wq);
	}

	rknpu_job_next(rknpu_dev, core_index);
}

static int rknpu_schedule_core_index(struct rknpu_device *rknpu_dev)
{
	int core_num = rknpu_dev->config->num_irqs;
	int task_num = rknpu_dev->subcore_datas[0].task_num;
	int core_index = 0;
	int i = 0;

	for (i = 1; i < core_num; i++) {
		if (task_num > rknpu_dev->subcore_datas[i].task_num) {
			core_index = i;
			task_num = rknpu_dev->subcore_datas[i].task_num;
		}
	}

	return core_index;
}

static void rknpu_job_schedule(struct rknpu_job *job)
{
	struct rknpu_device *rknpu_dev = job->rknpu_dev;
	struct rknpu_subcore_data *subcore_data = NULL;
	int i = 0, core_index = 0;
	unsigned long flags;

	if (job->args->core_mask == RKNPU_CORE_AUTO_MASK) {
		core_index = rknpu_schedule_core_index(rknpu_dev);
		job->args->core_mask = rknpu_core_mask(core_index);
		job->use_core_num = 1;
		atomic_set(&job->run_count, job->use_core_num);
		atomic_set(&job->interrupt_count, job->use_core_num);
	}

	if (rknpu_iommu_domain_get_and_switch(rknpu_dev, job->iommu_domain_id)) {
		job->ret = -EINVAL;
		return;
	}
	/* #patch64: this job now OWNS a domain reference. Exactly one release may act on it. */
	set_bit(0, &job->dom_held);

	spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
	for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
		if (job->args->core_mask & rknpu_core_mask(i)) {
			subcore_data = &rknpu_dev->subcore_datas[i];
			list_add_tail(&job->head[i], &subcore_data->todo_list);
			subcore_data->task_num += rknpu_get_task_number(job, i);
		}
	}
	spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

	for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
		if (job->args->core_mask & rknpu_core_mask(i))
			rknpu_job_next(rknpu_dev, i);
	}
}

static void rknpu_job_abort(struct rknpu_job *job)
{
	struct rknpu_device *rknpu_dev = job->rknpu_dev;
	struct rknpu_subcore_data *subcore_data = NULL;
	unsigned long flags;
	int i = 0;

	if (test_and_clear_bit(0, &job->dom_held))   /* #patch64 */
		rknpu_iommu_domain_put(rknpu_dev);

	msleep(100);

	spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
	for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
		if (job->args->core_mask & rknpu_core_mask(i)) {
			subcore_data = &rknpu_dev->subcore_datas[i];
			if (job == subcore_data->job) {
				/* #patch35: clear UNCONDITIONALLY.
				 *
				 * Upstream gates this on !job->irq_entry[i], but rknpu_job_cleanup()
				 * below frees the job either way -- so whenever irq_entry[i] is set
				 * this left a DANGLING pointer in subcore_data->job. The IRQ handler,
				 * rknpu_job_next() and the progress watchdog all read that field, so
				 * the next reader touches freed slab. The resulting corruption panics
				 * somewhere unrelated seconds later (observed:
				 * __memcg_kmem_charge_page via fork from dropbear, and
				 * sg_free_table/__free_pages), which is why it never looked like an
				 * NPU bug. Reached whenever an aborted job is stuck -- e.g. after
				 * "switch iommu domain time out".
				 *
				 * Keep the task_num adjustment gated as before so the accounting is
				 * unchanged; only the freed-pointer publication is fixed. */
				if (!job->irq_entry[i])
					subcore_data->task_num -=
						rknpu_get_task_number(job, i);
				subcore_data->job = NULL;
			}
			/* #patch67: drop the job from this core's todo_list
			 * before rknpu_job_cleanup() frees it below.
			 *
			 * #patch35 above fixes the RUNNING job's dangling
			 * pointer; this fixes the QUEUED one. A job aborted
			 * while still on todo_list stayed linked, so the next
			 * rknpu_job_next() list_first_entry()'d it and
			 * list_del_init()'d through freed slab -- a WRITE to a
			 * wild address (WnR=1), oopsing in rknpu_job_next via
			 * rknpu_job_schedule/rknpu_submit_ioctl. Opened by a
			 * concurrent domain switch reaping jobs
			 * (rknpu_iommu_domain_get_and_switch ->
			 * rknpu_reap_all_cores -> rknpu_job_timeout_clean)
			 * while another thread submits.
			 *
			 * rknpu_job_wait() already does this on its "job commit
			 * failed" path; the abort path did not. list_del_init()
			 * is idempotent, so it is safe for a job that
			 * rknpu_job_next() already dequeued. */
			list_del_init(&job->head[i]);
		}
	}
	spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

	if (job->ret == -ETIMEDOUT) {
		LOG_ERROR("job timeout, flags: %#x:\n", job->flags);
		for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
			if (job->args->core_mask & rknpu_core_mask(i)) {
				void __iomem *rknpu_core_base =
					rknpu_dev->base[i];
				LOG_ERROR(
					"\tcore %d irq status: %#x, raw status: %#x, require mask: %#x, task counter: %#x, elapsed time: %lldus\n",
					i, REG_READ(RKNPU_OFFSET_INT_STATUS),
					REG_READ(RKNPU_OFFSET_INT_RAW_STATUS),
					job->int_mask[i],
					(REG_READ(
						 rknpu_dev->config
							 ->pc_task_status_offset) &
					 rknpu_dev->config->pc_task_number_mask),
					ktime_us_delta(ktime_get(),
						       job->timestamp));
			}
		}
		/* #patch42: a FAST-ABORTed job (#patch41) also lands here with -ETIMEDOUT, but a
		 * device-wide rknpu_soft_reset() is the wrong hammer for it. Fast-abort fires
		 * ~30x more often than the natural 60 s timeout it replaces, and each device-wide
		 * reset disrupts healthy jobs on the OTHER cores. Measured: turning fast-abort on
		 * cut submit failures 14 -> 2 but pushed the worst in-kernel wait 61.5 s -> 184.3 s,
		 * because the extra resets stalled everyone else. Reset only the cores this job
		 * actually owns; the device-wide path is kept for genuine timeouts. */
		if (job->flags & RKNPU_JOB_STALLED) {
			for (i = 0; i < rknpu_dev->config->num_irqs; i++)
				if (job->args->core_mask & rknpu_core_mask(i))
					rknpu_soft_reset_core(rknpu_dev, i);
		} else {
			rknpu_soft_reset(rknpu_dev);
		}
	} else {
		LOG_ERROR(
			"job abort, flags: %#x, ret: %d, elapsed time: %lldus\n",
			job->flags, job->ret,
			ktime_us_delta(ktime_get(), job->timestamp));
	}

	rknpu_job_cleanup(job);
}

static inline uint32_t rknpu_fuzz_status(uint32_t status)
{
	uint32_t fuzz_status = 0;

	if ((status & 0x3) != 0)
		fuzz_status |= 0x3;

	if ((status & 0xc) != 0)
		fuzz_status |= 0xc;

	if ((status & 0x30) != 0)
		fuzz_status |= 0x30;

	if ((status & 0xc0) != 0)
		fuzz_status |= 0xc0;

	if ((status & 0x300) != 0)
		fuzz_status |= 0x300;

	if ((status & 0xc00) != 0)
		fuzz_status |= 0xc00;

	return fuzz_status;
}

static inline irqreturn_t rknpu_irq_handler(int irq, void *data, int core_index)
{
	struct rknpu_device *rknpu_dev = data;
	void __iomem *rknpu_core_base = rknpu_dev->base[core_index];
	struct rknpu_subcore_data *subcore_data = NULL;
	struct rknpu_job *job = NULL;
	uint32_t status = 0;
	unsigned long flags;

	/* #patch: NEVER touch NPU MMIO when the block is powered down.
	 *
	 * Both paths below access registers unconditionally -- the no-job path does
	 * REG_WRITE(RKNPU_INT_CLEAR), and the normal path does REG_READ(INT_STATUS).
	 * rknpu_power_off() is driven by a DEFERRED work item (rknpu_power_off_delay_work),
	 * so a late or spurious interrupt can arrive after power has gone. The register read
	 * then takes an external abort and the machine dies instantly -- no console output,
	 * no ping, requiring a power cycle.
	 *
	 * CAPTURED VIA NETCONSOLE 2026-08-26 (this is what that panic looked like):
	 *   pc : readl+0x4/0x20
	 *   lr : rknpu_irq_handler.isra.0+0x94/0x2f0
	 *   Call trace: readl / rknpu_core0_irq_handler / __handle_irq_event_percpu
	 *               ... el1_interrupt / cpuidle_enter / do_idle
	 * CPU 0 was IDLE -- i.e. the NPU had finished and powered down, and the IRQ landed after.
	 *
	 * power_refcount is an atomic, so unlike power_lock (a mutex) it is safe to read from
	 * hard-IRQ context. The check is racy in principle but closes the real window, which is
	 * an interrupt arriving well after power-off. If we are unpowered the NPU cannot be
	 * asserting anything, so IRQ_NONE is correct and cannot cause a level-IRQ storm. */
	if (atomic_read(&rknpu_dev->power_refcount) <= 0) {
		rknpu_cnt_unpow++;   /* #patch: NOT gated -- standing canary, 0 on a healthy driver */
		return IRQ_NONE;
	}
	if (static_branch_unlikely(&rknpu_dbg_key))
		rknpu_cnt_irq++;   /* #patch: hot path */

	subcore_data = &rknpu_dev->subcore_datas[core_index];

	spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
	job = subcore_data->job;
	if (!job) {
		rknpu_cnt_nojob++;   /* #patch: NOT gated -- anomaly path */
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
		REG_WRITE(RKNPU_INT_CLEAR | rknpu_int_mask_extra,   /* #patch66 */
			  RKNPU_OFFSET_INT_CLEAR);
		rknpu_job_next(rknpu_dev, core_index);
		return IRQ_HANDLED;
	}
	job->irq_entry[core_index] = true;
	spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);

	status = REG_READ(RKNPU_OFFSET_INT_STATUS);
	/* #patch66: did the previously-masked condition actually get delivered? */
	if (rknpu_int_mask_extra && (status & rknpu_int_mask_extra)) {
		if (++rknpu_int_extra_fired <= 8)
			LOG_ERROR("RKNPU: core %d EXTRA INT delivered: status=%#x raw=%#x task counter=%#x\n",
				  core_index, status,
				  REG_READ(RKNPU_OFFSET_INT_RAW_STATUS),
				  REG_READ(rknpu_dev->config->pc_task_status_offset) &
					  rknpu_dev->config->pc_task_number_mask);
		if (rknpu_int_extra_fired > 2000) {
			rknpu_int_mask_extra = 0;   /* storm guard: disarm, do not hang the board */
			LOG_ERROR("RKNPU: EXTRA INT storm (>2000) — disarming int_mask_extra\n");
		}
	}

	job->int_status[core_index] = status;

	if (rknpu_fuzz_status(status) != job->int_mask[core_index]) {
		LOG_ERROR(
			"invalid irq status: %#x, raw status: %#x, require mask: %#x, task counter: %#x\n",
			status, REG_READ(RKNPU_OFFSET_INT_RAW_STATUS),
			job->int_mask[core_index],
			(REG_READ(rknpu_dev->config->pc_task_status_offset) &
			 rknpu_dev->config->pc_task_number_mask));
		REG_WRITE(RKNPU_INT_CLEAR | rknpu_int_mask_extra,   /* #patch66 */
			  RKNPU_OFFSET_INT_CLEAR);
		return IRQ_HANDLED;
	}

	REG_WRITE(RKNPU_INT_CLEAR | rknpu_int_mask_extra,   /* #patch66 */
			  RKNPU_OFFSET_INT_CLEAR);

	rknpu_job_done(job, 0, core_index);

	return IRQ_HANDLED;
}

irqreturn_t rknpu_core0_irq_handler(int irq, void *data)
{
	return rknpu_irq_handler(irq, data, 0);
}

irqreturn_t rknpu_core1_irq_handler(int irq, void *data)
{
	return rknpu_irq_handler(irq, data, 1);
}

irqreturn_t rknpu_core2_irq_handler(int irq, void *data)
{
	return rknpu_irq_handler(irq, data, 2);
}

static void rknpu_job_timeout_clean(struct rknpu_device *rknpu_dev,
				    int core_mask)
{
	struct rknpu_job *job = NULL;
	unsigned long flags;
	struct rknpu_subcore_data *subcore_data = NULL;
	int i = 0;

	for (i = 0; i < rknpu_dev->config->num_irqs; i++) {
		if (core_mask & rknpu_core_mask(i)) {
			subcore_data = &rknpu_dev->subcore_datas[i];
			job = subcore_data->job;
			if (job &&
			    ktime_us_delta(ktime_get(), job->timestamp) >=
				    job->args->timeout) {
				rknpu_cnt_treap++;   /* #patch */
				rknpu_cnt_treap_age =
					ktime_us_delta(ktime_get(), job->timestamp);
				rknpu_soft_reset(rknpu_dev);

				spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
				subcore_data->job = NULL;
				spin_unlock_irqrestore(&rknpu_dev->irq_lock,
						       flags);

				if (test_and_clear_bit(0, &job->dom_held))   /* #patch64 */
					rknpu_iommu_domain_put(rknpu_dev); /* #patch: reap the domain ref this timed-out job leaked (job_done/job_abort put; timeout_clean forgot) -> fixes multi-domain switch wedge */
				do {
					schedule_work(&job->cleanup_work);

					spin_lock_irqsave(&rknpu_dev->irq_lock,
							  flags);

					if (!list_empty(
						    &subcore_data->todo_list)) {
						job = list_first_entry(
							&subcore_data->todo_list,
							struct rknpu_job,
							head[i]);
						list_del_init(&job->head[i]);
					} else {
						job = NULL;
					}

					spin_unlock_irqrestore(
						&rknpu_dev->irq_lock, flags);
				} while (job);
			}
		}
	}
}

/* #patch54: reap every core's timed-out job, releasing the iommu domain references they hold.
 *
 * Exported for the domain-switch escalation in rknpu_iommu.c. rknpu_job_timeout_clean() is not a
 * watchdog -- it only runs when ANOTHER submit arrives -- so when a stuck job pins the domain
 * refcount above zero there is nothing that will ever reap it, and every subsequent switch burns
 * its full 6 s timeout forever. */
void rknpu_reap_all_cores(struct rknpu_device *rknpu_dev)
{
	uint32_t mask = 0;
	int i;

	for (i = 0; i < rknpu_dev->config->num_irqs; i++)
		mask |= rknpu_core_mask(i);
	rknpu_job_timeout_clean(rknpu_dev, mask);
}

static int rknpu_submit(struct rknpu_device *rknpu_dev,
			struct rknpu_submit *args)
{
	struct rknpu_job *job = NULL;
	int ret = -EINVAL;

	if (args->task_number == 0) {
		LOG_ERROR("invalid rknpu task number!\n");
		return -EINVAL;
	}

	if (args->core_mask > rknpu_dev->config->core_mask) {
		LOG_ERROR("invalid rknpu core mask: %#x", args->core_mask);
		return -EINVAL;
	}

	job = rknpu_job_alloc(rknpu_dev, args);
	if (!job) {
		LOG_ERROR("failed to allocate rknpu job!\n");
		return -ENOMEM;
	}

	if (args->flags & RKNPU_JOB_FENCE_IN) {
#ifdef CONFIG_ROCKCHIP_RKNPU_FENCE
		struct dma_fence *in_fence;

		in_fence = sync_file_get_fence(args->fence_fd);

		if (!in_fence) {
			LOG_ERROR("invalid fence in fd, fd: %d\n",
				  args->fence_fd);
			return -EINVAL;
		}
		args->fence_fd = -1;

		/*
		 * Wait if the fence is from a foreign context, or if the fence
		 * array contains any fence from a foreign context.
		 */
		ret = 0;
		if (!dma_fence_match_context(in_fence,
					     rknpu_dev->fence_ctx->context))
			ret = dma_fence_wait_timeout(in_fence, true,
						     args->timeout);
		dma_fence_put(in_fence);
		if (ret < 0) {
			if (ret != -ERESTARTSYS)
				LOG_ERROR("Error (%d) waiting for fence!\n",
					  ret);

			return ret;
		}
#else
		LOG_ERROR(
			"failed to use rknpu fence, please enable rknpu fence config!\n");
		rknpu_job_free(job);
		return -EINVAL;
#endif
	}

	if (args->flags & RKNPU_JOB_FENCE_OUT) {
#ifdef CONFIG_ROCKCHIP_RKNPU_FENCE
		ret = rknpu_fence_alloc(job);
		if (ret) {
			rknpu_job_free(job);
			return ret;
		}
		job->args->fence_fd = rknpu_fence_get_fd(job);
		args->fence_fd = job->args->fence_fd;
#else
		LOG_ERROR(
			"failed to use rknpu fence, please enable rknpu fence config!\n");
		rknpu_job_free(job);
		return -EINVAL;
#endif
	}

	/* #patch39: arm the progress watchdog for EVERY submit, not just NONBLOCK ones.
	 *
	 * The arm used to live inside the NONBLOCK branch below, so a BLOCKING submit was never
	 * observed by the watchdog at all. That is not a corner case: ork_dyn_colsplit() issues
	 * blocking submits by default ("NO barrier + BLOCKING submit -- EXACTLY the mcworker",
	 * src/npu/core/colsplit.c:48) and only goes NONBLOCK under ORK_F16_SENTINEL. Its
	 * ork_dyn_ prefix makes it look like the doorbell path; it is not.
	 *
	 * Measured consequence: a 1.5B-Q8 run that stalled 8 times with wd_period_us=1000 recorded
	 * wd_ticks=0 -- the hrtimer never started once, so the instrumentation built to catch these
	 * stalls could not see the very path that stalls. A stalled blocking submit then sits in
	 * rknpu_job_wait() for its full timeout (observed: 60.6 s) with nothing sampling it.
	 *
	 * A blocking job is in flight exactly like an async one, so there is no reason to treat it
	 * differently; the handler still self-disarms once no core has a job. */
	rknpu_wd_arm(rknpu_dev);

	if (args->flags & RKNPU_JOB_NONBLOCK) {
		job->flags |= RKNPU_JOB_ASYNC;
		/*
		 * #patch73: the RKNPU_IOCTL() wrapper scopes a power reference to the ioctl. For a
		 * blocking submit that is enough, because the ioctl does not return until the job is
		 * done -- but this one returns as soon as the job is committed, dropping the reference
		 * while the hardware is still running. The deferred power-off then fires under a live
		 * job and the completion interrupt lands on an unpowered block, where rknpu_irq_handler
		 * must bail (touching MMIO unpowered killed the machine outright -- see the netconsole
		 * panic note there). The completion is then never accounted: interrupt_count is only
		 * decremented in rknpu_job_done(), reached only from that handler, so the job owns its
		 * core forever and every later submit queues behind it and never commits.
		 * MEASURED: cnt_unpow = 3441 dropped interrupts, and a stuck owner logged with
		 * "flags 0x2, int_cnt 1, run_cnt 0" at 60s of age.
		 *
		 * rknpu_power_get() increments the refcount unconditionally and only reports a failure
		 * from rknpu_power_on(), which cannot run here because the ioctl reference already has
		 * the block powered. So set the bit either way -- the count was taken.
		 * Released in rknpu_job_free(): the one point every job passes through exactly once
		 * (including rknpu_job_timeout_clean(), which reaches cleanup through neither job_done
		 * nor job_abort), and the right context -- job_done() runs in hard IRQ while
		 * rknpu_power_put_delay() takes power_lock, a mutex.
		 */
		rknpu_power_get(rknpu_dev);
		set_bit(0, &job->pwr_held);
		rknpu_job_timeout_clean(rknpu_dev, job->args->core_mask);
		rknpu_job_schedule(job);
		ret = job->ret;
		if (ret) {
			rknpu_job_abort(job);
			return ret;
		}
	} else {
		rknpu_job_schedule(job);
		if (args->flags & RKNPU_JOB_PC)
			job->ret = rknpu_job_wait(job);

		args->task_counter = job->args->task_counter;
		ret = job->ret;
		if (!(args->flags & RKNPU_JOB_PC))
			rknpu_dbg_nopc++;   /* #patchB7: blocking submit that never waited */
		if (!ret && !(job->flags & RKNPU_JOB_DONE)) {   /* #patchB7 */
			rknpu_dbg_false_ok++;
			LOG_ERROR("RKNPU: SUBMIT returned OK but job NOT DONE: job_flags=%#x args_flags=%#x pc=%d tn=%u core=%#x counter=%u (n=%lu)\n",
				  job->flags, args->flags,
				  !!(args->flags & RKNPU_JOB_PC), args->task_number,
				  args->core_mask, args->task_counter, rknpu_dbg_false_ok);
		}
		if (!ret)
			rknpu_job_cleanup(job);
		else
			rknpu_job_abort(job);
	}

	return ret;
}

#ifdef CONFIG_ROCKCHIP_RKNPU_DRM_GEM
int rknpu_submit_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	struct rknpu_device *rknpu_dev = dev_get_drvdata(dev->dev);
	struct rknpu_submit *args = data;

	return rknpu_submit(rknpu_dev, args);
}
#endif

#ifdef CONFIG_ROCKCHIP_RKNPU_DMA_HEAP
int rknpu_submit_ioctl(struct rknpu_device *rknpu_dev, unsigned long data)
{
	struct rknpu_submit args;
	int ret = -EINVAL;

	if (unlikely(copy_from_user(&args, (struct rknpu_submit *)data,
				    sizeof(struct rknpu_submit)))) {
		LOG_ERROR("%s: copy_from_user failed\n", __func__);
		ret = -EFAULT;
		return ret;
	}

	ret = rknpu_submit(rknpu_dev, &args);

	if (unlikely(copy_to_user((struct rknpu_submit *)data, &args,
				  sizeof(struct rknpu_submit)))) {
		LOG_ERROR("%s: copy_to_user failed\n", __func__);
		ret = -EFAULT;
		return ret;
	}

	return ret;
}
#endif

int rknpu_get_hw_version(struct rknpu_device *rknpu_dev, uint32_t *version)
{
	void __iomem *rknpu_core_base = rknpu_dev->base[0];

	if (version == NULL)
		return -EINVAL;

	*version = REG_READ(RKNPU_OFFSET_VERSION) +
		   (REG_READ(RKNPU_OFFSET_VERSION_NUM) & 0xffff);

	return 0;
}

int rknpu_get_bw_priority(struct rknpu_device *rknpu_dev, uint32_t *priority,
			  uint32_t *expect, uint32_t *tw)
{
	void __iomem *base = rknpu_dev->bw_priority_base;

	if (!base)
		return -EINVAL;

	spin_lock(&rknpu_dev->lock);

	if (priority != NULL)
		*priority = _REG_READ(base, 0x0);

	if (expect != NULL)
		*expect = _REG_READ(base, 0x8);

	if (tw != NULL)
		*tw = _REG_READ(base, 0xc);

	spin_unlock(&rknpu_dev->lock);

	return 0;
}

int rknpu_set_bw_priority(struct rknpu_device *rknpu_dev, uint32_t priority,
			  uint32_t expect, uint32_t tw)
{
	void __iomem *base = rknpu_dev->bw_priority_base;

	if (!base)
		return -EINVAL;

	spin_lock(&rknpu_dev->lock);

	if (priority != 0)
		_REG_WRITE(base, priority, 0x0);

	if (expect != 0)
		_REG_WRITE(base, expect, 0x8);

	if (tw != 0)
		_REG_WRITE(base, tw, 0xc);

	spin_unlock(&rknpu_dev->lock);

	return 0;
}

int rknpu_clear_rw_amount(struct rknpu_device *rknpu_dev)
{
	void __iomem *rknpu_core_base = rknpu_dev->base[0];
	const struct rknpu_config *config = rknpu_dev->config;
	unsigned long flags;

	if (config->amount_top == NULL) {
		LOG_WARN("Clear rw_amount is not supported on this device!\n");
		return 0;
	}

	if (config->pc_dma_ctrl) {
		uint32_t pc_data_addr = 0;

		spin_lock_irqsave(&rknpu_dev->irq_lock, flags);
		pc_data_addr = REG_READ(RKNPU_OFFSET_PC_DATA_ADDR);

		REG_WRITE(0x1, RKNPU_OFFSET_PC_DATA_ADDR);
		REG_WRITE(0x80000101, config->amount_top->offset_clr_all);
		REG_WRITE(0x00000101, config->amount_top->offset_clr_all);
		if (config->amount_core) {
			REG_WRITE(0x80000101,
				  config->amount_core->offset_clr_all);
			REG_WRITE(0x00000101,
				  config->amount_core->offset_clr_all);
		}
		REG_WRITE(pc_data_addr, RKNPU_OFFSET_PC_DATA_ADDR);
		spin_unlock_irqrestore(&rknpu_dev->irq_lock, flags);
	} else {
		spin_lock(&rknpu_dev->lock);
		REG_WRITE(0x80000101, config->amount_top->offset_clr_all);
		REG_WRITE(0x00000101, config->amount_top->offset_clr_all);
		if (config->amount_core) {
			REG_WRITE(0x80000101,
				  config->amount_core->offset_clr_all);
			REG_WRITE(0x00000101,
				  config->amount_core->offset_clr_all);
		}
		spin_unlock(&rknpu_dev->lock);
	}

	return 0;
}

int rknpu_get_rw_amount(struct rknpu_device *rknpu_dev, uint32_t *dt_wr,
			uint32_t *dt_rd, uint32_t *wd_rd)
{
	void __iomem *rknpu_core_base = rknpu_dev->base[0];
	const struct rknpu_config *config = rknpu_dev->config;
	int amount_scale = config->pc_data_amount_scale;

	if (config->amount_top == NULL) {
		LOG_WARN("Get rw_amount is not supported on this device!\n");
		return 0;
	}

	spin_lock(&rknpu_dev->lock);

	if (dt_wr != NULL) {
		*dt_wr = REG_READ(config->amount_top->offset_dt_wr) *
			 amount_scale;
		if (config->amount_core) {
			*dt_wr += REG_READ(config->amount_core->offset_dt_wr) *
				  amount_scale;
		}
	}

	if (dt_rd != NULL) {
		*dt_rd = REG_READ(config->amount_top->offset_dt_rd) *
			 amount_scale;
		if (config->amount_core) {
			*dt_rd += REG_READ(config->amount_core->offset_dt_rd) *
				  amount_scale;
		}
	}

	if (wd_rd != NULL) {
		*wd_rd = REG_READ(config->amount_top->offset_wt_rd) *
			 amount_scale;
		if (config->amount_core) {
			*wd_rd += REG_READ(config->amount_core->offset_wt_rd) *
				  amount_scale;
		}
	}

	spin_unlock(&rknpu_dev->lock);

	return 0;
}

int rknpu_get_total_rw_amount(struct rknpu_device *rknpu_dev, uint32_t *amount)
{
	const struct rknpu_config *config = rknpu_dev->config;
	uint32_t dt_wr = 0;
	uint32_t dt_rd = 0;
	uint32_t wd_rd = 0;
	int ret = -EINVAL;

	if (config->amount_top == NULL) {
		LOG_WARN(
			"Get total_rw_amount is not supported on this device!\n");
		return 0;
	}

	ret = rknpu_get_rw_amount(rknpu_dev, &dt_wr, &dt_rd, &wd_rd);

	if (amount != NULL)
		*amount = dt_wr + dt_rd + wd_rd;

	return ret;
}
