/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Felix Zeng <felix.zeng@rock-chips.com>
 */

#ifndef __LINUX_RKNPU_IOMMU_H
#define __LINUX_RKNPU_IOMMU_H

#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/version.h>

#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
#include <linux/dma-iommu.h>
#endif

#include "rknpu_drv.h"

enum iommu_dma_cookie_type {
	IOMMU_DMA_IOVA_COOKIE,
	IOMMU_DMA_MSI_COOKIE,
};

struct rknpu_iommu_dma_cookie {
	enum iommu_dma_cookie_type type;

	/* Full allocator for IOMMU_DMA_IOVA_COOKIE */
	struct iova_domain iovad;
};

dma_addr_t rknpu_iommu_dma_alloc_iova(struct iommu_domain *domain, size_t size,
				      u64 dma_limit, struct device *dev,
				      bool size_aligned);

void rknpu_iommu_dma_free_iova(struct rknpu_iommu_dma_cookie *cookie,
			       dma_addr_t iova, size_t size, bool size_aligned);

int rknpu_iommu_dma_map_sg(struct device *dev, struct scatterlist *sg,
			   int nents, enum dma_data_direction dir,
			   bool iova_aligned);

void rknpu_iommu_dma_unmap_sg(struct device *dev, struct scatterlist *sg,
			      int nents, enum dma_data_direction dir,
			      bool iova_aligned);

int rknpu_iommu_init_domain(struct rknpu_device *rknpu_dev);
int rknpu_iommu_switch_domain(struct rknpu_device *rknpu_dev, int domain_id);
void rknpu_iommu_free_domains(struct rknpu_device *rknpu_dev);
int rknpu_iommu_domain_get_and_switch(struct rknpu_device *rknpu_dev,
				      int domain_id);
int rknpu_iommu_domain_put(struct rknpu_device *rknpu_dev);

/* #patchB9: THE LIVE DOMAIN. Under the light switch (#patchB1/B2) the IOMMU core is not involved in
 * a domain change, so iommu_get_domain_for_dev() reports the group's DEFAULT domain (domain 0)
 * forever. Every map/unmap/flush that must target the domain the NPU is actually running in has to
 * use this instead. Using the core accessor silently targets domain 0: the mapping lands in the
 * wrong page table and the next job is committed but never completes. */
struct iommu_domain *rknpu_iommu_live_domain(struct device *dev);

/* #patchB10: scoped override of the core's DEFAULT domain, for the dma-buf attachment map/unmap only
 * (the DMA API cannot be told which domain to use). Returns the previous value; restore it. */
struct iommu_domain *rknpu_iommu_default_swap(struct device *dev, struct iommu_domain *dom);

#if KERNEL_VERSION(5, 10, 0) < LINUX_VERSION_CODE
int iommu_get_dma_cookie(struct iommu_domain *domain);
#endif

#endif

int rknpu_iommu_reinit(struct rknpu_device *rknpu_dev);   /* #patch61 */
