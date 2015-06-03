/*
 * Freescale MPC83xx DMA XOR Engine support
 *
 * Copyright (C) 2008-2009 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author:
 *   Dipen Dudhat <Dipen.Dudhat@freescale.com>, Oct 2008
 *   Maneesh Gupta <Maneesh.Gupta@freescale.com>, Oct 2008
 *   Surender Kumar <R66464@freescale.com>, Oct 2008
 *   Vishnu Suresh <Vishnu@freescale.com>, Oct  2008
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef DMAXOR_H
#define DMAXOR_H

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/cache.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/raid/xor.h>

#define TOTAL_PAIRS (7)

struct fsl_xor_desc {
	struct dma_async_tx_descriptor async_tx;
	struct list_head node;
	dma_addr_t dest;
	dma_addr_t src_list[TOTAL_PAIRS - 1];
	unsigned int count;
	unsigned int len;
	void (*release_hdl) (void *, int);
	void *release_arg;
};

struct fsl_xor_adma_device {
	struct device *dev;
	struct dma_device common;
	int max_xor_srcs;
};

struct fsl_xor_chan {
	dma_cookie_t completed_cookie;	/* The maximum cookie completed */
	spinlock_t desc_lock;	/* lock for tx queue */
	unsigned int total_desc;	/* Number of descriptors allocated */
	struct list_head pending_q;
	struct list_head in_process_q;
	struct list_head free_desc;
	struct fsl_xor_adma_device *device;
	struct dma_chan common;
	unsigned int status;	/* Is channel busy */
	int id;			/* Raw id of this channel */
};

#ifdef CONFIG_FSL_XOR_ADMA
int fsl_xor_adma_register(struct fsl_xor_adma_device **ppdev,
			  int max_xor_srcs, struct device *dev);
void fsl_xor_adma_unregister(struct fsl_xor_adma_device *pdev);
#else
#define fsl_xor_adma_register(ppdev, max_xor_srcs, dev) (-1)
#define fsl_xor_adma_unregister(pdev)	NULL
#endif	/* CONFIG_FSL_XOR_ADMA */

int talitos_request_xor(struct device *dev, dma_addr_t dest,
			dma_addr_t *src_list, int count, int len,
			void *context);
#endif /* DMAXOR_H */
