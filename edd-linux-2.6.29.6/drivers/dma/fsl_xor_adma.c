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
 * Description:
 *   DMA XOR engine driver for Freescale SEC Engine. The Talitos Driver
 *   is used to expose the SEC Engine as a DMA engine with XOR capability.
 *   The support for ZERO SUM is not implemented at present.
 *   This code is the frontend which exposes a DMA device with XOR Capability
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include "fsl_xor.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/of_platform.h>


#define INITIAL_FSL_DESC_COUNT (0x100)
#define XOR_MAX_PAGE_SIZE (0xFFFFu)
#define XOR_DEVICE_BUSY (0x01)
#define XOR_DEVICE_ERROR (0x02)

/**
 * fsl_xor_adma_cleanup - Cleanup descriptor from Tx descriptor queue.
 * @return: Maximum number of Sources that can be handled
 */
static int fsl_xor_adma_get_max_xor(struct fsl_xor_adma_device *pdev)
{
	return pdev->max_xor_srcs;
}

/**
 * fsl_xor_adma_cleanup - Cleanup descriptor from in_process queue
 * back to the free desciptor queue.
 * @xor_chan : Freescale XOR ADMA channel
 */
static void fsl_xor_adma_cleanup(struct fsl_xor_chan *xor_chan)
{
	struct fsl_xor_desc *desc, *_desc;
	dma_async_tx_callback callback;
	void *callback_param;

	spin_lock_bh(&xor_chan->desc_lock);
	list_for_each_entry_safe(desc, _desc, &xor_chan->in_process_q, node) {
		if (dma_async_is_complete(desc->async_tx.cookie,
					  xor_chan->completed_cookie,
					  xor_chan->common.cookie)
		    == DMA_IN_PROGRESS)
			break;

		callback = desc->async_tx.callback;
		callback_param = desc->async_tx.callback_param;

		if (callback) {
			spin_unlock_bh(&xor_chan->desc_lock);
			callback(callback_param);
			spin_lock_bh(&xor_chan->desc_lock);
		}
		list_del(&desc->node);
		list_add_tail(&desc->node, &xor_chan->free_desc);
	}
	spin_unlock_bh(&xor_chan->desc_lock);
}

/**
 * fsl_xor_adma_dependency_added - Cleanup any pending descriptor.
 * @chan : DMA channel
 */
static void fsl_xor_adma_dependency_added(struct dma_chan *chan)
{
	struct fsl_xor_chan *xor_chan = container_of(chan,
						     struct fsl_xor_chan,
						     common);

	fsl_xor_adma_cleanup(xor_chan);
}

/**
 * fsl_xor_adma_is_complete - Check for status and update the cookies.
 * @chan : DMA channel
 */
static enum dma_status fsl_xor_adma_is_complete(struct dma_chan *chan,
						dma_cookie_t cookie,
						dma_cookie_t *done,
						dma_cookie_t *used)
{
	struct fsl_xor_chan *xor_chan = container_of(chan,
						     struct fsl_xor_chan,
						     common);
	dma_cookie_t last_used;
	dma_cookie_t last_complete;

	last_used = chan->cookie;
	last_complete = xor_chan->completed_cookie;

	if (done)
		*done = last_complete;

	if (used)
		*used = last_used;

	return dma_async_is_complete(cookie, last_complete, last_used);
}

/**
 * fsl_xor_adma_update_cookie - Update the cookies
 * @xor_chan :  XOR DMA channel
 * @xor_desc :  The descriptor which was XORed
 */
static void fsl_xor_adma_update_cookie(struct fsl_xor_chan *xor_chan,
				       struct fsl_xor_desc *xor_desc)
{
	spin_lock_bh(&xor_chan->desc_lock);
	xor_chan->completed_cookie = xor_desc->async_tx.cookie;
	xor_chan->status &= ~XOR_DEVICE_BUSY;
	spin_unlock_bh(&xor_chan->desc_lock);
	fsl_xor_adma_cleanup(xor_chan);
}

/**
 * fsl_xor_adma_error_handler - Upon device error do Synchronous XOR
 * on the descriptors, and update the cookies.
 * @xor_chan :  XOR DMA channel
 * @desc :  The descriptor which was NOT XORed
 */
static void fsl_xor_adma_error_handler(struct fsl_xor_chan *xor_chan,
				       struct fsl_xor_desc *xor_desc)
{
	struct fsl_xor_desc *desc, *_desc;

	xor_blocks(xor_desc->count, xor_desc->len,
		   xor_desc->dest, xor_desc->src_list);
	fsl_xor_adma_update_cookie(xor_chan, xor_desc);

	if (!list_empty(&xor_chan->in_process_q))
		return;

	/*
	 * No more descriptors in process.
	 * Talitos is done flushing channels
	 * and ready to accept more.
	 * Reset error status
	 */
	spin_lock_bh(&xor_chan->desc_lock);
	xor_chan->status &= ~XOR_DEVICE_ERROR;
	spin_unlock_bh(&xor_chan->desc_lock);
}

/**
 * fsl_xor_adma_release_handler - Handler to be called upon completion
 * @desc : Freescale XOR ADMA descriptor
 */
void fsl_xor_adma_release_handler(struct fsl_xor_desc *desc, int status)
{
	struct fsl_xor_chan *xor_chan = container_of(desc->async_tx.chan,
						     struct fsl_xor_chan,
						     common);

	if (status) {
		/* Talitos Encountered Error */
		spin_lock_bh(&xor_chan->desc_lock);
		xor_chan->status |= XOR_DEVICE_ERROR;
		spin_unlock_bh(&xor_chan->desc_lock);
		fsl_xor_adma_error_handler(xor_chan, desc);
		return;
	}

	fsl_xor_adma_update_cookie(xor_chan, desc);
	return;
}

/**
 * fsl_xor_adma_issue_pending - Request for XOR operation and move the
 * submitted requests to from pending queue to in_process queue
 * @chan : DMA channel
 */
static void fsl_xor_adma_issue_pending(struct dma_chan *chan)
{
	struct fsl_xor_chan *xor_chan = container_of(chan, struct fsl_xor_chan,
						     common);
	struct fsl_xor_desc *desc, *_desc;
	unsigned long flags;
	int status = 0;

	if (xor_chan->status)
		return;

	spin_lock_irqsave(&xor_chan->desc_lock, flags);
	list_for_each_entry_safe(desc, _desc, &xor_chan->pending_q, node) {

		status = talitos_request_xor(xor_chan->device->dev, desc->dest,
					     desc->src_list, desc->count,
					     desc->len, desc);

		if (!status) {
			list_del(&desc->node);
			list_add_tail(&desc->node, &xor_chan->in_process_q);
		} else {
			xor_chan->status |= XOR_DEVICE_BUSY;
			break;
		}
	}
	spin_unlock_irqrestore(&xor_chan->desc_lock, flags);
}

/**
 * fsl_xor_adma_tx_submit - Prepare the descriptors form submission, move
 * them to the pending queue.
 * @tx : DMA Async_tx Transaction descriptor
 */
static dma_cookie_t fsl_xor_adma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct fsl_xor_desc *desc = container_of(tx, struct fsl_xor_desc,
						 async_tx);
	struct fsl_xor_chan *xor_chan = container_of(tx->chan,
						     struct fsl_xor_chan,
						     common);
	dma_cookie_t cookie;

	spin_lock_bh(&xor_chan->desc_lock);

	cookie = xor_chan->common.cookie + 1;
	if (cookie < 0)
		cookie = 1;

	desc->async_tx.cookie = cookie;
	xor_chan->common.cookie = desc->async_tx.cookie;

	if (list_empty(&xor_chan->pending_q))
		list_splice(&desc->async_tx.tx_list, xor_chan->pending_q.prev);
	else
		list_splice_init(&desc->async_tx.tx_list,
				 xor_chan->pending_q.prev);

	spin_unlock_bh(&xor_chan->desc_lock);
	return cookie;
}

/**
 * fsl_xor_adma_alloc_descriptor - Allocate a Freescale XOR ADMA descriptor
 * @xor_chan :  Freescale XOR ADMA channel
 */
static struct fsl_xor_desc *fsl_xor_adma_alloc_descriptor(struct fsl_xor_chan
							  *xor_chan,
							  gfp_t flags)
{
	struct fsl_xor_desc *desc = NULL;

	desc = kzalloc(sizeof(*desc), flags);

	if (desc) {
		xor_chan->total_desc++;
		desc->async_tx.tx_submit = fsl_xor_adma_tx_submit;
		desc->release_hdl = (void (*)(void *, int))
		    fsl_xor_adma_release_handler;
		desc->release_arg = desc;
	}

	return desc;
}

/**
 * fsl_xor_adma_free_chan_resources - Free XOR ADMA resources, mainly the queues
 * @chan :  DMA channel
 */
static void fsl_xor_adma_free_chan_resources(struct dma_chan *chan)
{
	struct fsl_xor_chan *xor_chan = container_of(chan,
						     struct fsl_xor_chan,
						     common);
	struct fsl_xor_desc *desc, *_desc;

	spin_lock_bh(&xor_chan->desc_lock);

	list_for_each_entry_safe(desc, _desc, &xor_chan->pending_q, node) {
		list_del(&desc->node);
		xor_chan->total_desc--;
		kfree(desc);
	}

	list_for_each_entry_safe(desc, _desc, &xor_chan->in_process_q, node) {
		list_del(&desc->node);
		xor_chan->total_desc--;
		kfree(desc);
	}

	list_for_each_entry_safe(desc, _desc, &xor_chan->free_desc, node) {
		list_del(&desc->node);
		xor_chan->total_desc--;
		kfree(desc);
	}

	BUG_ON(unlikely(xor_chan->total_desc));	/* Some descriptor not freed? */
	spin_unlock_bh(&xor_chan->desc_lock);
}

/**
 * fsl_xor_adma_alloc_chan_resources - Allocate XOR ADMA resources
 * @chan :  DMA channel
 * @ret: No of allocated descriptors or -ENOMEM
 */
static int fsl_xor_adma_alloc_chan_resources(struct dma_chan *chan)
{
	struct fsl_xor_chan *xor_chan = container_of(chan,
						     struct fsl_xor_chan,
						     common);
	struct fsl_xor_desc *desc = NULL;
	int i = 0, status = 0;
	LIST_HEAD(tmp_list);

	if (!list_empty(&xor_chan->free_desc))
		return xor_chan->total_desc;

	for (i = 0; i < INITIAL_FSL_DESC_COUNT; i++) {
		desc = fsl_xor_adma_alloc_descriptor(xor_chan, GFP_KERNEL);
		if (!desc) {
			dev_err(xor_chan->common.device->dev,
				"Only %d initial descriptors\n", i);
			break;
		}
		list_add_tail(&desc->node, &tmp_list);
	}

	if (i) {
		/* At least one desc is allocated */
		spin_lock_bh(&xor_chan->desc_lock);
		list_splice_init(&tmp_list, &xor_chan->free_desc);
		spin_unlock_bh(&xor_chan->desc_lock);
		status = xor_chan->total_desc;
	} else {
		status = -ENOMEM;
	}

	return status;
}

/**
 * fsl_xor_adma_prep_xor - Prepare a Freescale XOR ADMA descriptor for operation
 * @chan :  DMA channel
 * @len :  length of source page
 */
static struct
dma_async_tx_descriptor *fsl_xor_adma_prep_xor(struct dma_chan *chan,
					       dma_addr_t dma_dest,
					       dma_addr_t *dma_src,
					       unsigned int src_cnt,
					       size_t len, unsigned long flags)
{
	struct fsl_xor_chan *xor_chan;
	struct fsl_xor_desc *first = NULL, *prev = NULL, *new;
	size_t copy;
	int i = 0;

	if ((!chan) || (!len))
		return NULL;

	xor_chan = container_of(chan, struct fsl_xor_chan, common);
	if (xor_chan->status && XOR_DEVICE_ERROR)
		return NULL;

	do {
		new = NULL;
		spin_lock_bh(&xor_chan->desc_lock);
		if (!list_empty(&xor_chan->free_desc)) {
			new = container_of(xor_chan->free_desc.next,
					   struct fsl_xor_desc, node);
			list_del(&new->node);
		} else {
			new = fsl_xor_adma_alloc_descriptor(xor_chan,
							    GFP_KERNEL);
		}
		spin_unlock_bh(&xor_chan->desc_lock);

		if (new) {
			dma_async_tx_descriptor_init(&new->async_tx,
						     &xor_chan->common);
			INIT_LIST_HEAD(&new->node);
			INIT_LIST_HEAD(&new->async_tx.tx_list);
		} else {
			dev_err(xor_chan->common.device->dev,
				"No free memory for XOR DMA descriptor\n");
			return NULL;
		}

		copy = min(len, XOR_MAX_PAGE_SIZE);
		new->len = copy;
		new->count = src_cnt;
		new->dest = dma_dest;
		dma_dest += copy;

		while (i < src_cnt) {
			new->src_list[i] = dma_src[i];
			dma_src[i] += copy;
			i++;
		}

		if (!first)
			first = new;

		new->async_tx.cookie = 0;
		new->async_tx.flags = DMA_CTRL_ACK;
		prev = new;
		len -= copy;
		list_add_tail(&new->node, &first->async_tx.tx_list);
	} while (len);

	new->async_tx.flags = DMA_PREP_INTERRUPT;
	new->async_tx.cookie = -EBUSY;

	return (first ? &first->async_tx : NULL);
}

/**
 * fsl_xor_adma_register - Initialize the Freescale XOR ADMA device
 *
 * The device is initialized along with the channel.
 * It is registered as a DMA device with the capability to perform
 * XOR operation. It is registerd with the Async_tx layer.
 * This is called from within the Talitos driver
 * The various queues and channel resources are also allocated.
 */
int fsl_xor_adma_register(struct fsl_xor_adma_device **ppdev,
			  int max_xor_srcs, struct device *dev)
{
	int error = 0;
	struct fsl_xor_adma_device *pdev = NULL;
	struct fsl_xor_chan *xor_chan = NULL;
	struct dma_chan *chan = NULL;

	pdev = kzalloc(sizeof(*pdev), GFP_KERNEL);
	if (!pdev) {
		dev_err(dev, "Not enough memory for XOR DMA\n");
		error = -ENOMEM;
		goto error;
	}

	pdev->dev = dev;
	pdev->max_xor_srcs = max_xor_srcs;
	pdev->common.device_alloc_chan_resources =
	    fsl_xor_adma_alloc_chan_resources;
	pdev->common.device_free_chan_resources =
	    fsl_xor_adma_free_chan_resources;
	pdev->common.device_prep_dma_xor = fsl_xor_adma_prep_xor;
	pdev->common.max_xor = fsl_xor_adma_get_max_xor(pdev);
	pdev->common.device_is_tx_complete = fsl_xor_adma_is_complete;
	pdev->common.device_issue_pending = fsl_xor_adma_issue_pending;
	pdev->common.device_terminate_all = fsl_xor_adma_dependency_added;
	pdev->common.dev = dev;
	*ppdev = pdev;

	xor_chan = kzalloc(sizeof(*xor_chan), GFP_KERNEL);

	if (!xor_chan) {
		dev_err(dev,
			"No free memory for allocating XOR DMA channels!\n");
		error = -ENOMEM;
		goto free_dev;
	}

	dma_cap_set(DMA_XOR, pdev->common.cap_mask);
	xor_chan->device = pdev;
	xor_chan->common.device = &pdev->common;
	INIT_LIST_HEAD(&pdev->common.channels);
	INIT_LIST_HEAD(&xor_chan->pending_q);
	INIT_LIST_HEAD(&xor_chan->in_process_q);
	INIT_LIST_HEAD(&xor_chan->free_desc);
	spin_lock_init(&xor_chan->desc_lock);
	xor_chan->total_desc = 0;
	list_add_tail(&xor_chan->common.device_node, &pdev->common.channels);
	pdev->common.chancnt++;

	error = dma_async_device_register(&pdev->common);
	if (error) {
		dev_err(dev, "Unable to register XOR with Async_tx\n");
		goto free_chan;
	}

	dev_info(dev, "Registered FSL_XOR_ADMA\n");
	return 0;

	dma_async_device_unregister(&pdev->common);
free_chan:
	list_for_each_entry(chan, &pdev->common.channels, device_node) {
		list_del(&chan->device_node);
		pdev->common.chancnt--;
	}
	kfree(xor_chan);
free_dev:
	kfree(pdev);
	*ppdev = NULL;
error:
	return error;
}

/**
 * fsl_xor_adma_unregister - Remove the Freescale XOR ADMA device
 */
void fsl_xor_adma_unregister(struct fsl_xor_adma_device *pdev)
{
	struct fsl_xor_chan *xor_chan = NULL;
	struct dma_chan *chan;

	dma_async_device_unregister(&pdev->common);
	list_for_each_entry(chan, &pdev->common.channels, device_node) {
		xor_chan = container_of(chan, struct fsl_xor_chan, common);
		list_del(&chan->device_node);
		pdev->common.chancnt--;
		kfree(xor_chan);
	}
	kfree(pdev);

	return;
}
