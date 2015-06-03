/*
 * drivers/net/gianfar_1588.c
 *
 * Copyright (C) 2008, 2009 Freescale Semiconductor, Inc.
 * Copyright (C) 2009 IXXAT Automation, GmbH
 *
 * Author: Anup Gangwar <anup.gangwar@freescale.com>
 *	   Yashpal Dutta <yashpal.dutta@freescale.com>
 *
 * Gianfar Ethernet Driver -- IEEE 1588 interface functionality
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#include <linux/vmalloc.h>
#include "gianfar.h"

static int gfar_ptp_init_circ(struct gfar_ptp_circular_t *buf);
static int gfar_ptp_is_empty(struct gfar_ptp_circular_t *buf);
static int gfar_ptp_nelems(struct gfar_ptp_circular_t *buf);
static int gfar_ptp_is_full(struct gfar_ptp_circular_t *buf);
static int gfar_ptp_insert(struct gfar_ptp_circular_t *buf,
			struct gfar_ptp_data_t *data);
static int gfar_ptp_find_and_remove(struct gfar_ptp_circular_t *buf,
			int key, struct gfar_ptp_data_t *data);

static DECLARE_WAIT_QUEUE_HEAD(ptp_rx_ts_wait);
#define PTP_GET_RX_TIMEOUT	(HZ/10)

static u32 freq_compensation;

/*
 * Resource required for accessing 1588 Timer Registers. There are few 1588
 * modules registers which are present in eTSEC1 memory space only. The second
 * reg entry there in denotes the 1588 regs.
 */
int gfar_ptp_init(struct gfar_private *priv)
{
	priv->ptimer = ioremap(priv->timer_resource.start,
		sizeof(struct gfar_1588));
	if ((priv->ptimer == NULL) ||
			gfar_ptp_init_circ(&(priv->rx_time_sync)) ||
			gfar_ptp_init_circ(&(priv->rx_time_del_req)) ||
			gfar_ptp_init_circ(&(priv->rx_time_pdel_req)) ||
			gfar_ptp_init_circ(&(priv->rx_time_pdel_resp)))
		return 1;
	return 0;
}

void gfar_ptp_cleanup(struct gfar_private *priv)
{
	if (priv->ptimer != NULL)
		iounmap(priv->ptimer);
	vfree(priv->rx_time_sync.data_buf);
	vfree(priv->rx_time_del_req.data_buf);
	if (priv->rx_time_pdel_req.data_buf)
		vfree(priv->rx_time_pdel_req.data_buf);
	if (priv->rx_time_pdel_resp.data_buf)
		vfree(priv->rx_time_pdel_resp.data_buf);

}

inline int gfar_ptp_do_txstamp(struct sk_buff *skb)
{
	u16  *udp_port;
	char *pkt_type;

	if (skb->len > 44) {
		pkt_type = (char *)(skb->data + GFAR_PTP_PKT_TYPE_OFFS);
		udp_port = (u16 *)(skb->data + GFAR_PTP_PORT_OFFS);

		/* Check if port is 319 for PTP Event, and check for UDP */
		if ((*udp_port == 0x013F) &&
				(*pkt_type == GFAR_PACKET_TYPE_UDP))
			return 1;
	}

	return 0;
}

void gfar_ptp_store_rxstamp(struct net_device *dev, struct sk_buff *skb)
{
	int msg_type, seq_id, control;
	struct gfar_ptp_data_t tmp_rx_time;
	struct gfar_private *priv = netdev_priv(dev);
	u16 udp_port;
	char pkt_type;

	pkt_type = *(((char *)skb->data) + GFAR_PTP_PKT_TYPE_OFFS);
	udp_port = *((u16 *)(skb->data + GFAR_PTP_PORT_OFFS));
	seq_id = *((u16 *)(skb->data + GFAR_PTP_SEQ_ID_OFFS));
	control = *((u8 *)(skb->data + GFAR_PTP_CTRL_OFFS));

	/* Check if port is 319 for PTP Event, and check for UDP */
	if ((udp_port == 0x13F) && (pkt_type == GFAR_PACKET_TYPE_UDP)) {
		tmp_rx_time.key = seq_id;
		tmp_rx_time.item.high = *((u32 *)skb->data);
		tmp_rx_time.item.low = *(((u32 *)skb->data) + 1);

		switch (control) {
		case GFAR_PTP_CTRL_SYNC:
			gfar_ptp_insert(&(priv->rx_time_sync), &tmp_rx_time);
			break;
		case GFAR_PTP_CTRL_DEL_REQ:
			gfar_ptp_insert(&(priv->rx_time_del_req), &tmp_rx_time);
			break;
		case GFAR_PTP_CTRL_ALL_OTHER:
			/* clear transportSpecific field*/
			msg_type = (*((u8 *)(skb->data +
					GFAR_PTP_MSG_TYPE_OFFS))) & 0x0F;
			switch (msg_type) {
			case GFAR_PTP_MSG_TYPE_PDREQ:
				gfar_ptp_insert(&(priv->rx_time_pdel_req),
					&tmp_rx_time);
				break;
			case GFAR_PTP_MSG_TYPE_PDRESP:
				gfar_ptp_insert(&(priv->rx_time_pdel_resp),
					&tmp_rx_time);
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
		wake_up_interruptible(&ptp_rx_ts_wait);
	}
}

int gfar_ptp_init_circ(struct gfar_ptp_circular_t *buf)
{
	buf->data_buf =	kmalloc((DEFAULT_PTP_RX_BUF_SZ + 1) *
				sizeof(struct gfar_ptp_data_t), GFP_KERNEL);
	if (!buf->data_buf)
		return -ENOMEM;

	buf->front = 0;
	buf->end = 0;
	buf->size = (DEFAULT_PTP_RX_BUF_SZ + 1);

	return 0;
}

static int gfar_ptp_calc_index(int size, int curr_index, int offset)
{
	return  (curr_index + offset) % size;
}

int gfar_ptp_is_empty(struct gfar_ptp_circular_t *buf)
{
	return buf->front == buf->end;
}

int gfar_ptp_nelems(struct gfar_ptp_circular_t *buf)
{
	const int front = buf->front;
	const int end = buf->end;
	const int size = buf->size;
	int n_items;

	if (end > front)
		n_items = end - front;
	else if (end < front)
		n_items = size - (front - end);
	else
		n_items = 0;

	return n_items;
}

int gfar_ptp_is_full(struct gfar_ptp_circular_t *buf)
{
	return gfar_ptp_nelems(buf) == (buf->size - 1);
}

int gfar_ptp_insert(struct gfar_ptp_circular_t *buf,
				struct gfar_ptp_data_t *data)
{
	struct gfar_ptp_data_t *tmp;

	if (gfar_ptp_is_full(buf))
		return 1;

	tmp = (buf->data_buf + buf->end);

	tmp->key = data->key;
	tmp->item.high = data->item.high;
	tmp->item.low = data->item.low;

	buf->end = gfar_ptp_calc_index(buf->size, buf->end, 1);

	return 0;
}

int gfar_ptp_find_and_remove(struct gfar_ptp_circular_t *buf,
			int key, struct gfar_ptp_data_t *data)
{
	int i;
	int size = buf->size, end = buf->end;

	if (gfar_ptp_is_empty(buf))
		return 1;

	i = buf->front;
	while (i != end) {
		if ((buf->data_buf + i)->key == key)
			break;
		i = gfar_ptp_calc_index(size, i, 1);
	}

	if (i == end) {
		buf->front = buf->end;
		return 1;
	}

	data->item.high = (buf->data_buf + i)->item.high;
	data->item.low = (buf->data_buf + i)->item.low;

	buf->front = gfar_ptp_calc_index(size, i, 1);

	return 0;
}

/* Set the 1588 timer counter registers */
static void gfar_set_1588cnt(struct net_device *dev,
			struct gfar_ptp_time *gfar_time)
{
	struct gfar_private *priv = netdev_priv(dev);
	u64 alarm_value = 0, temp_alarm_val = 0;

	temp_alarm_val = (u64)(gfar_time->low) + ((u64)gfar_time->high << 32);
	alarm_value = (u64)(div64_u64(temp_alarm_val, TMR_SEC)) * (u64)TMR_SEC;
	alarm_value += (u64)TMR_ALARM1_L + (((u64)TMR_ALARM1_H) << 32) -
		(TMR_CTRL_TCLK_PRD >> 16);

	/* We must write the tmr_cnt_l register first */
	gfar_write(&priv->ptimer->tmr_alarm1_l, (u32)alarm_value);
	if (gfar_read(&priv->regs->tsec_id) == TSEC_REV_15) {
		gfar_write(&priv->ptimer->tmr_fiper1, (TMR_FIPER1 -
			(2 * (TMR_CTRL_TCLK_PRD >> 16))));
	} else {
		gfar_write(&priv->ptimer->tmr_fiper1, (TMR_FIPER1 -
			(TMR_CTRL_TCLK_PRD >> 16)));
	}
	gfar_write(&priv->ptimer->tmr_cnt_l, (u32)gfar_time->low);
	gfar_write(&priv->ptimer->tmr_cnt_h, (u32)gfar_time->high);
	gfar_write(&priv->ptimer->tmr_alarm1_h, (u32)(alarm_value>>32));
}

/* Get both the time-stamps and use the larger one */
static void gfar_get_tx_timestamp(struct gfar __iomem *regs,
			struct gfar_ptp_time *tx_time)
{
	struct gfar_ptp_time tx_set_1, tx_set_2;
	u32 tmp;

	/* Read the low register first */
	tx_set_1.low = gfar_read(&regs->tmr_txts1_l);
	tx_set_1.high = gfar_read(&regs->tmr_txts1_h);

	tx_set_2.low = gfar_read(&regs->tmr_txts2_l);
	tx_set_2.high = gfar_read(&regs->tmr_txts2_h);

	tmp = 0;

	if (tx_set_2.high > tx_set_1.high)
		tmp = 1;
	else if (tx_set_2.high == tx_set_1.high)
		if (tx_set_2.low > tx_set_1.low)
			tmp = 1;

	if (tmp == 0) {
		tx_time->low = tx_set_1.low;
		tx_time->high = tx_set_1.high;
	} else {
		tx_time->low = tx_set_2.low;
		tx_time->high = tx_set_2.high;
	}
}

static int gfar_get_rx_time(struct gfar_private *priv, struct ifreq *ifr,
		struct gfar_ptp_time *rx_time, int mode)
{
	struct gfar_ptp_data_t tmp;
	int key, flag;

	if (get_user(key, (int __user *)ifr->ifr_data))
		return -EFAULT;

	switch (mode) {
	case PTP_GET_RX_TIMESTAMP_SYNC:
		flag = gfar_ptp_find_and_remove(&(priv->rx_time_sync),
						key, &tmp);
		break;
	case PTP_GET_RX_TIMESTAMP_DEL_REQ:
		flag = gfar_ptp_find_and_remove(&(priv->rx_time_del_req),
						key, &tmp);
		break;
	case PTP_GET_RX_TIMESTAMP_PDELAY_REQ:
		flag = gfar_ptp_find_and_remove(&(priv->rx_time_pdel_req),
						key, &tmp);
		break;
	case PTP_GET_RX_TIMESTAMP_PDELAY_RESP:
		flag = gfar_ptp_find_and_remove(&(priv->rx_time_pdel_resp),
						key, &tmp);
		break;
	default:
		flag = 1;
		break;
	}

	if (!flag) {
		rx_time->high = tmp.item.high;
		rx_time->low = tmp.item.low;
		return 0;
	}

	wait_event_interruptible_timeout(ptp_rx_ts_wait, 0,
			PTP_GET_RX_TIMEOUT);

	switch (mode) {
	case PTP_GET_RX_TIMESTAMP_SYNC:
		flag = gfar_ptp_find_and_remove(&(priv->rx_time_sync),
						key, &tmp);
		break;
	case PTP_GET_RX_TIMESTAMP_DEL_REQ:
		flag = gfar_ptp_find_and_remove(&(priv->rx_time_del_req),
						key, &tmp);
		break;
	case PTP_GET_RX_TIMESTAMP_PDELAY_REQ:
		flag = gfar_ptp_find_and_remove(&(priv->rx_time_pdel_req),
						key, &tmp);
		break;
	case PTP_GET_RX_TIMESTAMP_PDELAY_RESP:
		flag = gfar_ptp_find_and_remove(&(priv->rx_time_pdel_resp),
						key, &tmp);
		break;
	}

	if (flag)
		return -ENOIOCTLCMD;

	rx_time->high = tmp.item.high;
	rx_time->low = tmp.item.low;
	return 0;
}

static void gfar_get_curr_cnt(struct gfar_1588 __iomem *ptimer,
			struct gfar_ptp_time *curr_time)
{
	curr_time->low = gfar_read(&ptimer->tmr_cnt_l);
	curr_time->high = gfar_read(&ptimer->tmr_cnt_h);
}

/* IOCTL Handler for PTP Specific IOCTL Commands coming from PTPD Application */
int gfar_ioctl_1588(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->regs;
	struct gfar_ptp_time cnt;
	int addend;
	struct gfar_ptp_time rx_time, tx_time, curr_time;
	int retval = 0;

	switch (cmd) {
	case PTP_GET_RX_TIMESTAMP_SYNC:
	case PTP_GET_RX_TIMESTAMP_DEL_REQ:
	case PTP_GET_RX_TIMESTAMP_PDELAY_REQ:
	case PTP_GET_RX_TIMESTAMP_PDELAY_RESP:
		retval = gfar_get_rx_time(priv, ifr, &rx_time, cmd);
		if (retval)
			return retval;

		if (copy_to_user(ifr->ifr_data, &rx_time, sizeof(rx_time)))
			retval = -EFAULT;
		break;
	case PTP_GET_TX_TIMESTAMP:
		gfar_get_tx_timestamp(regs, &tx_time);
		if (copy_to_user(ifr->ifr_data, &tx_time, sizeof(tx_time)))
			retval = -EFAULT;
		break;
	case PTP_GET_CNT:
		gfar_get_curr_cnt(priv->ptimer, &curr_time);
		if (copy_to_user(ifr->ifr_data, &curr_time, sizeof(curr_time)))
			retval = -EFAULT;
		break;
	case PTP_SET_CNT:
		if (copy_from_user(&cnt, ifr->ifr_data,
				sizeof(struct gfar_ptp_time)))
			return -EFAULT;
		gfar_set_1588cnt(dev, &cnt);
		break;
	case PTP_ADJ_ADDEND:
		if (get_user(addend, (int __user *)ifr->ifr_data))
			return -EFAULT;
		/* assign new value directly */
		gfar_write(&priv->ptimer->tmr_add, addend);
		break;
	case PTP_GET_ADDEND:
		/* return initial timer add value
		 * to calculate drift correction */
		if (copy_to_user(ifr->ifr_data, &freq_compensation,
				sizeof(freq_compensation)))
			retval = -EFAULT;
		break;
	case PTP_CLEANUP_TIMESTAMP_BUFFERS:
		/* reset sync buffer */
		priv->rx_time_sync.front = 0;
		priv->rx_time_sync.end = 0;
		priv->rx_time_sync.size = (DEFAULT_PTP_RX_BUF_SZ + 1);
		/* reset delay_req buffer */
		priv->rx_time_del_req.front = 0;
		priv->rx_time_del_req.end = 0;
		priv->rx_time_del_req.size = (DEFAULT_PTP_RX_BUF_SZ + 1);
		/* reset pdelay_req buffer */
		priv->rx_time_pdel_req.front = 0;
		priv->rx_time_pdel_req.end = 0;
		priv->rx_time_pdel_req.size = (DEFAULT_PTP_RX_BUF_SZ + 1);
		/* reset pdelay_resp buffer */
		priv->rx_time_pdel_resp.front = 0;
		priv->rx_time_pdel_resp.end = 0;
		priv->rx_time_pdel_resp.size = (DEFAULT_PTP_RX_BUF_SZ + 1);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return retval;
}

/* Function to initialize Filer Entry
 * far: Offset in Filer Table in SRAM
 * fcr: Filer Control register value corresponds to table entry
 * fpr: Filer Entry Value
 */
inline void gfar_write_filer(struct net_device *dev, unsigned int far,
			unsigned int fcr, unsigned int fpr)
{
	struct gfar_private *priv = netdev_priv(dev);
	gfar_write(&priv->regs->rqfar, far);
	gfar_write(&priv->regs->rqfcr, fcr);
	gfar_write(&priv->regs->rqfpr, fpr);
}

/* 1588 Module intialization and filer table populating routine*/
void gfar_1588_start(struct net_device *dev)
{
	u32 freq = 0;
	struct gfar_private *priv = netdev_priv(dev);

	gfar_write(&priv->ptimer->tmr_prsc, TMR_PRSC);
	if (gfar_read(&priv->regs->tsec_id) == TSEC_REV_15) {
		gfar_write(&priv->ptimer->tmr_fiper1, (TMR_FIPER1 -
			(2 * (TMR_CTRL_TCLK_PRD >> 16))));
	} else {
		gfar_write(&priv->ptimer->tmr_fiper1, (TMR_FIPER1 -
			(TMR_CTRL_TCLK_PRD >> 16)));
	}
	gfar_write(&priv->ptimer->tmr_alarm1_l, TMR_ALARM1_L -
		(TMR_CTRL_TCLK_PRD >> 16));
	gfar_read(&priv->ptimer->tmr_alarm1_l); /* Delay of one cycle */
	gfar_write(&priv->ptimer->tmr_alarm1_h, TMR_ALARM1_H);

	/* Need to mask the TCLK bits as they are initialized with 1 */
	gfar_write(&priv->ptimer->tmr_ctrl,
		(gfar_read(&priv->ptimer->tmr_ctrl)
		& ~TMR_CTRL_TCLK_MASK) | TMR_CTRL_TCLK_PRD |
		TMR_CTRL_FIPER_START);

	freq = TMR_SEC / (TMR_CTRL_TCLK_PRD>>16);
	/* initialize TMR_ADD with the initial frequency compensation value:
	 * freq_compensation = (2^32 / frequency ratio)
	 */
	freq_compensation = div64_u64(((u64)freq << 32), TMR_OSC_FREQ);
	gfar_write(&priv->ptimer->tmr_add, freq_compensation);

	/* Select 1588 Timer source and enable module for starting Tmr Clock */
	gfar_write(&priv->ptimer->tmr_ctrl,
		gfar_read(&priv->ptimer->tmr_ctrl) |
		TMR_CTRL_ENABLE | TMR_CTRL_SYS_CLK | TMR_CTRL_FIPER_START);
}

/* Cleanup routine for 1588 module.
 * When PTP is disabled this routing is called */
void gfar_1588_stop(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);

	gfar_write(&priv->ptimer->tmr_ctrl,
		gfar_read(&priv->ptimer->tmr_ctrl)
		& ~TMR_CTRL_ENABLE);
}

