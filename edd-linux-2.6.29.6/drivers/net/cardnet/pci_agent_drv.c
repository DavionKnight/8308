/*
 * drivers/net/cardnet/pci_agent_drv.c
 *
 * Copyright (C) 2005-2009 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Xiaobo Xie <X.Xie@freescale.com>
 *         Jiang Yutang <b14898@freescale.com>
 *
 * Description:
 * PCIE Agent Ethernet Driver for Freescale PowerPC8315 Processor
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/in.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/in6.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <asm/checksum.h>
#include <asm/page.h>
#include <sysdev/fsl_soc.h>
#include "pci_agent_lib.h"
#include <asm/fsl_gtm.h>

static int eth;
module_param(eth, int, 0);

#define CARD_TIMEOUT 5
#define HAVE_TX_TIMEOUT
#ifdef HAVE_TX_TIMEOUT
static int timeout = CARD_TIMEOUT;
module_param(timeout, int, 0);
#endif

#define	PCI_SPACE_ADDRESS	0xC0000000

#define NEED_LOCAL_PAGE		0

#define	CTL_STATUS_SIZE	24
#define	RX_SPACE_SIZE	(2*1024-12)
#define	TX_SPACE_SIZE	(2*1024-12)
#define	AGENT_DRIVER_MEM_SIZE	(CTL_STATUS_SIZE+RX_SPACE_SIZE+TX_SPACE_SIZE)

#define PEX_CFG_BAR0 0x010
#define PEX_EPIWTAR0 0xDE0

struct cardnet_share_mem {
	u32	hstatus;
	u32	astatus;

	u32	rx_flags;
	u32	rx_packetlen;
	u8	rxbuf[2*1024 - 12];

	u32	tx_flags;
	u32	tx_packetlen;
	u8	txbuf[2*1024 - 12];
};

struct card_priv {
	struct cardnet_share_mem *share_mem;
	/*void __iomem *mesgu;*/

	struct sk_buff *cur_tx_skb;
	int rx_packetlen;
	int tx_packetlen;
	u32 ccsrbar;
	spinlock_t lock; /* lock for set card_priv */
	struct net_device_stats stats;
	struct pci_agent_dev *pcidev;
};

static void card_tx_timeout(struct net_device *dev);
static void card_rx(struct net_device *dev, int len, unsigned char *buf);

static void *mapped_immrbar;
static struct gtm_timer *tmr;

static void gtm_hook(struct net_device *dev, u16 rfr)
{
	struct card_priv *priv;
	struct cardnet_share_mem *shmem;
	u32  statusword = 0;
	int len;

	if (!dev)
		return;
	priv = (struct card_priv *) netdev_priv(dev);

	/* Lock the device */
	spin_lock(&priv->lock);
	shmem = (struct cardnet_share_mem *) priv->share_mem;

	statusword = (u32)rfr;
	if (statusword == HOST_SENT) {
		len = shmem->rx_packetlen;
		card_rx(dev, len, shmem->rxbuf);
		shmem->astatus = AGENT_GET;
	} else {
		printk(KERN_INFO "The message is not for me!message=0x%x\n",
					statusword);
		spin_unlock(&priv->lock);
		return;
	}

	/* Unlock the device and we are done */
	spin_unlock(&priv->lock);
	return;
}

static irqreturn_t card_isr(int irq, void *dev_id)
{
	u16 event, rfr;

	event = in_be16(tmr->gtevr);
	gtm_ack_timer16(tmr, event);

	if (event & GTEVR_REF) {
		rfr = in_be16(tmr->gtrfr);
		gtm_stop_timer16(tmr);
		gtm_hook((struct net_device *)dev_id, rfr);
	}
	return event ? IRQ_HANDLED : IRQ_NONE;
}

inline void pci_agent_cache_flush(void *addr)
{
	asm volatile("dcbf %0, %1" : : "r"(0), "r"((uint32_t)addr));
}

/*
 * Open and close
 */
int card_open(struct net_device *dev)
{
	struct card_priv *tp = netdev_priv(dev);
	int retval;
	u32 phys_addr;
	struct pci_agent_dev *pci_dev;
	struct device_node *np;
	const unsigned int *reg;

	for_each_compatible_node(np, NULL, "fsl,gtm") {
		reg = of_get_property(np, "reg", NULL);
		if((reg) && (*reg == 0x500)) {
			/* get GTM timer 4*/
			tmr = gtm_get_specific_timer16(np->data, 3);
			of_node_put(np);
			break;
		}
		of_node_put(np);
	}
	if (!tmr) {
		printk(KERN_INFO "Cannot get GTM timer 4\n");
		retval = -ENOMEM;
		goto err_out;
	}
	retval = request_irq(tmr->irq, card_isr, 0, "card GTM timer4", dev);
	if (retval) {
		printk(KERN_INFO "Cannot request irq\n");
		retval = -ENOMEM;
		goto err_irq;
	}

	tp->share_mem = (struct cardnet_share_mem *)
				__get_free_pages(GFP_ATOMIC, NEED_LOCAL_PAGE);
	if (tp->share_mem == NULL) {
		printk(KERN_INFO "Cannot get free pages\n");
		retval = -ENOMEM;
		goto err_nopage;
	}

	phys_addr = virt_to_phys((void *)tp->share_mem);

	memset(tp->share_mem, 0, PAGE_SIZE << NEED_LOCAL_PAGE);
	tp->pcidev = (struct pci_agent_dev *)
			kmalloc(sizeof(struct pci_agent_dev), GFP_KERNEL);
	if (tp->pcidev == NULL) {
		printk(KERN_INFO "Cannot allocate men\n");
		retval = -ENOMEM;
		goto err_nomem;
	}

	pci_dev = tp->pcidev;
	memset(pci_dev, 0, sizeof(struct pci_agent_dev));
	pci_dev->mem_addr = (u32)tp->share_mem;
	pci_dev->local_addr = phys_addr;
	pci_dev->pci_addr = PCI_SPACE_ADDRESS;
	pci_dev->mem_size = AGENT_DRIVER_MEM_SIZE;
	pci_dev->window_num = 1;

	/*Setup inbound first window Translation Address Registers in agent*/
	mapped_immrbar = ioremap(get_immrbase(), IMMRBAR_SIZE);
	if (mapped_immrbar == NULL) {
		printk(KERN_INFO "Cannot remap memory, aborting cardnet");
		retval = -ENOMEM;
		goto err_remap;
	}
	out_le32(mapped_immrbar + MPC83xx_PCIE2_OFFSET + PEX_EPIWTAR0,
			phys_addr | 0x00000001);
	pci_dev->pci_addr =
		in_le32(mapped_immrbar + MPC83xx_PCIE2_OFFSET + PEX_CFG_BAR0);
	pci_dev->pci_addr = pci_dev->pci_addr & 0xfffff000;

	if (check_mem_region(pci_dev->pci_addr, AGENT_DRIVER_MEM_SIZE)) {
		printk(KERN_INFO "cardnet:memory already in use!\n");
		retval = -EBUSY;
		goto err_checkmem;
	}
	if (request_mem_region(pci_dev->pci_addr, AGENT_DRIVER_MEM_SIZE,
				"cardnet") == NULL) {
		printk(KERN_INFO "cardnet:request memory region err!\n");
		retval = -ENOMEM;
		goto err_requestmem;
	}

	dev->mem_start = phys_addr;
	dev->mem_end = phys_addr + AGENT_DRIVER_MEM_SIZE;
	dev->base_addr = pci_dev->pci_addr;

	/*
	 * Assign the hardware address of the agent: use "\0FSLD0",
	 * The hardware address of the host use "\0FSLD1";
	 */
	memcpy(dev->dev_addr, "\0FSLD0", ETH_ALEN);
	printk(KERN_INFO "%s device is up\n", dev->name);

	netif_carrier_on(dev);
	netif_start_queue(dev);
	return 0;
err_requestmem:
err_checkmem:
	iounmap(mapped_immrbar);
err_remap:
	kfree(tp->pcidev);
err_nomem:
	free_pages((unsigned long)tp->share_mem, NEED_LOCAL_PAGE);
err_nopage:
	free_irq(tmr->irq, dev);
err_irq:
	tmr = NULL;
err_out:
	return retval;
}

int card_release(struct net_device *dev)
{
	struct card_priv *tp = netdev_priv(dev);
	struct cardnet_share_mem *share_mem = \
			(struct cardnet_share_mem *)tp->share_mem;
	struct pci_agent_dev *pci_dev;
	pci_dev = tp->pcidev;

	free_pages((unsigned long)share_mem, NEED_LOCAL_PAGE);
	/*iounmap(tp->mesgu);*/
	/* release ports, irq and such*/
	release_mem_region(pci_dev->pci_addr, AGENT_DRIVER_MEM_SIZE);
	kfree(tp->pcidev);
	free_irq(tmr->irq, dev);
	iounmap(mapped_immrbar);
	netif_stop_queue(dev); /* can't transmit any more */

	return 0;
}

/*
 * Configuration changes (passed on by ifconfig)
 */
int card_config(struct net_device *dev, struct ifmap *map)
{
	if (dev->flags & IFF_UP) /* can't act on a running interface */
		return -EBUSY;
	/* Don't allow changing the I/O address */
	if (map->base_addr != dev->base_addr) {
		printk(KERN_WARNING "cardnet: Can't change I/O address\n");
		return -EOPNOTSUPP;
	}
	/* Allow changing the IRQ */
	if (map->irq != dev->irq)
		dev->irq = map->irq;

	/* ignore other fields */
	return 0;
}

/*
 * Receive a packet: retrieve, encapsulate and pass over to upper levels
 */
void card_rx(struct net_device *dev, int len, unsigned char *buf)
{
	struct card_priv *priv = (struct card_priv *) netdev_priv(dev);
	struct sk_buff *skb;

	skb = dev_alloc_skb(len+2);
	if (!skb) {
		printk(KERN_ERR "card rx: low on mem - packet dropped\n");
		priv->stats.rx_dropped++;
		return;
	}

	skb_reserve(skb, 2);
	memcpy(skb_put(skb, len), buf, len);
	/* Write metadata, and then pass to the receive level */
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_UNNECESSARY; /* don't check it */
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += len;

	netif_rx(skb);
	return;
}


static int ppc83xx_remote_intr_obm(struct card_priv *priv, u16 message)
{
	/*send message to rc by outbound mail*/
	out_le32((mapped_immrbar + MPC83xx_PCIE2_OFFSET + 0xba0), 0x00000c00);
	out_le32((mapped_immrbar + MPC83xx_PCIE2_OFFSET + 0xb24), message);
	out_le32((mapped_immrbar + MPC83xx_PCIE2_OFFSET + 0xb20), 0x00000001);
}


/*
 * Transmit a packet (called by the kernel)
 */
int card_tx(struct sk_buff *skb, struct net_device *dev)
{
	int len;
	int j = 0, count = 0;
	char *data;
	struct card_priv *priv = (struct card_priv *) netdev_priv(dev);
	struct cardnet_share_mem *shmem =
				(struct cardnet_share_mem *)priv->share_mem;

	if (skb == NULL) {
		printk(KERN_ERR "skb is NULL\n");
		return 0;
	}

	len = skb->len < ETH_ZLEN ? ETH_ZLEN : skb->len;
	if (len < sizeof(struct ethhdr) + sizeof(struct iphdr)) {
		printk(KERN_INFO "packet too short (%i octets)\n", len);
		priv->stats.tx_dropped++;
		dev_kfree_skb(priv->cur_tx_skb);
		return 0;
	}
	if (len > 2040) {
		printk(KERN_INFO "packet too long (%i octets)\n", len);
		priv->stats.tx_dropped++;
		dev_kfree_skb(priv->cur_tx_skb);
		return 0;
	}

	spin_lock(&priv->lock);

	data = skb->data;
	dev->trans_start = jiffies; /* save the timestamp */
	priv->cur_tx_skb = skb;
	priv->tx_packetlen = len;
	while (shmem->hstatus == AGENT_SENT) {
		udelay(2); j++;
		if (j > 1000)
			break;
	}

	if (j > 1000) {
		netif_stop_queue(dev);
		priv->stats.tx_dropped++;
		spin_unlock(&priv->lock);
		return 0;
	}

	shmem->tx_flags = ++count;
	shmem->tx_packetlen = len;
	memcpy(shmem->txbuf, data, len);
	shmem->hstatus = AGENT_SENT;

	dev_kfree_skb(priv->cur_tx_skb);
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += priv->tx_packetlen;

	ppc83xx_remote_intr_obm(priv, AGENT_SENT);

	spin_unlock(&priv->lock);

	return 0;
}

/*
 * Deal with a transmit timeout.
 */
void card_tx_timeout(struct net_device *dev)
{
	struct card_priv *priv = (struct card_priv *) netdev_priv(dev);
	struct cardnet_share_mem *shmem =
			(struct cardnet_share_mem *)priv->share_mem;

	printk(KERN_INFO "Transmit timeout at %ld, latency %ld\n",
		jiffies, jiffies - dev->trans_start);

	dev_kfree_skb(priv->cur_tx_skb);
	shmem->hstatus = 0;
	priv->stats.tx_errors++;
	netif_wake_queue(dev);
	return;
}

/*
 * Ioctl commands
 */
int card_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	printk(KERN_DEBUG "ioctl\n");
	return 0;
}

struct net_device_stats *card_stats(struct net_device *dev)
{
	struct card_priv *priv = (struct card_priv *) netdev_priv(dev);
	return &priv->stats;
}

int card_rebuild_header(struct sk_buff *skb)
{
	struct ethhdr *eth = (struct ethhdr *) skb->data;
	struct net_device *dev = skb->dev;

	memcpy(eth->h_source, dev->dev_addr, dev->addr_len);
	memcpy(eth->h_dest, dev->dev_addr, dev->addr_len);
	eth->h_dest[ETH_ALEN-1] ^= 0x01; /* dest is us xor 1 */

	return 0;
}

int card_header(struct sk_buff *skb, struct net_device *dev,
		unsigned short type, void *daddr, void *saddr,
		unsigned int len)
{
	struct ethhdr *eth = (struct ethhdr *)skb_push(skb, ETH_HLEN);

	eth->h_proto = htons(type);
	memcpy(eth->h_source, dev->dev_addr, dev->addr_len);
	memcpy(eth->h_dest,   dev->dev_addr, dev->addr_len);
	eth->h_dest[ETH_ALEN-1]   ^= 0x01;
	return dev->hard_header_len;
}

int card_change_mtu(struct net_device *dev, int new_mtu)
{
	spinlock_t *lock = &((struct card_priv *) netdev_priv(dev))->lock;
	unsigned long flags;

	/* check ranges */
	if ((new_mtu < 68) || (new_mtu > 1500))
		return -EINVAL;
	spin_lock_irqsave(lock, flags);
	dev->mtu = new_mtu;
	spin_unlock_irqrestore(lock, flags);
	return 0; /* success */
}

/*
 * init function.
 */
void card_init(struct net_device *dev)
{
	struct card_priv *priv;
	ether_setup(dev); /* assign some of the fields */
	dev->open		= card_open;
	dev->stop		= card_release;
	dev->set_config		= card_config;
	dev->hard_start_xmit	= card_tx;
	dev->do_ioctl		= card_ioctl;
	dev->get_stats		= card_stats;
	dev->change_mtu		= card_change_mtu;
	/*dev->rebuild_header	= card_rebuild_header;*/
	/*dev->hard_header	= card_header;*/

#ifdef HAVE_TX_TIMEOUT
	dev->tx_timeout		= card_tx_timeout;
	dev->watchdog_timeo	= timeout;
#endif

	dev->watchdog_timeo = timeout;

	/* keep the default flags, just add NOARP */
	dev->flags	|= IFF_NOARP;
	dev->features	|= NETIF_F_NO_CSUM;
	/*dev->hard_header_cache = NULL;*/	/* Disable caching */
	/*SET_MODULE_OWNER(dev);*/

	/*
	 * Then, allocate the priv field. This encloses the statistics
	 * and a few private fields.
	 */
	priv = netdev_priv(dev);
	memset(priv, 0, sizeof(struct card_priv));

	/*priv->mesgu = ioremap(get_immrbase() + MPC83xx_DMA_OFFSET,
				MPC83xx_DMA_SIZE);*/
	/*dev->irq = ppc83xx_agent_interrupt_init(priv->mesgu);*/
	spin_lock_init(&((struct card_priv *) netdev_priv(dev))->lock);
	return;
}

/*
 * The devices
 */
struct net_device *card_devs;

static __init int card_init_module(void)
{
	int result, device_present = 0;
	int card_eth;
	char interface_name[16];

	card_eth = eth; /* copy the cfg datum in the non-static place */
	if (!card_eth)
		strcpy(interface_name, "ceth%d");
	else
		strcpy(interface_name, "eth%d");

	card_devs = alloc_netdev(sizeof(struct card_priv),
					interface_name, card_init);
	if (card_devs == NULL)
		return -ENODEV;

	result = register_netdev(card_devs);
	if (result) {
		printk(KERN_ERR "card: error %i registering device \"%s\"\n",
			result, interface_name);
		free_netdev(card_devs);
	} else
		device_present++;
	printk(KERN_INFO "register device named-----%s\n", card_devs->name);
	printk(KERN_INFO "mpc83xx agent drvier init succeed\n");

	return device_present ? 0 : -ENODEV;
}

static __exit void card_cleanup(void)
{
	unregister_netdev(card_devs);
	free_netdev(card_devs);
	return;
}
module_init(card_init_module);
module_exit(card_cleanup);

MODULE_AUTHOR("Xiaobo Xie<X.Xie@freescale.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MPC83xx Processor PCIE Agent Ethernet Driver");
