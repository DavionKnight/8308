/*
 * drivers/net/cardnet/pci_agent_lib.h
 *
 * Copyright (C) 2005-2009 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Xiaobo Xie <X.Xie@freescale.com>
 *         Jiang Yutang <b14898@freescale.com>
 *
 * Description:
 * Freescale mpc8315erdb pcie control registers memory map.
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

#ifndef PCI_AGENT_LIB_H
#define PCI_AGENT_LIB_H

#define MPC83xx_PCI1_OFFSET	0x8500
#define MPC83xx_PCI1_SIZE	0x100
#define MPC83xx_PCIE2_OFFSET	0xA000
#define MPC83xx_DMA_OFFSET	0x8000
#define MPC83xx_DMA_SIZE	0x300
#define MPC83xx_IOS_OFFSET	0x8400
#define MPC83xx_IOS_SIZE	0x100
#define MPC83xx_GTM1_OFFSET	0x500

#define PPC83XX_NETDRV_NAME	"boardnet: PPC83xx PCIE Agent Ethernet Driver"
#define DRV_VERSION		"1.0"

#define PFX PPC83XX_NETDRV_NAME ": "

#define	IMMRBAR_SIZE	0x00100000

#define	AGENT_MEM_BASE_ADDR	0x00
#define	AGENT_MEM_SIZE		0x00001000

/* tx, rx and device flags */
#define	AGENT_SENT		0x00000a01
#define	AGENT_GET		0x00000a00
#define	HOST_SENT		0x00000501
#define HOST_GET		0x00000500

/* Define max packet buffer */
#define	MAX_PACKET_BUF		(2*1024)
#define NEED_LOCAL_PAGE		0

/* Default timeout period */
#define	BOARDNET_TIMEOUT	5	/* In jiffies */

#ifdef BOARDNET_NDEBUG
# define assert(expr) do {} while (0)
#else
# define assert(expr) \
	if (!(expr)) { \
		printk(KERN_DEBUG "Assertion failed! %s,%s,%s,line=%d\n", \
		#expr, __FILE__, __func__, __LINE__); \
	}
#endif


#define CFR2 0x4 /* Timer3/4 Configuration  */
#define CFR2_STP4 0x20 /* Stop timer  */
#define CFR2_GM4  0x08 /* Gate mode for pin 4  */
#define CFR2_RST4 0x10 /* Reset timer  */
#define EVR4 0x36/* Timer4 Event Register  */
#define GTEVR_REF 0x0002 /* Output reference event  */
#define GTEVR_CAP 0x0001 /* Counter Capture event   */
#define PSR4 0x3e /* Timer4 Prescaler Register  */
#define GTPSR_PPS  0x00FF /* Primary Prescaler Bits (256). */
#define CNR4 0x2e /* Timer4 Counter Register  */
#define RFR4 0x26 /* Timer4 Reference Register  */
#define MDR4 0x22 /* Timer4 Mode Register  */
#define MDR_SPS  0xff00 /* Secondary Prescaler value (256) */
#define MDR_ORI  0x0010 /* Output reference interrupt enable  */
#define MDR_FRR  0x0008 /* Free run/restart  */
#define MDR_ICLK_DIV16 0x0004

struct pci_agent_dev {
       u32     local_addr;
       u32     mem_addr;
       u32     mem_size;
       u32     pci_addr;
       u32     window_num;
       u32     irq;
       u32     message;
};

/*
 *  PCI Express
 */
struct pex_inbound_window {
	u32 ar;
	u32 tar;
	u32 barl;
	u32 barh;
};

struct pex_outbound_window {
	u32 ar;
	u32 bar;
	u32 tarl;
	u32 tarh;
};

struct pex_csb_bridge {
	u32 pex_csb_ver;
	u32 pex_csb_cab;
	u32 pex_csb_ctrl;
	u8 res0[8];
	u32 pex_dms_dstmr;
	u8 res1[4];
	u32 pex_cbs_stat;
	u8 res2[0x20];
	u32 pex_csb_obctrl;
	u32 pex_csb_obstat;
	u8 res3[0x98];
	u32 pex_csb_ibctrl;
	u32 pex_csb_ibstat;
	u8 res4[0xb8];
	u32 pex_wdma_ctrl;
	u32 pex_wdma_addr;
	u32 pex_wdma_stat;
	u8 res5[0x94];
	u32 pex_rdma_ctrl;
	u32 pex_rdma_addr;
	u32 pex_rdma_stat;
	u8 res6[0xd4];
	u32 pex_ombcr;
	u32 pex_ombdr;
	u8 res7[0x38];
	u32 pex_imbcr;
	u32 pex_imbdr;
	u8 res8[0x38];
	u32 pex_int_enb;
	u32 pex_int_stat;
	u32 pex_int_apio_vec1;
	u32 pex_int_apio_vec2;
	u8 res9[0x10];
	u32 pex_int_ppio_vec1;
	u32 pex_int_ppio_vec2;
	u32 pex_int_wdma_vec1;
	u32 pex_int_wdma_vec2;
	u32 pex_int_rdma_vec1;
	u32 pex_int_rdma_vec2;
	u32 pex_int_misc_vec;
	u8 res10[4];
	u32 pex_int_axi_pio_enb;
	u32 pex_int_axi_wdma_enb;
	u32 pex_int_axi_rdma_enb;
	u32 pex_int_axi_misc_enb;
	u32 pex_int_axi_pio_stat;
	u32 pex_int_axi_wdma_stat;
	u32 pex_int_axi_rdma_stat;
	u32 pex_int_axi_misc_stat;
	u8 res11[0xa0];
	struct pex_outbound_window pex_outbound_win[4];
	u8 res12[0x100];
	u32 pex_epiwtar0;
	u32 pex_epiwtar1;
	u32 pex_epiwtar2;
	u32 pex_epiwtar3;
	u8 res13[0x70];
	struct pex_inbound_window pex_inbound_win[4];
};

struct pex83xx {
	u8 pex_cfg_header[0x404];
	u32 pex_ltssm_stat;
	u8 res0[0x30];
	u32 pex_ack_replay_timeout;
	u8 res1[4];
	u32 pex_gclk_ratio;
	u8 res2[0xc];
	u32 pex_pm_timer;
	u32 pex_pme_timeout;
	u8 res3[4];
	u32 pex_aspm_req_timer;
	u8 res4[0x18];
	u32 pex_ssvid_update;
	u8 res5[0x34];
	u32 pex_cfg_ready;
	u8 res6[0x24];
	u32 pex_bar_sizel;
	u8 res7[4];
	u32 pex_bar_sel;
	u8 res8[0x20];
	u32 pex_bar_pf;
	u8 res9[0x88];
	u32 pex_pme_to_ack_tor;
	u8 res10[0xc];
	u32 pex_ss_intr_mask;
	u8 res11[0x25c];
	struct pex_csb_bridge bridge;
	u8 res12[0x160];
};

#endif

