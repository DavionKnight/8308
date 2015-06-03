/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * Author: Freescale unknown
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS for A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <hwconfig.h>
#include <i2c.h>
#include <libfdt.h>
#include <fdt_support.h>
#include <pci.h>
#include <mpc83xx.h>
#include <netdev.h>
#include <asm/io.h>
#include <asm/fsl_serdes.h>
#include <ns16550.h>
#include <nand.h>

DECLARE_GLOBAL_DATA_PTR;

int board_early_init_f(void)
{
	volatile immap_t *im = (immap_t *)CONFIG_SYS_IMMR;
//#ifdef CONFIG_MMC
	volatile clk83xx_t *clk = (volatile clk83xx_t *)&im->clk;
//#endif

	if (im->pmc.pmccr1 & PMCCR1_POWER_OFF)
		gd->flags |= GD_FLG_SILENT;

#ifdef CONFIG_MMC
	/* Configure the clock for SDHC controller */
	clrsetbits_be32(&clk->sccr, SCCR_SDHCCM, SCCR_SDHCCM_1);

	/* Enable cache snooping in eSDHC controller in system configuration registers */
	im->sysconf.sdhccr = 0x02000000;
#endif
	/* Configure the clock for USB controller */
	clrsetbits_be32(&clk->sccr, SCCR_USBDRCM, SCCR_USBDRCM_1);

{
int i;
#define FPGA_RESET_BIT		0x01000000
#define LED_BIT_1			0x01400000
#define LED_BIT_2			0x01200000
	setbits_be32(&im->gpio->dir, FPGA_RESET_BIT);
	
	//
	clrbits_be32(&im->gpio->dat, FPGA_RESET_BIT);
	for(i = 0; i< 2000; ++i)
				;
	setbits_be32(&im->gpio->dat, FPGA_RESET_BIT);
}
	return 0;
}

#ifndef CONFIG_NAND_SPL
static u8 read_board_info(void)
{
	return 0xEC;
}

int checkboard(void)
{
	static const char * const rev_str[] = {
		"0.0",
		"0.1",
		"1.0",
		"1.1",
		"<unknown>",
	};
	u8 info;
	int i;

	info = read_board_info();
	i = (!info) ? 4 : info & 0x03;

	printf("Board: Freescale MPC8308ERDB Rev %s\n", rev_str[i]);

	return 0;
}


/*
 * Miscellaneous late-boot configurations
 *
*/
/*int misc_init_r(void)
{
	int rc = 0;

	return rc;
}*/
#if defined(CONFIG_OF_BOARD_SETUP)
void fdt_tsec1_fixup(void *fdt, bd_t *bd)
{
	const char disabled[] = "disabled";
	const char *path;
	int ret;

	if (hwconfig_arg_cmp("board_type", "tsec1")) {
		return;
	} else if (!hwconfig_arg_cmp("board_type", "ulpi")) {
		printf("NOTICE: No or unknown board_type hwconfig specified.\n"
		       "        Assuming board with TSEC1.\n");
		return;
	}

	ret = fdt_path_offset(fdt, "/aliases");
	if (ret < 0) {
		printf("WARNING: can't find /aliases node\n");
		return;
	}

	path = fdt_getprop(fdt, ret, "ethernet0", NULL);
	if (!path) {
		printf("WARNING: can't find ethernet0 alias\n");
		return;
	}

	do_fixup_by_path(fdt, path, "status", disabled, sizeof(disabled), 1);
}

void ft_board_setup(void *blob, bd_t *bd)
{
	ft_cpu_setup(blob, bd);
//	fdt_fixup_dr_usb(blob, bd);
//	fdt_tsec1_fixup(blob, bd);
}
#endif

int board_eth_init(bd_t *bis)
{
	cpu_eth_init(bis);	/* Initialize TSECs first */
	return pci_eth_init(bis);
}
#else
void board_init_f(ulong bootflag)
{
	volatile immap_t *immr = (volatile immap_t *)CONFIG_SYS_IMMR;
	u32 csb_clk;
	u8 spmf;

	spmf = ((immr->reset.rcwl & HRCWL_SPMF) >> HRCWL_SPMF_SHIFT);

#if defined(CONFIG_83XX_CLKIN)
		csb_clk = CONFIG_83XX_CLKIN * spmf;
#else
		csb_clk = 0;
#endif /* CONFIG_83XX_CLKIN */

	board_early_init_f();
	NS16550_init((NS16550_t)CONFIG_SYS_NS16550_COM1,
			csb_clk / 16 / CONFIG_BAUDRATE);
	puts("\nNAND boot...... ");
	init_timebase();
	initdram(0);
	relocate_code(CONFIG_SYS_NAND_U_BOOT_RELOC + 0x10000, (gd_t *)gd,
			CONFIG_SYS_NAND_U_BOOT_RELOC);
}

void board_init_r(gd_t *gd, ulong dest_addr)
{
	nand_boot();
}

void putc(char c)
{
	if (gd->flags & GD_FLG_SILENT)
		return;

	if (c == '\n')
		NS16550_putc((NS16550_t)CONFIG_SYS_NS16550_COM1, '\r');

	NS16550_putc((NS16550_t)CONFIG_SYS_NS16550_COM1, c);
}
#endif


