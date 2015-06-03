/*
 * Wake up PM by GTM Timer4 Module
 *
 * Copyright (C) 2009 Freescale Semiconductor, Inc.
 *
 * Author: Jiang Yutang <b14898@freescale.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <sysdev/fsl_soc.h>
#include <linux/proc_fs.h>
#include <asm/fsl_gtm.h>

static struct gtm_timer *wpmtimer;
static struct proc_dir_entry *proc_wpmtimer;

static int wpmtimer_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	unsigned int len;

	len = sprintf(page, "%08d\n", 0);
	return len;
}

static ssize_t wpmtimer_write( struct file *filp, const char __user *buff, unsigned long len, void *data )
{
	unsigned long interval = simple_strtoul(buff, NULL, 0);

	gtm_set_timer16(wpmtimer, interval * 1000000, 0);

	return len;
}

irqreturn_t wpmtimer_irq(int irq, void *p)
{
	gtm_stop_timer16(wpmtimer);
	return IRQ_HANDLED;
}

static int __init wpmtimer_init(void)
{
	int ret = 0;
	struct device_node *np, *gtm;
	const phandle *ph;

	/* find wakeup timer */
	for_each_compatible_node(np, NULL, "fsl,mpc8313-pmc") {
		ph = of_get_property(np, "fsl,mpc8313-wakeup-timer", NULL);
		if (ph){
			gtm = of_find_node_by_phandle(*ph);
			of_node_put(np);
			break;
		}
		of_node_put(np);
	}
	if (!gtm) {
		printk("Can not find device node : fsl,mpc8313-wakeup-timer\n");
		ret = -ENODEV;
		goto err_node;
	}
	/* get GTM timer 4*/
	wpmtimer = gtm_get_specific_timer16(gtm->data, 3);
	of_node_put(gtm);

	if (!wpmtimer) {
		printk("failed to request timer\n");
		ret = -ENODEV;
		goto err_gettimer;
	}

	ret = request_irq(wpmtimer->irq, wpmtimer_irq, IRQF_DISABLED, "wpmtimer", 0);
	if (ret) {
		printk("failed to request timer irq\n");
		ret = -ENODEV;
		goto err_irq;
	}

	proc_wpmtimer = create_proc_entry( "wpmtimeout", 0644, NULL );
	if (proc_wpmtimer == NULL) {
		printk("failed to create /proc/wpmtimeout \n");
		ret = -ENODEV;
		goto err_proc;
	} else {
		proc_wpmtimer->read_proc = wpmtimer_read;
		proc_wpmtimer->write_proc = wpmtimer_write;
		proc_wpmtimer->owner = THIS_MODULE;
		printk("create /proc/wpmtimeout success\n");
	}

	printk("wpmtimer init success.\n");
	return 0;
err_proc:
	free_irq(wpmtimer->irq, 0);
err_irq:
	gtm_put_timer16(wpmtimer);
err_gettimer:
err_node:
	return ret;
}

static void __exit wpmtimer_exit(void)
{
	remove_proc_entry("wpmtimer", NULL);
	free_irq(wpmtimer->irq, 0);
	gtm_put_timer16(wpmtimer);
	printk("wpmtimer exit success.\n");
}

module_init(wpmtimer_init);
module_exit(wpmtimer_exit);

MODULE_DESCRIPTION("Wake up PM by GTM Timer4 Module");
MODULE_AUTHOR("Jiang Yutang <b14898@freescale.com>");
MODULE_LICENSE("GPL");
