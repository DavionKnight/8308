/*********************************
 *Author:     zhangjj@bjhuahuan.com
 *date:	      2015-10-20
 *Modified:
 ********************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>

extern int fpga_spi_read(u16 addr, u16 size, u16 *data_buf);
extern int fpga_spi_write(u16 addr, u16 size, u16 *data_buf);

static int __init spitest_init(void)
{
        int status = 0;
	char buf[20];

     	printk(KERN_ALERT "spitest driver init...");

	fpga_spi_read(0,1,buf);
	printk("0 buf=0x%x 0x%x 0x%x\n",buf[0],buf[1],buf[2]);
	fpga_spi_read(0xb,1,buf);
	printk("b buf=0x%x 0x%x 0x%x\n",buf[0],buf[1],buf[2]);
	fpga_spi_read(0xc,1,buf);
	printk("c buf=0x%x 0x%x 0x%x\n",buf[0],buf[1],buf[2]);
	fpga_spi_read(0xd,1,buf);
	printk("d buf=0x%x 0x%x 0x%x\n",buf[0],buf[1],buf[2]);
	printk("write \n");
	buf[0] = 0xdd;
	buf[1] = 0xdd;
	fpga_spi_write(0xd, 1, buf);
	buf[0] = 0xcc;
	buf[1] = 0xcc;
	fpga_spi_write(0xc, 1, buf);
	buf[0] = 0xbb;
	buf[1] = 0xbb;
	fpga_spi_write(0xb, 1, buf);

	fpga_spi_read(0,1,buf);
	printk("0 buf=0x%x 0x%x 0x%x\n",buf[0],buf[1],buf[2]);
	fpga_spi_read(0xb,1,buf);
	printk("b buf=0x%x 0x%x 0x%x\n",buf[0],buf[1],buf[2]);
	fpga_spi_read(0xc,1,buf);
	printk("c buf=0x%x 0x%x 0x%x\n",buf[0],buf[1],buf[2]);
	fpga_spi_read(0xd,1,buf);
	printk("d buf=0x%x 0x%x 0x%x\n",buf[0],buf[1],buf[2]);

     	printk(KERN_ALERT "Done\n");

     	return 0;
}

static void __exit spitest_exit(void)
{
     	printk(KERN_ALERT "spitest exit\n");
}

module_init(spitest_init);
module_exit(spitest_exit);
MODULE_LICENSE("GPL");


