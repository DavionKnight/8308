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
#include <linux/uaccess.h>

#define SPI_R	0
#define SPI_W	1

typedef struct fpga_rw_t{
	unsigned short addr;
	unsigned short size;
	unsigned short buf[20];
}fpga_rw;

extern int fpga_spi_read(u16 addr, u16 size, u16 *data_buf);
extern int fpga_spi_write(u16 addr, u16 size, u16 *data_buf);

static long spi_rw_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
        int  err = 0;
        int  retval = 0;

        int i;
        fpga_rw opt;

	copy_from_user(&opt, (fpga_rw *)arg, sizeof(fpga_rw));

        switch (cmd)
        {
                case SPI_R:
			fpga_spi_read(opt.addr, opt.size, opt.buf);
			copy_to_user((fpga_rw*)arg, &opt, sizeof(fpga_rw));		
			break;
		case SPI_W:
			fpga_spi_write(opt.addr, opt.size, opt.buf);
			break;
		default:
			printk("command error\n");
	}
	return 0;
}

static int spi_rw_open(struct inode *inode, struct file *filp)
{
        return 0;
}
static int spi_rw_release(struct inode *inode, struct file *filp)
{
        return 0;
}

static struct file_operations spi_rw_fops = {
        .owner          = THIS_MODULE,
        .unlocked_ioctl = spi_rw_ioctl,
        .open           = spi_rw_open,
        .release        = spi_rw_release,
};

static struct class *spi_cls;
int major;
struct cdev spiapi_cdev;

static int __init spiApi_init(void)
{
        int status = 0;
	char buf[20];
	 dev_t dev_id;

     	printk(KERN_ALERT "spiApi driver init...");

	fpga_spi_read(0, 1, buf);
	printk("buf=0x%x 0x%x\n",buf[0],buf[1]);

        alloc_chrdev_region(&dev_id, 0, 1, "spiapi");
        major = MAJOR(dev_id);

        cdev_init(&spiapi_cdev, &spi_rw_fops);
        cdev_add(&spiapi_cdev, dev_id, 1);

        spi_cls = class_create(THIS_MODULE, "spiapi");

        device_create(spi_cls, NULL, dev_id, NULL, "spiapi");


     	printk(KERN_ALERT "Done\n");

     	return 0;
}

static void __exit spiApi_exit(void)
{
     	printk(KERN_ALERT "spiApi exit\n");
}

module_init(spiApi_init);
module_exit(spiApi_exit);
MODULE_LICENSE("GPL");


