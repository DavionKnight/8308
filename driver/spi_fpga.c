#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <asm/uaccess.h>
#include <linux/time.h>

extern int fpga_spi_read(u16 addr, u16 size, u8 *data_buf);
extern int fpga_spi_write(u16 addr, u16 size, u8 *data_buf);

static int __init test_init(void)
{
	int status;
	struct spi_device *spi;
	unsigned char buf[20]={0xaa,0xaa,0xaa,0xaa};
	 
	printk(KERN_ALERT "driver init!\n");
	fpga_spi_read(0xa,2,buf);

	return 0;
}

static void __exit test_exit(void)
{
}

module_init(test_init);
module_exit(test_exit);
MODULE_LICENSE("GPL");
