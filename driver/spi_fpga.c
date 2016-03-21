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
	int status,i=0;
	struct spi_device *spi;
	struct timeval start,stop;

	unsigned char buf[20]={0xaa,0xaa,0xaa,0xaa};
	 
	printk(KERN_ALERT "driver init!\n");
	do_gettimeofday(&start);
	for(i=0;i<1000;i++)
	{
		fpga_spi_read(0x2c,2,buf);
		fpga_spi_write(0x2c,2,buf);
	}
	do_gettimeofday(&stop);
	printk("start:tv_sec=%d,tv_usec=%d\n",start.tv_sec,start.tv_usec);
	printk("stop :tv_sec=%d,tv_usec=%d\n",stop.tv_sec,stop.tv_usec);
	printk("buf=0x%02x 0x%02x\n",buf[0],buf[1]);
	return 0;
}

static void __exit test_exit(void)
{
}

module_init(test_init);
module_exit(test_exit);
MODULE_LICENSE("GPL");
