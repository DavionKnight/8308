/*!***************************************************************************
*!
*! FILE NAME  : spi_drv.c
*!
*! DESCRIPTION: 
*! spi_drv_v2
*! 
*! 
*! Revision 1.20  2013/09/12 09:02:28 huahuan
*! 
*!
*!***************************************************************************/


#include <stddef.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h> 
#include <linux/errno.h> 
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/version.h>
#include <linux/init.h>

#include "spi_drv.h"

#ifdef	DEBUG
# define debugk(fmt,args...)	printk(fmt ,##args)
#else
# define debugk(fmt,args...)
#endif

int fpga_spi_read(u16 addr, u16 size, u16 *data_buf);
int fpga_spi_write(u16 addr, u16 size, u16 *data_buf);

#define DEV_MAJOR				207
#define DRIVER_VERSION			"$ Revision: 2.00 $"
#define SPI_DEV_NUM				1

static unsigned int 	dev_major; 
static char 			devname[]		= "spi_drv";
struct cdev 			spi_cdev;

static void spi_rd(spi_reg * preg)
{
	int ret;
	
	ret = fpga_spi_read(preg->addr, preg->size, preg->pbuf);
	
	if(ret < 0)
		printk(KERN_INFO"read fpga failed id=%d\n", ret);
}

static void spi_wt(spi_reg *preg)
{
	int ret;
	
	ret = fpga_spi_write(preg->addr, preg->size, preg->pbuf);
	
	if(ret < 0)
		printk(KERN_INFO"write fpga failed id=%d\n", ret);
}

/*-----------------------------------------------------------------------------
 *  port num must be remap due to the hardware connection
 *-----------------------------------------------------------------------------*/

static int spi_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg) 
{
	spi_reg reg;

	if( copy_from_user(&reg, (spi_reg *)arg, sizeof(spi_reg)) )
		return -EFAULT;

	switch( cmd )
	{
		case STATUS_SPI_GET:
			spi_rd(&reg);
			if (copy_to_user((spi_reg *)arg, &reg, sizeof(spi_reg)))
				return -EFAULT;
			return (0);
		break;
		case STATUS_SPI_SET:
			spi_wt(&reg);
			return (0);
		break;
	}
	
	return (-ENODEV);
}

static int spi_open( struct inode *inode, struct file *file )
{
	return 0; 
}

static int spi_release( struct inode  * inode, struct file * filp ) 
{
	return 0;
}

static struct file_operations spi_fops = 
{
	.owner        = THIS_MODULE,
	.ioctl        = spi_ioctl,
	.open	      = spi_open,
	.release	  = spi_release,	
}; 

/*
 * Initialize the driver - Register the character device
 */
static int __init spi_init(void)
{
	int result;

	dev_t devno = MKDEV(DEV_MAJOR, 0);
		
	if (DEV_MAJOR)
	{
		result = register_chrdev_region(devno,  SPI_DEV_NUM, devname);
	}
	else  
	{
        result = alloc_chrdev_region(&devno, 0, SPI_DEV_NUM, devname);
		dev_major = MAJOR(devno);
    }  
  	
	cdev_init(&spi_cdev, &spi_fops);
	spi_cdev.owner = THIS_MODULE;
	spi_cdev.ops = &spi_fops;
	cdev_add(&spi_cdev, MKDEV(DEV_MAJOR, 0), SPI_DEV_NUM);
	
	printk (KERN_INFO"%s driver " DRIVER_VERSION " initialized\n", devname );
	
	return (0);
}

/*
 * Cleanup - unregister the driver
 */
void spi_cleanup(void)
{
	/*
	 * Unregister the device
	 */
	
	cdev_del(&spi_cdev);   
	unregister_chrdev_region(MKDEV(DEV_MAJOR, 0),SPI_DEV_NUM);
}

module_init( spi_init );
module_exit( spi_cleanup );
MODULE_LICENSE("GPL");
