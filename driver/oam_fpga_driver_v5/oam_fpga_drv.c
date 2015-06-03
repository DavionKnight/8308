/*!***************************************************************************
*!
*! FILE NAME  : oam_fpga_drv.c
*!
*! DESCRIPTION: 
*! oam_fpga_drv_v4
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

#include "oam_fpga_drv.h"

//#define	DEBUG

#ifdef	DEBUG
# define debugk(fmt,args...)	printk(fmt ,##args)
#else
# define debugk(fmt,args...)
#endif

int fpga_spi_read(u16 addr, u16 size, u16 *data_buf);
int fpga_spi_write(u16 addr, u16 size, u16 *data_buf);

#define DEV_MAJOR				206
#define DRIVER_VERSION			"$Revision: 1.00 $"
#define OAM_DEV_NUM				1

static unsigned int 	dev_major; 
static char 		devname[]		= "oam_fpga_drv";
struct cdev 		oam_fpga_cdev;

static void oam_fpga_rd(uint8 cid, uint8 addr, uint8 *buf, uint len)
{
	unsigned short	tmp, i;

	/*check fpga is idle*/
	spi_is_idle()
	//addr
	spi_wr_addr((((unsigned short)(0x60 | ((cid & 0x7) << 1))) << 8) | addr)
	//size
	spi_wr_size(len)
	//enable
	spi_enable(SPI_RdEn)
	/*check fpga is idle*/
	spi_is_idle()
	//data
	for(i = 0; i < len && i < 8; i++)
	{
		fpga_spi_read(SPI_RdData + i, 1, &tmp);
		buf[i] = tmp;
	}
	printk("\t\rend: songwenting robo_read addr=%x\n\n", addr);
}

static void oam_fpga_wt(uint8 cid, uint8 addr, uint8 *buf, uint len)
{
	unsigned short	tmp, i;

	/*check fpga is idle*/
	spi_is_idle()
	//addr
	spi_wr_addr((((unsigned short)(0x61 | ((cid & 0x7) << 1))) << 8) | addr);
	//size
	spi_wr_size(len)
	//data
	for(i = 0; i < len && i < 8; i++)
	{
		tmp = buf[i];
		fpga_spi_write(SPI_WrData + i, 1, &tmp);
	}
	printk("\t\rend: songwenting robo_write addr=%x\n\n", addr);
	//enable
	spi_enable(SPI_WrEn)
	/*check fpga is idle*/
	spi_is_idle()
}


static int oam_fpga_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg) 
{
	oam_fpga_reg reg;

	if( copy_from_user(&reg, (oam_fpga_reg *)arg, sizeof(oam_fpga_reg)) )
		return -EFAULT;

	switch( cmd )
	{
		case STATUS_OAM_FPGA_GET:
					
			oam_fpga_rd(reg.cid, reg.addr, reg.buf, reg.len);
			
			if (copy_to_user((oam_fpga_reg *)arg, &reg, sizeof(oam_fpga_reg)))
				return -EFAULT;	
			return (0);

		case STATUS_OAM_FPGA_SET:
			
			oam_fpga_wt(reg.cid, reg.addr, reg.buf, reg.len);
			return (0);
	}
	
	return (-ENODEV);
}

static int oam_fpga_open( struct inode *inode, struct file *file )
{
	return 0; 
}

static int oam_fpga_release( struct inode  * inode, struct file * filp ) 
{
	return 0;
} 
static struct file_operations oam_fpga_fops = 
{
	.owner        = THIS_MODULE,
	.ioctl        = oam_fpga_ioctl,
	.open	     = oam_fpga_open,
	.release	     = oam_fpga_release,	
}; 


/*
 * Initialize the driver - Register the character device
 */
static int __init oam_fpga_init(void)
{
	int result;

	//memory map
	//request_mem_region(FPGA_PHY_ADDR,FPGA_MEM_SIZE,fpga_driver);
	dev_t devno = MKDEV(DEV_MAJOR, 0);

	if (DEV_MAJOR)
	{
        result = register_chrdev_region(devno,  OAM_DEV_NUM, devname);
	}
	else  
	{
        result = alloc_chrdev_region(&devno, 0, OAM_DEV_NUM, devname);
       	dev_major = MAJOR(devno);
    }  
  	
	cdev_init(&oam_fpga_cdev, &oam_fpga_fops);
	oam_fpga_cdev.owner = THIS_MODULE;
	oam_fpga_cdev.ops = &oam_fpga_fops;

	cdev_add(&oam_fpga_cdev, MKDEV(DEV_MAJOR, 0), OAM_DEV_NUM);
	
	printk (KERN_INFO"%s driver" DRIVER_VERSION " initialized\n", devname );

	return (0);
}


/*
 * Cleanup - unregister the driver
 */
void oam_fpga_cleanup(void)
{

	/*
	 * Unregister the device
	 */

	cdev_del(&oam_fpga_cdev);   
    unregister_chrdev_region(MKDEV(DEV_MAJOR, 0),OAM_DEV_NUM); 
	/*
	 * If there's an error, report it
	 */
	/*if(ret < 0)
	 *{
	 *	printk ("b5325_drv unregister_chrdev: error %d\n", ret);
	 *}
	 */
}

module_init( oam_fpga_init );
module_exit( oam_fpga_cleanup );
MODULE_LICENSE("GPL");
