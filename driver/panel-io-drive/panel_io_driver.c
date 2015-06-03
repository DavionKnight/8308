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
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <linux/of_platform.h>
#include <asm/time.h>
#include <asm/ipic.h>
#include <asm/udbg.h>
#include <sysdev/fsl_pci.h>
#include <sysdev/fsl_soc.h>
#include <linux/spi/spi.h>

#include <asm/qe.h>
#include <asm/qe_ic.h>
#include <linux/delay.h>
//#include <linux/unistd.h> 

#include "panel_io_driver.h"

#define 		PANEL_IO_DEV_MAJOR      231
#define 		DRIVER_NAME			    "panel_io"

#define 		MPC8XXX_GPIO_PINS		32

#define		    GPIO_ADDR_OFFSET        0xc00
#define         SICRH_ADDR_OFFSET       0x118

#define 		GPIO_DIR				0x00
#define 		GPIO_ODR				0x04
#define 		GPIO_DAT				0x08
#define 		GPIO_IER				0x0c
#define 		GPIO_IMR				0x10
#define 		GPIO_ICR				0x14
#define         RST_SWITCH              16

static struct mutex		ioctl_lock;

void __iomem *immap;


static inline u32 mpc8xxx_gpio2mask(unsigned int gpio)
{
	return 1u << (MPC8XXX_GPIO_PINS - 1 - gpio);
}
/*
void sleep(void)
{
	unsigned int n;
	for(n=0;n<(~0);n++);
}
*/
static long
panel_io_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	
	if (_IOC_TYPE(cmd) != PANEL_IO_MAGIC)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	mutex_lock(&ioctl_lock);

	switch (cmd) 
	{
		case BCM53284_RESET:
			clrbits32(immap + GPIO_ADDR_OFFSET + GPIO_DAT, mpc8xxx_gpio2mask(Reset_s));
			mdelay(500);
			setbits32(immap + GPIO_ADDR_OFFSET + GPIO_DAT, mpc8xxx_gpio2mask(Reset_s));
			break;
		case BCM5482S_RESET:
			clrbits32(immap + GPIO_ADDR_OFFSET + GPIO_DAT, mpc8xxx_gpio2mask(Reset_phy));
			mdelay(500);
			setbits32(immap + GPIO_ADDR_OFFSET + GPIO_DAT, mpc8xxx_gpio2mask(Reset_phy));
			break;
		case FPGA_RESET:
			clrbits32(immap + GPIO_ADDR_OFFSET + GPIO_DAT, mpc8xxx_gpio2mask(nFPGA_RESET));
			mdelay(500);
			setbits32(immap + GPIO_ADDR_OFFSET + GPIO_DAT, mpc8xxx_gpio2mask(nFPGA_RESET));
			break;
		case MPC_OUT_FLASH_EN:
			clrbits32(immap + GPIO_ADDR_OFFSET + GPIO_DAT, mpc8xxx_gpio2mask(CPU_OUT_FLASH_EN));
			break;
		case FPGA_OUT_FLASH_EN:
			setbits32(immap + GPIO_ADDR_OFFSET + GPIO_DAT, mpc8xxx_gpio2mask(CPU_OUT_FLASH_EN));
			break;
		default:
			break;
	}
	
	mutex_unlock(&ioctl_lock);

	return retval;
}

static int panel_io_open(struct inode *inode, struct file *filp)
{
	return 0;
}
static int panel_io_release(struct inode *inode, struct file *filp)
{
   return 0;
}

static struct file_operations panel_io_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = panel_io_ioctl,
	.open =		panel_io_open,
	.release =	panel_io_release,
};

static int __init panel_io_init(void)
{
	int status =0;
	int d;
	
	status = register_chrdev(PANEL_IO_DEV_MAJOR, DRIVER_NAME, &panel_io_fops);
	if( status <  0)
	{
		printk("stauts = %d \n", status);
		return status;
	}

	mutex_init(&ioctl_lock);
	
	immap = ioremap(get_immrbase(), 0xd00);
	if(immap)
	{
	    // set GPIO_SEL = 1
	    setbits32(immap + SICRH_ADDR_OFFSET, mpc8xxx_gpio2mask(GPIO_SEL));
	    // set GTM = 11, IEEE1588_B = 11
	    d = mpc8xxx_gpio2mask(IEEE1588_A_10) | mpc8xxx_gpio2mask(IEEE1588_A_11) | mpc8xxx_gpio2mask(IEEE1588_B_16) \
	    | mpc8xxx_gpio2mask(IEEE1588_B_17);
	    setbits32(immap + SICRH_ADDR_OFFSET, d);
	    
	    d = mpc8xxx_gpio2mask(Reset_s) | mpc8xxx_gpio2mask(Reset_phy) | mpc8xxx_gpio2mask(nFPGA_RESET) \
	    | mpc8xxx_gpio2mask(SPI_CS_FPGA) | mpc8xxx_gpio2mask(SPI_CS_Flash) | mpc8xxx_gpio2mask(CPU_OUT_FLASH_EN);
	    // set GPIO_1,3,4,5,12,14 as output
		setbits32(immap + GPIO_ADDR_OFFSET + GPIO_DIR, d);
		// set GPIO_1,3,4,5,12,14 is actively driven as an output
		clrbits32(immap + GPIO_ADDR_OFFSET + GPIO_ODR, d);
		// set GPIO_1,3,4,5,12,14 to high level
		setbits32(immap + GPIO_ADDR_OFFSET + GPIO_DAT, d);
	}
	else
	{
		status = -1; //return value
		iounmap(immap);
	}

	return status;
}
static void __exit panel_io_exit(void)
{
   	unregister_chrdev(PANEL_IO_DEV_MAJOR, DRIVER_NAME);
	iounmap(immap);
}
module_init(panel_io_init);
module_exit(panel_io_exit);
MODULE_LICENSE("GPL");
