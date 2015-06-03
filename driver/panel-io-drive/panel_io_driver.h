#ifndef  PANEL_IO_DRIVER_H
#define  PANEL_IO_DRIVER_H

#include <linux/types.h>

#define PANEL_IO_MAGIC			'p'

#define  GPIO_SEL               23
#define  Reset_s                4
#define  Reset_phy              5
#define  nFPGA_RESET            14
#define  CPU_OUT_FLASH_EN		12

#define  IEEE1588_A_10			10
#define  IEEE1588_A_11			11
#define  IEEE1588_B_16			16
#define  IEEE1588_B_17			17
#define  SPI_CS_FPGA			1
#define  SPI_CS_Flash			3

//reset BCM53284
#define BCM53284_RESET			_IO(PANEL_IO_MAGIC, 1)

//reset BCM5482S
#define BCM5482S_RESET			_IO(PANEL_IO_MAGIC, 2)

//reset FPGA
#define FPGA_RESET		    	_IO(PANEL_IO_MAGIC, 3)

//Flash access cs
#define MPC_OUT_FLASH_EN		_IO(PANEL_IO_MAGIC, 4)
#define FPGA_OUT_FLASH_EN		_IO(PANEL_IO_MAGIC, 5)

#endif
