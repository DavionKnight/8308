/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include "panel_io_driver.h"

#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))
#define MULTI_REG_LEN_MAX	12
#define SPI_FPGA_WR_SINGLE	0x01
#define SPI_FPGA_WR_BURST	0x02
#define SPI_FPGA_RD_BURST	0x03
#define SPI_FPGA_RD_SINGLE	0x05

/* Read / Write SPI chip select number */
#define SPI_IOC_RD_CHIP_SLECT_NUM		_IOR(SPI_IOC_MAGIC, 5, __u8)
#define SPI_IOC_WR_CHIP_SLECT_NUM		_IOW(SPI_IOC_MAGIC, 5, __u8)

static const char *device = "/dev/spidev28672.0";

static uint8_t mode = 3;
static uint8_t bits = 8;
static uint32_t speed = 4000000;
static uint16_t delay = 0;

static int fd;

typedef struct
{
	unsigned short addr;
	unsigned short size;
	unsigned short pbuf[MULTI_REG_LEN_MAX];
}spi_fpga_reg;

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static void fpga_spi_read(spi_fpga_reg *preg)
{
	int ret, i;
	uint8_t tx_buf[sizeof(short) * MULTI_REG_LEN_MAX + 3] = {0};
	uint8_t rx_buf[sizeof(short) * MULTI_REG_LEN_MAX + 3] = {0};
	
	tx_buf[0] = SPI_FPGA_RD_BURST;
	tx_buf[1] = (unsigned char)((preg->addr >> 7) & 0xff);
	tx_buf[2] = (unsigned char)((preg->addr << 1) & 0xff);
	
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_buf,
		.rx_buf = (unsigned long)rx_buf,
		.len = sizeof(short) * preg->size + 3,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == -1)
		pabort("can't send spi message");

	//for(i = 0; i < (sizeof(short) * MULTI_REG_LEN_MAX + 3); i++)
		//printf("rx_buf[%d] = 0x%02X\n", i, rx_buf[i]);

	for(i = 0; i < preg->size; i++)
		preg->pbuf[i] = ((unsigned short)rx_buf[3 + i*2] << 8) | ((unsigned short)rx_buf[4 + i*2]);
	
	return;
}

static void fpga_spi_write(spi_fpga_reg *preg)
{
	int ret, i;
	uint8_t tx_buf[sizeof(short) * MULTI_REG_LEN_MAX + 3] = {0};
	
	tx_buf[0] = SPI_FPGA_WR_BURST;
	tx_buf[1] = (unsigned char)((preg->addr >> 7) & 0xff);
	tx_buf[2] = (unsigned char)((preg->addr << 1) & 0xff);
	
	for(i = 0; i < preg->size; i++)
	{
		tx_buf[3 + i*2] = (unsigned char)((preg->pbuf[i] >> 8) & 0xff);
		tx_buf[4 + i*2] = (unsigned char)((preg->pbuf[i]) & 0xff);
	}

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_buf,
		.rx_buf = (unsigned long)NULL,
		.len = sizeof(short) * preg->size + 3,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	//for(i = 0; i < (sizeof(short) * MULTI_REG_LEN_MAX + 3); i++)
		//printf("tx_buf[%d] = 0x%02X\n", i, tx_buf[i]);

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == -1)
		pabort("can't send spi message");

	return;
}

int main(int argc, char *argv[])
{
	spi_fpga_reg reg;
	int	i, data, ret, flag = 1;
	uint8_t select_id;

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open SPI device\n");

	select_id = 0;
	
	ret = ioctl(fd, SPI_IOC_WR_CHIP_SLECT_NUM, &select_id);
	if (ret == -1)
		pabort("can't set spi select ID");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	memset(&reg, 0, sizeof(reg));

	while( flag )
   	{
   		printf( "\n\t read[1] or write[2] or test[3] or quit[0]: " );
    	scanf( "%d", &data );
    	
   		switch( data )
   		{
   			case 0:
	    		flag = 0;
				break;
    		case 1:
		    	printf( "\t\t input the addr (hex): " );
		    	scanf( "%x", &data );
				reg.addr = data;
				printf( "\t\t input the size (hex): " );
				scanf( "%x", &data );
				reg.size = data;
				if( reg.size > MULTI_REG_LEN_MAX )
				{
					printf( "\n\t\t The size must be less than %d\n", sizeof(reg.pbuf) ); 
					flag = 0;
				}
				else
				{
					printf( "\t\t read the register on the %02X:%02X\n\t\t", reg.addr, reg.size);
					fpga_spi_read(&reg);
					for( i = 0; i < reg.size; i++ )
						printf( "0x%04X ", reg.pbuf[i]);
					printf( "\n" );
    			}
    		break;
    		case 2:
		    	printf( "\t\t input the addr (hex): " );
		    	scanf( "%x", &data );
				reg.addr = data;
				printf( "\t\t input the size (hex): " );
				scanf( "%x", &data );
				reg.size	= data;
				if( reg.size > MULTI_REG_LEN_MAX )
				{
					printf( "\n\t\t The size must be less than %d\n", sizeof(reg.pbuf) ); 
					flag	= 0;
				}
				else
				{
					for( i = 0; i < reg.size; i++ )
					{
						printf( "\t\t input number_%d (hex): ", i );
						scanf( "%x", &data );
						reg.pbuf[i]	= data;
					}
					
					printf( "\t\t write the register on the %02X:%02X\n\t\t ", reg.addr, reg.size );
					for( i = 0; i < reg.size; i++ )
						printf( "0x%04X ", reg.pbuf[i] );
					printf( "\n" );
						
					fpga_spi_write(&reg);
				}
    		break;
    		default:
				while(1)
				{
					fpga_spi_read(&reg);
					for( i = 0; i < reg.size; i++ )
						printf( "0x%04X ", reg.pbuf[i]);
					printf( "\n" );
				}
    		break;
    	}
	}

	close(fd);

	return 0;
}
