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


/* Read / Write SPI chip select number */
#define SPI_IOC_RD_CHIP_SLECT_NUM		_IOR(SPI_IOC_MAGIC, 5, __u8)
#define SPI_IOC_WR_CHIP_SLECT_NUM		_IOW(SPI_IOC_MAGIC, 5, __u8)

#define W25_ERASE_CHIP			_IOW(SPI_IOC_MAGIC, 6 , __u8)
#define W25_ERASE_SECTOR		_IOW(SPI_IOC_MAGIC, 7 , __u32)
#define W25P16_READ				_IOR(SPI_IOC_MAGIC, 8 , __u32)
#define W25P16_WRITE			_IOW(SPI_IOC_MAGIC, 8 , __u32)
typedef struct{
	loff_t addr;
	size_t len;
	u_char buf[256];
}w25_rw_date_t;


static const char *device = "/dev/spidev28672.0";
static uint8_t mode=3;
static uint8_t bits = 8;
static uint32_t speed = 200000;
//static uint16_t delay = 20;

#define LEN		0x100
#define ADDR	0x1000



int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;
	
	uint8_t  chip_select = 1;

	w25_rw_date_t  w25p16_date;
	unsigned int sector_addr = ADDR;
	int i;

	fd = open(device, O_RDWR | O_SYNC | O_DSYNC | O_RSYNC);
	if (fd < 0)
		printf("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		printf("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		printf("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printf("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printf("can't get max speed hz");

	chip_select = 1;
	ret = ioctl(fd, SPI_IOC_WR_CHIP_SLECT_NUM, &chip_select);
	if (ret == -1)
		printf("cs failed");

//**********************************************************************first read++++++++++++++++++++++++

	w25p16_date.addr = ADDR;
	w25p16_date.len =  LEN;
	memset(w25p16_date.buf, 0x00, sizeof(w25p16_date.buf));//clear

	ret = ioctl(fd, W25P16_READ, (unsigned long)&w25p16_date);
	if (ret == -1)
		printf("after erase read FAILD");
    printf("\n");
	printf("\n+++++++++++++++++++++++++++++++before earse:++++++++++++++++++++++++++++++++++++++++++++++\n");
	for( i = 0; i <  LEN; i++)
	{
		if( (i %8) == 0)
			printf("\n");
		printf(" buf[0x%02x] = 0x%02x", i, w25p16_date.buf[i]);
	}
#if 0
	ret = ioctl(fd,  W25_ERASE_CHIP, NULL);
	if (ret == -1)
		printf("ERASE FAILD");
	
	w25p16_date.addr = ADDR;
	w25p16_date.len =  LEN;
	memset(w25p16_date.buf, 0x00, sizeof(w25p16_date.buf));//clear
	ret = ioctl(fd, W25P16_READ, (unsigned long)&w25p16_date);
	if (ret == -1)
		printf("after erase read FAILD");
    printf("\n");
    printf("\n+++++++++++++++++++++++++++++++after earse:++++++++++++++++++++++++++++++++++++++++++++++\n");
	for( i = 0; i <  LEN; i++)
	{
		if( (i %8) == 0)
			printf("\n");
		printf(" buf[0x%02x] = 0x%02x", i, w25p16_date.buf[i]);
	}

	for(i = 0; i <  LEN; i++)
		w25p16_date.buf[i] = i;

	ret = ioctl(fd, W25P16_WRITE, (unsigned long)&w25p16_date);
	if (ret == -1)
		printf("write FAILD");

	memset(w25p16_date.buf, 0x00, sizeof(w25p16_date.buf));//clear

	ret = ioctl(fd, W25P16_READ, (unsigned long)&w25p16_date);
	if (ret == -1)
		printf("after erase read FAILD");
	printf("\n");
    printf("\n+++++++++++++++++++++++++++++++after program:++++++++++++++++++++++++++++++++++++++++++++++\n");
	for( i = 0; i < LEN; i++)
	{
		if( (i %8) == 0)
			printf("\n");
		printf(" buf[0x%02x] = 0x%02x", i, w25p16_date.buf[i]);
	}
#endif
	printf("\n");
	close(fd);

	return ret;
}
