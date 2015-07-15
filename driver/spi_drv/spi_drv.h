#ifndef _OAM_FPGA_H_
#define _OAM_FPGA_H_


//#include <linux/config.h>
#include <asm/ioctl.h>

/* ioctl's */
#define STATUS_SPI_GET		_IOR('L', 1, unsigned long)
#define STATUS_SPI_SET		_IOW('L', 2, unsigned long)

#define MULTI_REG_LEN_MAX	5



typedef struct
{
	unsigned short	addr;
	unsigned short	size;
	unsigned char	pbuf[MULTI_REG_LEN_MAX];
}spi_reg;

#undef uchar
#define uchar			unsigned char
#undef ushort
#define ushort			unsigned short
#undef uint
#define uint			unsigned int

#endif
