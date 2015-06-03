#ifndef _OAM_FPGA_H_
#define _OAM_FPGA_H_

//#include <linux/config.h>
#include <asm/ioctl.h>

/* ioctl's */
#define STATUS_OAM_FPGA_GET		_IOR('L', 1, unsigned long)
#define STATUS_OAM_FPGA_SET		_IOW('L', 2, unsigned long)

#define SPI_WrEn		0x0030
#define SPI_RdEn		0x0031
#define SPI_Addr		0x0032
#define SPI_WrData		0x0034
#define SPI_Idle		0x003C
#define SPI_Size		0x003D
#define SPI_RdData		0x0048

#define spi_is_idle()		do { fpga_spi_read(SPI_Idle, 1, &tmp); } while ( !(tmp & 0x0001) );
#define spi_wr_addr(addr)	{tmp = (addr); fpga_spi_write(SPI_Addr, 1, &tmp);}
#define spi_wr_size(size)	{tmp = size; fpga_spi_write(SPI_Size, 1, &tmp);}
#define spi_enable(En)		{fpga_spi_read(En, 1, &tmp); tmp ^= 0x0001; fpga_spi_write(En, 1, &tmp);}

#define uint8 unsigned char

typedef struct
{
	uint8 cid;
	uint8 addr;
	uint8 buf[8];
	uint len;
}oam_fpga_reg;

#undef uchar
#define uchar			unsigned char
#undef ushort
#define ushort			unsigned short
#undef uint
#define uint			unsigned int

#endif
