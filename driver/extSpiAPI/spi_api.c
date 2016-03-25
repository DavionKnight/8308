/**********************************************
 * @file	bcm53101.c
 * @author	zhangjj <zhangjj@bjhuahuan.com>
 * @date	2015-10-22
 *********************************************/

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <string.h>

#define SPI_R   0
#define SPI_W   1

typedef struct fpga_rw_t{
        unsigned short addr;
        unsigned short size;
        unsigned short buf[20];
}fpga_rw;


void pdata(unsigned char *pdata, int count)
{
	int i;
	for (i = 0; i < count; i++) {
		printf(" %02x", pdata[i]);
	}
	printf("\n");
}

int main(int argc, char *argv[])
{
	int ret = 0, fpfd = 0;
	fpga_rw fpopt;

	fpfd = open( "/dev/spiapi", O_RDWR);
	if(fpfd == -1 )
		return -1;

	fpopt.addr = 0;
	fpopt.size = 1;
	memset(fpopt.buf, 0, sizeof(fpopt.buf));

	ioctl(fpfd, SPI_R, &fpopt);

	printf("result=0x%x\n",fpopt.buf[0]);	

	close(fpfd);

	return 0;	
}



