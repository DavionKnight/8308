#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "panel_io_driver.h"

static const char *device = "/dev/panel_io";  //don't change the name

int main(void)
{
	int fd, ret;
	printf("opening /dev/panel_io\n");
	fd = open(device, O_RDWR);
	if(fd < 0)
		printf("can't open device\n");
	
	ret = ioctl(fd, BCM53284_RESET, 0);
	if (ret == -1)
		printf("Reset BCM53284 failed\n");
		
	ret = ioctl(fd, BCM5482S_RESET, 0);
	if (ret == -1)
		printf("Reset BCM5482S failed\n");
		
	ret = ioctl(fd, FPGA_RESET, 0);
	if (ret == -1)
		printf("Reset FPGA failed\n");
		
	printf("Reset completed\n");
	
	ret = ioctl(fd, FPGA_OUT_FLASH_EN, 0);
	if (ret == -1)
		printf("choice Flash failed\n");
	
	close(fd);
	return 0;
}
