#!/bin/sh
insmod spi_drv.ko
mknod /dev/spi_drv c 207 0


