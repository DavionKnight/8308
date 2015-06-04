#!/bin/sh

export ARCH=powerpc
export PATH=/opt/eldk42/usr/bin:/opt/eldk42/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

make clean
rm /tftpboot/spi_drv.ko
rm /tftpboot/spi_drv
make

ppc_82xx-gcc main.c -o spi_drv

cp spi_drv /tftpboot/
cp spi_drv.ko /tftpboot/
