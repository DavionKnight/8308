#!/bin/sh

export ARCH=powerpc
export PATH=/opt/eldk42/usr/bin:/opt/eldk42/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

make clean
rm /tftpboot/oam_fpga_drv.ko

make
cp oam_fpga_drv.ko /tftpboot/

ppc_82xx-gcc main.c -o oam_drv_test
cp oam_drv_test /tftpboot/
