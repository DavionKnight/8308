#!/bin/sh
export ARCH=powerpc
#make menuconfig
#export PATH=/opt/eldk42/usr/bin:/opt/eldk42/bin:$PATH
export PATH=/home/kevin/Documents/ppc-tools/usr/bin:/home/kevin/Documents/ppc-tools/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

#make distclean
#make MPC8308EDD_NAND_config
#make clean

make all

cp spi_fpga.ko /tftpboot/

