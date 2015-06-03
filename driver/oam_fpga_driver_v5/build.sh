#!/bin/sh

export ARCH=powerpc
export PATH=/home/freddie/Compiler_PowerPC/usr/bin:/home/freddie/Compiler_PowerPC/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

make clean
rm /home/freddie/Documents/TFTP/CFG_PowerPC/oam_fpga_drv.ko

make
cp oam_fpga_drv.ko /home/freddie/Documents/TFTP/CFG_PowerPC/

ppc_82xx-gcc main.c -o oam_drv_test
cp oam_drv_test /home/freddie/Documents/TFTP/CFG_PowerPC/
