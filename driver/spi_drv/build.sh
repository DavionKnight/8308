#!/bin/sh

export ARCH=powerpc
export PATH=/home/freddie/Compiler_PowerPC/usr/bin:/home/freddie/Compiler_PowerPC/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

make clean
rm /home/freddie/Documents/TFTP/CFG_PowerPC/spi_drv.ko
rm /home/freddie/Documents/TFTP/CFG_PowerPC/spi_drv
make

ppc_82xx-gcc main.c -o spi_drv

cp spi_drv /home/freddie/Documents/TFTP/CFG_PowerPC
cp spi_drv.ko /home/freddie/Documents/TFTP/CFG_PowerPC
