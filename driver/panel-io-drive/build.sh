#!/bin/sh

export ARCH=powerpc
export PATH=/home/freddie/Compiler_PowerPC/usr/bin:/home/freddie/Compiler_PowerPC/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

make clean
rm /home/freddie/Documents/TFTP/CFG_PowerPC/panel_io_driver.ko
rm /home/freddie/Documents/TFTP/CFG_PowerPC/test_panel
make

ppc_82xx-gcc test_panel.c -o test_panel
ppc_82xx-gcc spidev_test.c -o spidev_test

cp test_panel /home/freddie/Documents/TFTP/CFG_PowerPC/
cp panel_io.sh /home/freddie/Documents/TFTP/CFG_PowerPC/
cp spidev_test /home/freddie/Documents/TFTP/CFG_PowerPC/
