#!/bin/bash

export ARCH=powerpc
export PATH=/home/freddie/Compiler_PowerPC/usr/bin:/home/freddie/Compiler_PowerPC/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

make clean
make distclean
make MPC8308EDD_NAND_config

if [ "$1" == "flash" ];then
rm /home/freddie/Documents/TFTP/CFG_PowerPC/u-boot-nand.bin
make all
cp ./u-boot-nand.bin /home/freddie/Documents/TFTP/CFG_PowerPC/
echo "boot from flash"
elif [ "$1" == "ram" ];then
rm /home/freddie/Documents/TFTP/CFG_PowerPC/u-boot.bin
make all
cp ./u-boot.bin /home/freddie/Documents/TFTP/CFG_PowerPC/
echo "boot from ram"
else
echo "error"
fi
