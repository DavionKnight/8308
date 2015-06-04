#!/bin/bash

export ARCH=powerpc
export PATH=/opt/eldk42/usr/bin:/opt/eldk42/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

make clean
make distclean
make MPC8308EDD_NAND_config

if [ "$1" == "flash" ];then
rm /tftpboot/u-boot-nand.bin
make all
echo "cp ./u-boot-nand.bin /tftpboot/"
cp ./u-boot-nand.bin /tftpboot/
echo "boot from flash"
elif [ "$1" == "ram" ];then
rm /tftpboot/u-boot.bin
make all
echo "cp ./u-boot.bin /tftpboot/"
cp ./u-boot.bin /tftpboot/
echo "boot from ram"
else
echo "error"
fi
