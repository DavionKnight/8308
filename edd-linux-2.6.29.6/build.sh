#!/bin/sh
export ARCH=powerpc
#make menuconfig
export PATH=/opt/eldk42/usr/bin:/opt/eldk42/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

#make distclean
#make MPC8308EDD_NAND_config
#make clean

make all
make uImage
make mpc8308edd.dtb

rm ../mkubi/bin/edd.dtb
rm ../mkubi/bin/edd_uImage

cp ./arch/powerpc/boot/mpc8308edd.dtb ../mkubi/bin/edd.dtb
cp ./arch/powerpc/boot/uImage ../mkubi/bin/edd_uImage

