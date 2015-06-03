#!/bin/sh

export PATH=/home/freddie/Compiler_PowerPC/usr/bin:/home/freddie/Compiler_PowerPC/bin:$PATH
rm *.gz
cp ramdisk_tmp ramdisk
gzip -f9 ramdisk
mkimage -A ppc -O linux -T ramdisk -C gzip -n "Linux ramdisk" -d ramdisk.gz uRamdisk.gz
rm ../mkubi/bin/edd_uRamdisk
cp uRamdisk.gz ../mkubi/bin/edd_uRamdisk
