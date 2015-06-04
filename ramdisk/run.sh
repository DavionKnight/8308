#!/bin/sh

export PATH=/opt/eldk42/usr/bin:/opt/eldk42/bin:$PATH
rm *.gz
cp ramdisk_tmp ramdisk
gzip -f9 ramdisk
mkimage -A ppc -O linux -T ramdisk -C gzip -n "Linux ramdisk" -d ramdisk.gz uRamdisk.gz
rm ../mkubi/bin/edd_uRamdisk
cp uRamdisk.gz ../mkubi/bin/edd_uRamdisk
