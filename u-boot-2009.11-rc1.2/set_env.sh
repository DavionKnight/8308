#!/bin/sh

export PATH=/home/kevin/Documents/ppc-tools/usr/bin:/home/kevin/Documents/ppc-tools/bin:$PATH
mkimage -A ppc -O linux -T script -C none -a 0 -e 0 -n "set envs but address script" -d ./set_env.script ./set_env.uscr
cp ./set_env.uscr /home/freddie/Documents/TFTP/CFG_PowerPC/
