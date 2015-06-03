#!/bin/sh

export PATH=/home/freddie/Compiler_PowerPC/usr/bin:/home/freddie/Compiler_PowerPC/bin:$PATH
mkimage -A ppc -O linux -T script -C none -a 0 -e 0 -n "set envs but address script" -d ./set_env.script ./set_env.uscr
cp ./set_env.uscr /home/freddie/Documents/TFTP/CFG_PowerPC/
