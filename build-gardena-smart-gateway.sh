#!/bin/bash

DEFCONFIG=gardena-smart-gateway_defconfig
BOARD=gardena_smart_gateway_mt7688
DESTDIR=/tftpboot/gardena
DTB=ralink/$BOARD.dtb
ARCH_USED=mips

if [ "$ARCH" != "$ARCH_USED"  ]
then
    echo Wrong cross toolchain configured!
    echo Please use \"mips\" to configure the toochain
    exit 0
fi

if [ "$1" == "savedefconfig"  ]
then
    echo make savedefconfig
    make savedefconfig
    cp defconfig arch/$ARCH/configs/$DEFCONFIG
    exit 0
fi

if [ "$1" == "config"  ]
then
    echo make mrproper
    make mrproper

    echo make $DEFCONFIG ...
    make $DEFCONFIG
    echo ... done

    sleep 1
fi

# build the image (vmlinux.bin) and dtb
make -j$BUILD_CPUS vmlinux.bin
make -j$BUILD_CPUS $DTB

# generate FIT image and copy it to the dest directoy
mkimage -f $BOARD.its $BOARD.itb
cp $BOARD.itb $DESTDIR
