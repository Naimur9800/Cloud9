#!/bin/bash
# v1.0
# RX Kernel build script

DEFCONFIG=v3702_defconfig
export KBUILD_BUILD_USER=Parthib-K24
export KBUILD_BUILD_HOST=travis-gccTestServer

mkdir -p out
export ARCH=arm ARCH_MTK_PLATFORM=mt6580
make -C $PWD O=$PWD/out ARCH=arm $DEFCONFIG
make -j4 -C $PWD O=$PWD/out ARCH=arm
