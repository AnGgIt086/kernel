#!/bin/bash
PATH=`pwd`/clang_tool/clang-r383902/bin:`pwd`/aarch64-linux-android-4.9/bin:$PATH
rm -rf out
mkdir -p out
make -j32 O=out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- CC=clang CLANG_TRIPLE=arm-linux-gnueabi- LD=ld.lld k65v1_64_bsp_defconfig
make -j32 O=out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- CC=clang CLANG_TRIPLE=arm-linux-gnueabi- LD=ld.lld
