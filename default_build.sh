#!/bin/sh

export PATH=$PATH:tools/lz4demo
make ARCH=arm CROSS_COMPILE=/android-kernel/android_source/GOOGLE/prebuilts/gcc/linux-x86/arm/arm-eabi-4.7/bin/arm-eabi- g2-open_com-perf_defconfig zImage -j16

