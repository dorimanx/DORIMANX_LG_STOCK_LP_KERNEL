#!/bin/sh

PYTHON_CHECK=$(ls -la /usr/bin/python | grep python3 | wc -l);
PYTHON_WAS_3=0;

if [ "$PYTHON_CHECK" -eq "1" ] && [ -e /usr/bin/python2 ]; then
	if [ -e /usr/bin/python2 ]; then
		rm /usr/bin/python
		ln -s /usr/bin/python2 /usr/bin/python
		echo "Switched to Python2 for building kernel will switch back when done";
		PYTHON_WAS_3=1;
	else
		echo "You need Python2 to building this kernel. install and come back."
		exit 1;
	fi;
else
	echo "Python2 is used! all good, building!";
fi;

export PATH=$PATH:tools/lz4demo
make ARCH=arm CROSS_COMPILE=/android-kernel/android_source/GOOGLE/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi- d802_defconfig zImage -j16

if [ "$PYTHON_WAS_3" -eq "1" ]; then
	rm /usr/bin/python
	ln -s /usr/bin/python3 /usr/bin/python
fi;
