#!/bin/sh
clear

# What you need installed to compile
# gcc, gpp, cpp, c++, g++, lzma, lzop, ia32-libs flex

# What you need to make configuration easier by using xconfig
# qt4-dev, qmake-qt4, pkg-config

# Setting the toolchain
# the kernel/Makefile CROSS_COMPILE variable to match the download location of the
# bin/ folder of your toolchain
# toolchain already axist and set! in kernel git. android-toolchain/bin/

# Structure for building and using this script

#--project/				(progect container folder)
#------lp_ramdisk_dorimanx/		(ramdisk files for boot.img)
#------ramdisk-lp-tmp/			(ramdisk tmp store without .git)
#--------lib/modules/			(modules dir, will be added to system on boot)
#------DORIMANX_LG_STOCK_LP_KERNEL/	(kernel source goes here)
#--------READY-RELEASES/		(When using all selector, all models ready kernels will go to this folder)
#--------READY-KERNEL/			(output directory, where the final boot.img is placed)
#----------meta-inf/			(meta-inf folder for your flashable zip)
#----------system/

# location
KERNELDIR=$(readlink -f .);
export PATH=$PATH:tools/lz4demo

CLEANUP()
{
	# begin by ensuring the required directory structure is complete, and empty
	echo "Initialising................."
	rm -rf "$KERNELDIR"/READY-KERNEL/boot
	rm -f "$KERNELDIR"/READY-KERNEL/*.zip
	rm -f "$KERNELDIR"/READY-KERNEL/*.img
	mkdir -p "$KERNELDIR"/READY-KERNEL/boot

	if [ -d ../ramdisk-lp-tmp ]; then
		rm -rf ../ramdisk-lp-tmp/*
	else
		mkdir ../ramdisk-lp-tmp
		chown root:root ../ramdisk-lp-tmp
		chmod 777 ../ramdisk-lp-tmp
	fi;

	# force regeneration of .dtb and zImage files for every compile
	rm -f arch/arm/boot/*.dtb
	rm -f arch/arm/boot/*.cmd
	rm -f arch/arm/boot/zImage
	rm -f arch/arm/boot/Image
}
CLEANUP;


BUILD_NOW()
{
	PYTHON_CHECK=$(ls -la /usr/bin/python | grep python3 | wc -l);
	PYTHON_WAS_3=0;

	if [ "$PYTHON_CHECK" -eq "1" ] && [ -e /usr/bin/python2 ]; then
		if [ -e /usr/bin/python2 ]; then
			rm /usr/bin/python
			ln -s /usr/bin/python2 /usr/bin/python
			echo "Switched to Python2 for building kernel will switch back when done";
			PYTHON_WAS_3=1;
		else
			echo "You need Python2 to build this kernel. install and come back."
			exit 1;
		fi;
	else
		echo "Python2 is used! all good, building!";
	fi;

	# get version from config
	GETVER=$(grep 'Kernel-.*-V' .config |sed 's/Kernel-//g' | sed 's/.*".//g' | sed 's/-L.*//g');
	GETBRANCH=$(grep '.*-LG' .config |sed 's/Kernel-Dorimanx-V//g' | sed 's/[1-9].*-LG-//g' | sed 's/.*".//g' | sed 's/-PWR.*//g');

	# remove all old modules before compile
	for i in $(find "$KERNELDIR"/ -name "*.ko"); do
		rm -f "$i";
	done;

	# Copy needed dtc binary to system to finish the build.
	if [ ! -e /bin/dtc ]; then
		cp -a tools/dtc-binary/dtc /bin/;
	fi;

	# Idea by savoca
	NR_CPUS=$(grep -c ^processor /proc/cpuinfo)

	if [ "$NR_CPUS" -le "2" ]; then
		NR_CPUS=4;
		echo "Building kernel with 4 CPU threads";
	else
		echo "Building kernel with $NR_CPUS CPU threads";
	fi;

	if [ ! -e .config ]; then
		cp arch/arm/configs/d802_defconfig ./.config;
	fi;

	# build zImage
	time make ARCH=arm CROSS_COMPILE=/android-kernel/android_source/GOOGLE/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi- zImage-dtb -j ${NR_CPUS}

	cp .config arch/arm/configs/d802_defconfig

	stat "$KERNELDIR"/arch/arm/boot/zImage || exit 1;

	# compile the modules, and depmod to create the final zImage
	echo "Compiling Modules............"
	time make ARCH=arm CROSS_COMPILE=/android-kernel/android_source/GOOGLE/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi- modules -j ${NR_CPUS} || exit 1

	# move the compiled zImage and modules into the READY-KERNEL working directory
	echo "Move compiled objects........"

	# Check that RAMDISK is for this Branch. and check if need to push changes before switch to needed branch.

	# copy all ROOT ramdisk files to ramdisk temp dir.
	cp -a ../lp_ramdisk_dorimanx/* ../ramdisk-lp-tmp/

	for i in $(find "$KERNELDIR" -name '*.ko'); do
		cp -av "$i" ../ramdisk-lp-tmp/lib/modules/;
	done;

	chmod 755 ../ramdisk-lp-tmp/lib/modules/*

	# remove empty directory placeholders from tmp-initramfs
	for i in $(find ../ramdisk-lp-tmp/ -name EMPTY_DIRECTORY); do
		rm -f "$i";
	done;

	if [ -e "$KERNELDIR"/arch/arm/boot/zImage ]; then

		if [ ! -d READY-KERNEL/boot ]; then
			mkdir READY-KERNEL/boot
		fi;

		cp arch/arm/boot/zImage READY-KERNEL/boot/

		# strip not needed debugs from modules.
		/android-kernel/android_source/GOOGLE/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi-strip --strip-unneeded ../ramdisk-lp-tmp/lib/modules/* 2>/dev/null
		/android-kernel/android_source/GOOGLE/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi-strip --strip-debug ../ramdisk-lp-tmp/lib/modules/* 2>/dev/null

		# create the ramdisk and move it to the output working directory
		echo "Create ramdisk..............."
		scripts/mkbootfs ../ramdisk-lp-tmp | gzip > ramdisk.gz 2>/dev/null
		mv ramdisk.gz READY-KERNEL/boot/

		# create the dt.img from the compiled device files, necessary for msm8974 boot images
		echo "Create dt.img................"
		./scripts/dtbTool -v -s 2048 -o READY-KERNEL/boot/dt.img arch/arm/boot/

		if [ "$PYTHON_WAS_3" -eq "1" ]; then
			rm /usr/bin/python
			ln -s /usr/bin/python3 /usr/bin/python
		fi;

		# add kernel config to kernle zip for other devs
		cp "$KERNELDIR"/.config READY-KERNEL/

		# build the final boot.img ready for inclusion in flashable zip
		echo "Build boot.img..............."
		cp scripts/mkbootimg READY-KERNEL/boot/
		cd READY-KERNEL/boot
		base=0x00000000
		offset=0x05000000
		tags_addr=0x00000100
		cmd_line="console=ttyHSL0,115200,n8 user_debug=31 ehci-hcd.park=3 msm_rtb.filter=0x0 androidboot.hardware=g2"
		./mkbootimg --kernel zImage --ramdisk ramdisk.gz --cmdline "$cmd_line" --base $base --pagesize 2048 --offset $offset --tags-addr $tags_addr --dt dt.img -o newboot.img
		mv newboot.img ../boot.img

		# cleanup all temporary working files
		echo "Post build cleanup..........."
		cd ..
		rm -rf boot

		# BUMP boot.img with magic key to install on JB/KK bootloader
		cd ..
		sh kernel_bump.sh
		mv READY-KERNEL/boot_bumped.img READY-KERNEL/boot.img
		echo "Kernel BUMP done!";
		cd READY-KERNEL/

		# create the flashable zip file from the contents of the output directory
		echo "Make flashable zip..........."
		zip -r Kernel-"${GETVER}"-KK-"$(date +"[%H-%M]-[%d-%m]-LG-${GETBRANCH}-PWR-CORE")".zip * >/dev/null
		stat boot.img
		rm -f ./*.img
		cd ..
	else
		if [ "$PYTHON_WAS_3" -eq "1" ]; then
			rm /usr/bin/python
			ln -s /usr/bin/python3 /usr/bin/python
		fi;

		# with red-color
		echo -e "\e[1;31mKernel STUCK in BUILD! no zImage exist\e[m"
	fi;
}
BUILD_NOW;
