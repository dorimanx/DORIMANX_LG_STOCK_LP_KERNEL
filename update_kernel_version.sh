#!/bin/sh

V1=$1
V2=$2

if [ "$V1" -ge "0" ] && [ "a$V2" != "a" ]; then
	sed -i "s/-Kernel-Dorimanx-V[0-9][0-9].[0-9]-LG-/-Kernel-Dorimanx-V${V1}.${V2}-LG-/g" arch/arm/configs/dorimanx_d800_defconfig
	sed -i "s/-Kernel-Dorimanx-V[0-9][0-9].[0-9]-LG-/-Kernel-Dorimanx-V${V1}.${V2}-LG-/g" arch/arm/configs/dorimanx_d801_defconfig
	sed -i "s/-Kernel-Dorimanx-V[0-9][0-9].[0-9]-LG-/-Kernel-Dorimanx-V${V1}.${V2}-LG-/g" arch/arm/configs/dorimanx_d802_defconfig
	sed -i "s/-Kernel-Dorimanx-V[0-9][0-9].[0-9]-LG-/-Kernel-Dorimanx-V${V1}.${V2}-LG-/g" arch/arm/configs/dorimanx_ls980_defconfig
	sed -i "s/-Kernel-Dorimanx-V[0-9][0-9].[0-9]-LG-/-Kernel-Dorimanx-V${V1}.${V2}-LG-/g" arch/arm/configs/dorimanx_vs980_defconfig
else
	echo "please input new kernel version with space, sh update_kernel_version.sh 8 8 or other numbers."
fi;
