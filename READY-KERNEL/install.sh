#!/sbin/sh

if [ -e /system/app/STweaks.apk ]; then
	rm /system/app/STweaks.apk;
fi;
if [ -e /system/priv-app/STweaks.apk ]; then
	rm /system/priv-app/STweaks.apk;
fi;
if [ -e /system/app/re.codefi.savoca.kcal-v1.1.apk ]; then
	rm /system/app/re.codefi.savoca.kcal-v1.1.apk;
fi;

dd if=/tmp/boot.img of=/dev/block/platform/msm_sdcc.1/by-name/boot || exit 1
exit 0
