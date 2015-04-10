#MAKE Directory
mkdir -p ../../codeRule/kernel
mkdir -p ../../codeRule/usb
mkdir -p ../../codeRule/fs
mkdir -p ../../codeRule/lcd
mkdir -p ../../codeRule/input
mkdir -p ../../codeRule/power
mkdir -p ../../codeRule/modem
mkdir -p ../../codeRule/sound

#USB
./scripts/checkfiles -x drivers/usb/gadget/ | tee -a gadget.log
./scripts/checkfiles -x drivers/usb/dwc3t/ | tee -a dwc3.log
mv gadget.log ../../codeRule/usb/
mv dwc3.log ../../codeRule/usb/

#KERNEL
./scripts/checkfiles -x arch/arm/mach-msm/ | tee -a mach-msm.log
./scripts/checkfiles -x arch/arm/mach-msm/lge | tee -a mach-msm_lge.log
./scripts/checkfiles -x include/linux/ | tee -a include_linux.log
mv mach-msm.log ../../codeRule/kernel/
mv mach-msm_lge.log ../../codeRule/kernel/
mv include_linux.log ../../codeRule/kernel/

#FS
./scripts/checkfiles -x drivers/mmc/ | tee -a mmc.log
./scripts/checkfiles -x fs/ | tee -a fs.log
mv mmc.log ../../codeRule/fs/
mv fs.log ../../codeRule/fs/

#LCD
./scripts/checkfiles -x drivers/video/msm/mdss/ | tee -a mdss.log
./scripts/checkfiles -x drivers/video/backlight | tee -a backlight.log
mv mdss.log ../../codeRule/lcd/
mv backlight.log ../../codeRule/lcd/

#INPUT
./scripts/checkfiles -x drivers/misc/lge_sm100.c | tee -a input.log
./scripts/checkfiles -x include/linux/lge_sm100.h  | tee -a input.log
./scripts/checkfiles -x drivers/leds/leds-qpnp.c | tee -a input.log
./scripts/checkfiles -x drivers/input/keyboard/gpio_keys.c | tee -a input.log
./scripts/checkfiles -x include/linux/gpio_keys.h  | tee -a input.log
./scripts/checkfiles -x drivers/input/touchscreen/touch_synaptics.c | tee -a input.log
./scripts/checkfiles -x include/linux/input/touch_synaptics.h  | tee -a input.log
mv input.log ../../codeRule/input/

#POWER
./scripts/checkfiles -x drivers/hwmon/ | tee -a hwmon.log
./scripts/checkfiles -x drivers/power/ | tee -a power.log
./scripts/checkfiles -x drivers/mfd/ | tee -a mfd.log
./scripts/checkfiles -x drivers/regulator/ | tee -a regulator.log
./scripts/checkfiles -x drivers/thermal/ | tee -a thermal.log
mv hwmon.log ../../codeRule/power/
mv power.log ../../codeRule/power/
mv mfd.log ../../codeRule/power/
mv regulator.log ../../codeRule/power/
mv thermal.log ../../codeRule/power/

#MODEM
./scripts/checkfiles -x arch/arm/mach-msm/peripheral-loader.c | tee -a modem.log
./scripts/checkfiles -x arch/arm/mach-msm/peripheral-loader.h | tee -a modem.log
./scripts/checkfiles -x arch/arm/mach-msm/pil-q6v5.c | tee -a modem.log
./scripts/checkfiles -x arch/arm/mach-msm/pil-q6v5.h | tee -a modem.log
./scripts/checkfiles -x arch/arm/mach-msm/pil-q6v5-mss.c | tee -a modem.log
./scripts/checkfiles -x arch/arm/mach-msm/smd.c | tee -a modem.log
./scripts/checkfiles -x arch/arm/mach-msm/smd_private.c | tee -a modem.log
./scripts/checkfiles -x arch/arm/mach-msm/smd_private.h | tee -a modem.log
#./scripts/checkfiles -x arch/arm/mach-msm/smd_lge.c | tee -a modem.log
#./scripts/checkfiles -x include/linux/diagchar.h | tee -a modem.log
#./scripts/checkfiles -x drivers/char/diag/Kconfig | tee -a modem.log
#./scripts/checkfiles -x drivers/char/diag/Makefile | tee -a modem.log
#./scripts/checkfiles -x drivers/char/diag/diagfwd.c | tee -a modem.log
#./scripts/checkfiles -x drivers/char/diag/lg_dm_dev_tty.c | tee -a modem.log
#./scripts/checkfiles -x drivers/char/diag/lg_dm_dev_tty.h | tee -a modem.log
#./scripts/checkfiles -x drivers/char/diag/lg_dm_tty.c | tee -a modem.log
#./scripts/checkfiles -x drivers/char/diag/lg_dm_tty.h | tee -a modem.log
mv modem.log ../../codeRule/modem/

#AUDIO
./scripts/checkfiles -x sound/soc/codecs/wcd9xxx-mbhc.c | tee -a sound.log
./scripts/checkfiles -x sound/soc/codecs/wcd9xxx-mbhc.h | tee -a sound.log
./scripts/checkfiles -x sound/soc/codecs/wcd9320.c | tee -a sound.log
./scripts/checkfiles -x sound/soc/msm/msm8974.c | tee -a sound.log
mv sound.log ../../codeRule/sound/
