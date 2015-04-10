
# LGE Board Selection
obj-$(CONFIG_MACH_MSM8974_G2) += 8974-g2/
obj-$(CONFIG_MACH_MSM8974_TIGERS) += 8974-g2/
obj-$(CONFIG_MACH_MSM8974_G3) += 8974ac-g3/
obj-$(CONFIG_MACH_MSM8974_DZNY) += 8974ac-dzny/

obj-$(CONFIG_MACH_LGE) += devices_lge.o
ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO_V1_7
obj-$(CONFIG_MACH_LGE) += lge_charging_scenario_v1_7.o
else
obj-$(CONFIG_MACH_LGE) += lge_charging_scenario.o
endif
obj-$(CONFIG_LCD_KCAL) += lge_kcal_ctrl.o
obj-$(CONFIG_WIFI_CONTROL_FUNC) += wifi_bcm_lge.o
obj-$(CONFIG_LGE_HANDLE_PANIC) += lge_handle_panic.o
obj-$(CONFIG_MACH_LGE) += lge_gpio_debug.o

obj-$(CONFIG_LGE_BOOTLOADER_LOG) += lge_bootloader_log.o
obj-$(CONFIG_LGE_BOOTLOADER_TIME_CHECKER) += lge_bootloader_time_checker.o

obj-$(CONFIG_LGE_QFPROM_INTERFACE) += lge_qfprom_access.o
obj-$(CONFIG_LGE_QSDL_SUPPORT) += lge_qsdl.o

CFLAGS_devices_lge.o += -Idrivers/staging/android
