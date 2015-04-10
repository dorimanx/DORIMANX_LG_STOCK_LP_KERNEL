#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI)	+= synaptics_i2c_rmi.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C) +=synaptics/

obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI4_I2C) +=synaptics/
ifdef CONFIG_MACH_LGE
ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_VU3EVB
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_VU3EVB) += lge_touch_core_vu3evb.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_VU3EVB) += touch_synaptics_vu3evb.o
else
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += lge_touch_core.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += touch_synaptics.o
endif
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += touch_synaptics_ds5_fw_upgrade.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += DS5/
endif
ifndef CONFIG_MACH_LGE
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)		+= synaptics_i2c_rmi4.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI4_DEV)	+= synaptics_rmi_dev.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE) 	+= synaptics_fw_update.o
endif

