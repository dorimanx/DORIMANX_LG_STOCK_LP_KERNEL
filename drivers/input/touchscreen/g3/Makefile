#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

ifdef CONFIG_MACH_MSM8974_G3_KDDI_EVB
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI4_I2C) +=synaptics/
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += lge_touch_core_kddi_evb.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += touch_synaptics_kddi_evb.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += touch_synaptics_ds4_fw_upgrade.o
else
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += lge_touch_core.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += touch_synaptics.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += touch_synaptics_proximity.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_EVB) += lge_touch_core.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_EVB) += touch_synaptics_evb.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += touch_synaptics_ds5_fw_upgrade.o
endif

obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += DS5/

#obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)		+= synaptics_i2c_rmi4.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX)	+= synaptics_dsx/
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI4_DEV)	+= synaptics_rmi_dev.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE) 	+= synaptics_fw_update.o

