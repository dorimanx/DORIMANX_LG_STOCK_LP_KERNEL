#
# Makefile for the ISDB-T FC8300 drivers.
#

# Each configuration option enables a list of files.
# LGE

obj-y             += bbm.o
obj-y             += fc8300_bb.o
obj-y             += fc8300_drv_api.o
obj-y             += fc8300_i2c.o
obj-y             += fc8300_isr.o
obj-y             += fc8300_ppi.o
obj-y             += fc8300_spi.o
obj-y             += fc8300_tun.o
obj-y             += fc8300_tun_table.o
obj-y             += fc8300b_tun.o
obj-y             += fc8300b_tun_table.o
obj-y             += fci_hal.o
obj-y             += fci_hpi.o
obj-y             += fci_oal.o
#obj-y		+= fci_ringbuffer.o
obj-y             += fci_tun.o

EXTRA_CFLAGS += -Idrivers/broadcast/oneseg/fc8300
EXTRA_CFLAGS += -Idrivers/broadcast/oneseg/fc8300/fc8300Driver/inc