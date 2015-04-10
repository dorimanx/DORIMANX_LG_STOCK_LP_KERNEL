LFLAGS  = -static -pthread

CC = /opt/toolchains/arm-2009q3/bin/arm-none-linux-gnueabi-gcc

OBJS = main.o monitor.o command.o clib.o fci_oal.o fc8300_api.o fci_dal.o fci_dataread.o
SRCS = $(OBJS:.o=.c)

.SUFFIXES : .c .o

fccon: $(OBJS)
	$(CC) $(LFLAGS) -o fccon $(OBJS)
	mv fccon $(PWD)/../

clean:
	rm -rf $(OBJS) ../fccon


