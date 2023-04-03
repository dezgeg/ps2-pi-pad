ccflags-y += -Wno-declaration-after-statement
ps2pipad-objs += kmod.o
ps2pipad-objs += fiq_asm.o
obj-m += ps2pipad.o

all: ps2pipad
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

ps2pipad: usermode.c uapi.h
	$(CC) -O3 $< -o $@
