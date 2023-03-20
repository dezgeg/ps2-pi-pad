ps2pipad-objs += ps2pipad_kmod.o
ps2pipad-objs += fiq_asm.o
obj-m += ps2pipad.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
