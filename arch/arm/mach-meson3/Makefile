#
# Makefile for the linux kernel.
#

# Common support
obj-y := core.o clock.o common.o fiq.o fiq_bridge.o
obj-m :=
obj-n :=
obj-  :=

obj-$(CONFIG_CPU_FREQ) 		+= cpu.o
obj-$(CONFIG_ARCH_MESON3)	+= pinmux.o
obj-$(CONFIG_ARCH_MESON3)	+= gpio.o
obj-$(CONFIG_ARCH_MESON3)	+= power_gate.o
obj-$(CONFIG_ARCH_MESON3)	+= sd_pad.o
obj-$(CONFIG_AML_SUSPEND) += power_suspend.o
obj-$(CONFIG_ARCH_MESON3)	+= lm.o usbclock.o clk_set.o sram.o sleep.o

obj-$(CONFIG_MACH_MESON3_REFF01)	+= board-m3-reff01.o
obj-$(CONFIG_MACH_MESON3_REFF02)	+= board-m3-reff02.o
obj-$(CONFIG_MACH_MESON3_REFF03)	+= board-m3-reff03.o
obj-$(CONFIG_MACH_MESON3_REFF04)	+= board-m3-reff04.o
obj-$(CONFIG_MACH_MESON3_REFF05)	+= board-m3-reff05.o
obj-$(CONFIG_MACH_MESON3_REFF06)	+= board-m3-reff06.o
obj-$(CONFIG_MACH_MESON3_REFF06_1024)	+= board-m3-reff06-1024.o
obj-$(CONFIG_MACH_MESON3_REFF06_848L)	+= board-m3-reff06-848L.o
obj-$(CONFIG_MACH_MESON3_REFF06_MID7306CE)	+= board-m3-reff06-mid7306ce.o
obj-$(CONFIG_MACH_MESON3_REFF07)	+= board-m3-reff07.o
obj-$(CONFIG_MACH_MESON3_REFF08)	+= board-m3-reff08.o
obj-$(CONFIG_MACH_MESON3_REFF09)	+= board-m3-reff09.o
obj-$(CONFIG_MACH_MESON3_REFF10)	+= board-m3-reff10.o
obj-$(CONFIG_MACH_MESON3_REFF11)	+= board-m3-reff11.o
obj-$(CONFIG_MACH_MESON3_REFF11_4329)	+= board-m3-reff11-4329.o
obj-$(CONFIG_MACH_MESON3_REFF11_P800)	+= board-m3-reff11-P800.o
obj-$(CONFIG_MACH_MESON3_REFF14)	+= board-m3-reff14.o
obj-$(CONFIG_MACH_MESON3_REFF16)	+= board-m3-reff16.o
# Power Management
obj-$(CONFIG_SUSPEND)			+= pm.o
