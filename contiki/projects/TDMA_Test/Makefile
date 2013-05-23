# -*- makefile -*-
.PHONY: rf231_slotted_hal.o rf231_slotted.o slotted_clear

CONTIKI_PROJECT = udpPlug
all:  $(CONTIKI_PROJECT)

flash: $(CONTIKI_PROJECT).flash

start_ocd: oocd
	@

slotted_clear:
	@ echo -n removing slotted objects...
	@-rm -f $(OBJECTDIR)/rf231_slotted*
	@ echo " done."	

client: slotted_clear $(CONTIKI_PROJECT)
	@mv $(CONTIKI_PROJECT).$(TARGET).elf $(CONTIKI_PROJECT).CLIENT.elf

koord : PROJECT_DEFINES += \
	SLOTTED_KOORDINATOR \

##	JITTER_SIMULATION

koord: slotted_clear $(CONTIKI_PROJECT)
	@mv $(CONTIKI_PROJECT).$(TARGET).elf $(CONTIKI_PROJECT).KOORD.elf

bin: $(CONTIKI_PROJECT).bin
	@

USE_USART3=1

UIP_CONF_IPV6=1
UIP_CONF_IPV6_RPL=1

TDMA_TEST=1

PROJECT_DEFINES = \
	AUTOSTART_ENABLE \
	TDMA_TEST=1 \
	PROJECT_CONF_H=\"project-conf.h\" \

ifeq ($(TDMA_MODE),KOORD_SIM)
PROJECT_DEFINES += \
	SLOTTED_KOORDINATOR \
	JITTER_SIMULATION
else ifeq ($(TDMA_MODE),KOORD)
PROJECT_DEFINES+= SLOTTED_KOORDINATOR
else 
PROJECT_DEFINES+= 
endif

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
