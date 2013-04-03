# -*- makefile -*-

CONTIKI_PROJECT = udpPlug
all: $(CONTIKI_PROJECT)

flash: $(CONTIKI_PROJECT).flash

USE_USART3=1

UIP_CONF_IPV6=1
UIP_CONF_IPV6_RPL=1

TDMA_TEST=1

PROJECT_DEFINES = \
	AUTOSTART_ENABLE \
	TDMA_TEST=1 \
	PROJECT_CONF_H=\"project-conf.h\" \

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
