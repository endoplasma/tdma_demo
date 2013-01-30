# -*- makefile -*-

CONTIKI_PROJECT = border-router 
all: $(CONTIKI_PROJECT) 

CONTIKI_SOURCEFILES += httpd-simple.c slip-bridge.c

UIP_CONF_IPV6=1
UIP_CONF_IPV6_RPL=1
USE_SLIP=1

PROJECT_DEFINES = \
	AUTOSTART_ENABLE \
	WEBSERVER=1 \
	PROJECT_CONF_H=\"project-conf.h\" \

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
