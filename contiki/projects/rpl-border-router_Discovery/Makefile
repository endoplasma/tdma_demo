# -*- makefile -*-

CONTIKI_PROJECT = border-router 
all: $(CONTIKI_PROJECT) 

CONTIKI_SOURCEFILES += httpd-simple.c slip-bridge.c

USE_USART3=1
USE_SLIP=1

UIP_CONF_IPV6=1
UIP_CONF_IPV6_RPL=1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
