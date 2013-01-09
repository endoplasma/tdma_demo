/*----------------------------------------------------------------------------
 * Name:    uart4_putchar.c
 * Purpose: 'Retarget' layer for target-dependent low level functions
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2011 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include "usbd_cdc_core.h"

#pragma import(__use_no_semihosting_swi)


extern CDC_IF_Prop_TypeDef  APP_FOPS;

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;


int fputc(int c, FILE *f) {
	uint8_t u8c = (uint8_t)c;
  APP_FOPS.pIf_DataTx(&u8c, 1);
	return c;
}


int fgetc(FILE *f) {
  return 0;
}


int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}


void _ttywrch(int c) {
	uint8_t u8c = (uint8_t)c;
  APP_FOPS.pIf_DataTx(&u8c, 1);
}


void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}
