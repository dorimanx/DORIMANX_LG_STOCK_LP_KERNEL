/*****************************************************************************
 Copyright(c) 2009 FCI Inc. All Rights Reserved

 File name : fci_oal.c

 Description : OS Adaptation Layer

 History :
 ----------------------------------------------------------------------
 2009/09/13 	jason		initial
*******************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include "fci_types.h"

void print_log(char *fmt,...)
{
	va_list ap;
	char str[256];

	va_start(ap,fmt);
	vsprintf(str,fmt,ap);

	printf("%s", str);

	va_end(ap);
}

void msWait(int ms)
{
	usleep(ms*1000);
}

