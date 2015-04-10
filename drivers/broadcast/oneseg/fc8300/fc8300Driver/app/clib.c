#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

unsigned char toint(char c)
{
	if ( c >= '0' && c <= '9' ) return(c-'0');
	else if ( c >= 'a' && c <= 'f' ) return(c-'a'+10);
	else if ( c >= 'A' && c <= 'F' ) return(c-'A'+10);
	else return(0);
}

unsigned int htoi(char *s)
{
	unsigned int sum = 0;

	while (*s)
	{
		sum = sum * 16 + toint(*s++);
	}
	return(sum);
}

int btoi(char *s)
{
	short sum = 0;

	while (*s)
	{
		sum = sum * 2 + toint(*s++);
	}
	return(sum);
}

unsigned int ctoi(char *s)
{
	unsigned int sum = 0;
	while(*s)
	{sum = sum * 10 + toint(*s++);}
	return sum;

}

int todecimal(char c)
{
	if(isdigit(c)) 
		return (c-'0');
	else           
		return (0);
}

int dtoi(char *s)
{
	int sum = 0;

	while(*s)
	{
		sum = sum * 10 + todecimal(*s++);
	}

	return sum;
}

