#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "fci_types.h"
#include "clib.h"
#include "monitor.h"
#include "command.h"

#define PROMPT	  "DMB"
#define DEVICE_FILENAME "/dev/dmb"

char *help_msg[] = {
     "-----------------------------------------------------------",
     "?/he/help : help display [mem/peri]",
     "ver       : version display",
     "exit   : software reset",
     "-----------------------------------------------------------",
	 "brd       : byte read command		[start addr] [length]", //---add start
	 "bwr       : byte write command    [start addr] [data] [length]",
	 "wrd       : word read command     [start addr] [length]",
	 "wwr       : word write command    [start addr] [data] [length]",
	 "lrd       : long read command     [start addr] [length]",
	 "lwr       : long write command    [start addr] [data] [length]",
     "-----------------------------------------------------------",
	 "verify    : [address] [retry]",
	 "iverify   : [address] [retry] [delay]",
     "-----------------------------------------------------------",
	 "tuner     : [frequency]",
     "-----------------------------------------------------------",
	 "reset     : [hex]",
	 "init      : ",
	 "channel   : [subChId] [svcChId]",

     "-----------------------------------------------------------",
	 "data      : [dump]",
	 "msc       : [dump|clear] [subChId]",
	 "ird       : [start addr] [length]",
	 "iwr       : [start addr] [data] [length]",
     "-----------------------------------------------------------",
     ""
};

char *argv[10];
int  argc;
char hist_buf[10][100];
char buf[100];
int  cnt;


/* white-space is ' ' ',' '\f' '\n' '\r' '\t' '\v' */

int isspace2(char c)
{
	if (c==' ' || c==',' || c=='\f' || c=='\n' || c=='\r' || c=='\t' || c=='\v') return 1;
	else return 0;
}

int make_argv(char *s, int argvsz, char *argv[])
{
        int argc = 0;

        /* split into argv */
        while (argc < argvsz - 1) {

                /* skip any white space */
                while ((*s == ' ') || (*s == '\t') || (*s == 0x0a))
                        ++s;

                if (*s == '\0')         /* end of s, no more args       */
                        break;

                argv[argc++] = s;       /* begin of argument string     */

                /* find end of string */
                while (*s && (*s != ' ') && (*s != '\t') && (*s != 0x0a))
                        ++s;

                if (*s == '\0')         /* end of s, no more args       */
                        break;

                *s++ = '\0';            /* terminate current arg         */
        }
        argv[argc] = NULL;

        return argc;
}

void version_display(void)
{
	print_log("\n");
	print_log("\t*********************************************\n");
	print_log("\t*      FCI   DMB RECEIVER MODEL             *\n");
	print_log("\t*                                           *\n");
	print_log("\t*         %s  %s             *\n", __DATE__,  __TIME__);
	print_log("\t*                                           *\n");
	print_log("\t*       Digital Multimedia Broadcast        *\n");
	print_log("\t*                                           *\n");
	print_log("\t*            http://www.fci.co.kr           *\n");
	print_log("\t*********************************************\n");

	print_log("\tVersion Information\n");
	print_log("\tFunctions:\n");
	print_log("\n");
}

void Mon_help_command(void)
{
	int  line;
	char **ptr;
	line = 0;

	ptr = help_msg;


	while(1)
	{
		if(*ptr[0] == 0) break;

		print_log("%s\n", (int)*ptr++);
		if(++line % 30 == 0)
		{
			print_log("\nPress return key when ready...");
			getchar();
		}
	}

}

void Monitor(void)
{
	char  tmp_buf[100];
	int   index;
	int   i;
	int minute, prev_min, sec, prev_sec;

	minute = prev_min = sec = prev_sec = 0;

	print_log("\nTask MONI Start...\n");

	for(cnt=0; cnt<10; cnt++) hist_buf[cnt][0] = '\0';
	cnt = 0;

	while ( 1 )
	{
		print_log("[%s] ", PROMPT);
		fgets(buf, 100, stdin );


		if(buf[0] == '.')	/* Previous command */
		{
			strcpy(buf, "!!");
		}

		if(buf[0] == '!')	/* History command */
		{
			if(buf[1] == '!' || (buf[1] >='0' && buf[1] <='9'))
			{
				if(buf[1] == '!')  index = (cnt+9) % 10;
				else	       index = buf[1] - '0';

				if(buf[2] == '\0' || buf[3] == '\0')
				{
					strcpy(buf, hist_buf[index]);
				}
				else
				{
					sprintf(tmp_buf, "%s %s", hist_buf[index], &buf[3]);
					strcpy(buf, tmp_buf);
				}
				print_log("\n%s", (int)buf);
			}
			else
			{
				for(i=0 ; i<10 ; i++)
				{
					print_log("%d %s\n", i, hist_buf[i]);
				}
				continue;
			}
		}

		strcpy(tmp_buf, buf);
		argc = make_argv(buf, sizeof(argv)/sizeof(argv[0]), argv);

		if ( argc != 0 )
		{
			anal_command(argc, argv);
			strcpy(hist_buf[cnt], tmp_buf);
			cnt++;
			if(cnt == 10) cnt = 0;
		}
	}
}

void exec_mon_cmd(char *ptr)
{
	char *sargv[10];
	int  sargc;

	sargc = make_argv(ptr, sizeof(sargv)/sizeof(sargv[0]), sargv);
	if(sargc != 0) anal_command(sargc, sargv);
}
