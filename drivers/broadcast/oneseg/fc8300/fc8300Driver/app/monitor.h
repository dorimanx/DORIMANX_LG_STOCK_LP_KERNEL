#ifndef __monitor_h__
#define __monitor_h__

#define iscmd(X)	(!strcmp(argv[0],X))
#define is1cmd(X)	(!strcmp(argv[1],X))
#define is2cmd(X)	(!strcmp(argv[2],X))
#define is3cmd(X)	(!strcmp(argv[3],X))
#define is4cmd(X)	(!strcmp(argv[4],X))
#define is5cmd(X)	(!strcmp(argv[5],X))
#define is6cmd(X)	(!strcmp(argv[6],X))
#define is7cmd(X)	(!strcmp(argv[7],X))

extern int mon_tid;

extern char *help_msg[];

extern char *argv[10];
extern int  argc;
extern char hist_buf[10][100];
extern char buf[100];
extern int  cnt;

extern void version_display(void);
extern void Monitor(void);
extern void Mon_help_command(void);
extern int make_argv(char *s, int argvsz, char *argv[]);

#endif /* __monitor_h__ */
