#ifndef __command_h__
#define __command_h__

extern void anal_command(int argc, char *argv[]);
extern void mmi_dump_cmd(int argc, char *argv[]);
extern void mmi_bbm_init_cmd(int argc, char *argv[]);
extern int String2Freq(char str[]);
extern int ChannelScan(void);
extern void mmi_epg_cmd (int argc, char *argv[]);
#endif /* __command_h__ */
