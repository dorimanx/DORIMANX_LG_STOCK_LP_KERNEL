#!/bin/sh

# This is dorimanx upload release file.
# You will need ncftp installed to use this.
# http://www.ncftp.com/download/ (i have used the 64bit linux)
# download extract, then run: make install and all set.
# You must have ftp user/pass set in /root/ftp_login_mirror1.cfg + /root/ftp_login_mirror1.cfg to access your server!
# set it like this:
# vi /root/ftp_login_mirrorX.cfg (must be root!) dont add the #

#host ip.of.YOUR-server
#user john
#pass password

# Save and chmod 700 /root/ftp_login_mirrorX.cfg
# have fun.

# Before you start make sure READY-RELEASES/ contain only new updates.

build_selector=$1;

case "${build_selector}" in
	d800)
	(
		cp READY-RELEASES/*D800* ../LG-G2-LP-DOWNLOADS/D800/
		ncftpput -f /root/ftp_login_mirror1.cfg /LG-LP/D800/ READY-RELEASES/*D800*
		#ncftpput -f /root/ftp_login_mirror2.cfg /LG-LP/D800/ READY-RELEASES/*D800*
	)&
	;;
	d801)
	(
		cp READY-RELEASES/*D801* ../LG-G2-LP-DOWNLOADS/D801/
		ncftpput -f /root/ftp_login_mirror1.cfg /LG-LP/D801/ READY-RELEASES/*D801*
		#ncftpput -f /root/ftp_login_mirror2.cfg /LG-LP/D801/ READY-RELEASES/*D801*
	)&
	;;
	d802)
	(
		cp READY-RELEASES/*D802* ../LG-G2-LP-DOWNLOADS/D802/
		ncftpput -f /root/ftp_login_mirror1.cfg /LG-LP/D802/ READY-RELEASES/*D802*
		#ncftpput -f /root/ftp_login_mirror2.cfg /LG-LP/D802/ READY-RELEASES/*D802*
	)&
	;;
	ls980)
	(
		cp READY-RELEASES/*LS980* ../LG-G2-LP-DOWNLOADS/LS980/
		ncftpput -f /root/ftp_login_mirror1.cfg /LG-LP/LS980/ READY-RELEASES/*LS980*
		#ncftpput -f /root/ftp_login_mirror2.cfg /LG-LP/LS980/ READY-RELEASES/*LS980*
	)&
	;;
	vs980)
	(
		cp READY-RELEASES/*VS980* ../LG-G2-LP-DOWNLOADS/VS980/
		ncftpput -f /root/ftp_login_mirror1.cfg /LG-LP/VS980/ READY-RELEASES/*VS980*
		#ncftpput -f /root/ftp_login_mirror2.cfg /LG-LP/VS980/ READY-RELEASES/*VS980*
		cd ../LG-G2-LP-DOWNLOADS/
		git add --all
		cd ../DORIMANX_LG_STOCK_LP_KERNEL/
	)&
	;;
	*)
		echo "input ready kernel build model";
	;;
esac;

