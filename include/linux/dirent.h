#ifndef _LINUX_DIRENT_H
#define _LINUX_DIRENT_H

struct linux_dirent64 {
	u64		d_ino;
	s64		d_off;
	unsigned short	d_reclen;
	unsigned char	d_type;
	char		d_name[0];
};

/* readdir2 porting -> start */
struct linux_dirent64_avn {
       u64             d_ino;
       s64             d_off;
       unsigned short  d_reclen;
       unsigned char   d_type;
       unsigned short  d_mode;
       unsigned long   d_mtime;
       long long       d_filesize;
       char            d_name[0];
};
/* readdir2 porting -> end */
#endif
