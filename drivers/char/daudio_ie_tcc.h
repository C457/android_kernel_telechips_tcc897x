#ifndef __DAUDIO_IE_TCC__
#define __DAUDIO_IE_TCC__

int is_tcc_ie_invalid_arg(int cmd, char value);
int set_tcc_ie(int mode, unsigned char level);
int get_tcc_ie(int mode, unsigned char *level);
void init_tcc_video_ie(unsigned char brightness, unsigned char contrast,
		unsigned char gamma, unsigned char saturation);
void init_tcc_ie(unsigned char brightness, unsigned char contrast,
		unsigned char gamma, unsigned char saturation);

#endif
