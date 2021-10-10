#ifndef __DAUDIO_IE__
#define __DAUDIO_IE__

int daudio_set_ie(int mode, unsigned char level);
int daudio_get_ie(int mode, unsigned char *level);

int get_ie_debug_level(void);

#endif
