#ifndef __DAUDIO_IE_TWXXXX__
#define __DAUDIO_IE_TWXXXX__

int is_twxxxx_ie_invalid_arg(int mode, char level);
int twxxxx_set_ie(int cmd, unsigned char level);
int twxxxx_get_ie(int cmd, unsigned char* level);
int tw8836_hue_valid_level(char level);

#endif
