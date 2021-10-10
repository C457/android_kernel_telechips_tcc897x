#ifndef __DAUDIO_IE_LUT___
#define __DAUDIO_IE_LUT___

/**
 * 0~ 255 -> TW8836 HUE (0 ~ 90)
 */
int parse_value_to_tw8836_hue(int value);

/**
 * TW8836 HUE (0 ~ 90) --> (0 ~ 255)
 */
int parse_tw8836_hue_to_value(int tw8836_hue);

#endif
