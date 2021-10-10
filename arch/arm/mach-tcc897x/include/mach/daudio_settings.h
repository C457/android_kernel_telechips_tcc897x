#ifndef __DAUDIO_SETTINGS__
#define __DAUDIO_SETTINGS__

#include <mach/daudio.h>
#include <mach/daudio_eng.h>

#define SETTINGS_PARTITION      "settings"

/**
 * SETTING PARTITION STRUCTURE.
 *
 * IE ORIGINAL DATA			0 ~ 511
 * IE ORIGINAL CMD			512 ~ 1023
 * IE BACKUP DATA			1024 ~ 1535
 * IE BACKUP CMD			1536 ~ 2047
 * ENGINEER MODE			2048 ~ 2559
 **/
#define OFFSET_EM_SETTINGS	 0	
#define OFFSET_VARIANT_SETTINGS				OFFSET_EM_SETTINGS + 512
#define OFFSET_BOOT_ACTIVE_SETTINGS			OFFSET_VARIANT_SETTINGS + 512
#define OFFSET_IE_SETTINGS					OFFSET_BOOT_ACTIVE_SETTINGS + 512

void print_ie_info(ie_setting_info *info);

int read_em_setting(em_setting_info *info);
int write_em_setting(em_setting_info *info);
int read_ie_setting(ie_setting_info *info);
int write_ie_setting(ie_setting_info *info);
void print_em_info(em_setting_info *info);
void update_settings(void);

#endif
