/**
 * @file ant_key_manager_config.h
 * @author DuoDuoJuZi
 * @date 2026-05-18
 * @brief ANT+ Network Key configuration
 */

#ifndef ANT_KEY_MANAGER_CONFIG_H__
#define ANT_KEY_MANAGER_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ANT_PLUS_NETWORK_KEY
#define ANT_PLUS_NETWORK_KEY {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}
#endif

#ifndef ANT_FS_NETWORK_KEY
#define ANT_FS_NETWORK_KEY {0, 0, 0, 0, 0, 0, 0, 0}
#endif

#ifdef __cplusplus
}
#endif

#endif
