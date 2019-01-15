#ifndef _RDB_ERROR_H
#define _RDB_ERROR_H

#ifdef  __cplusplus
extern "C" {
#endif

#define CMD_SUCCESS         0x10
#define CMD_FAIL            0x11
#define CMD_RESET           0x20
#define CMD_CW_ON           0x21
#define CMD_ANT_MISS        0x22
#define CMD_W_FLASH         0x23
#define CMD_R_FLASH         0x24
#define CMD_PWR_OUT         0x25
#define CMD_TAG_INVT        0x31
#define CMD_TAG_READ        0x32
#define CMD_TAG_WRITE       0x33
#define CMD_TAG_LOCK        0x34
#define CMD_TAG_KILL        0x35
#define CMD_TAG_NO          0x36
#define CMD_TAG_ACCESS      0x37
#define CMD_BUF_EMPTY       0x38
#define CMD_NXP_CMD         0x3C
#define CMD_PASSWD          0x40
#define CMD_INVLD_PARAM     0x41
#define CMD_WDCNT_LONG      0x42
#define CMD_MEMBANK_OOR     0x43    // OOR = out of range
#define CMD_LOCK_REGION     0x44
#define CMD_LOCK_TYPE       0x45
#define CMD_READER_ADDR     0x46
#define CMD_ANT_ID_OOR      0x47
#define CMD_PWR_OOR         0x48
#define CMD_FREQ_OOR        0x49
#define CMD_BAUD_OOR        0x4A
#define CMD_PEEPER_OOR      0x4B
#define CMD_EPC_TOOLONG     0x4C
#define CMD_EPC_ERROR       0x4D
#define CMD_EPC_MODE        0x4E
#define CMD_FREQ_RANGE      0x4F
#define CMD_RN16_TAG        0x50
#define CMD_INVLD_DRM       0x51
#define CMD_PLL_LOCK        0x52
#define CMD_RF_RSP          0x53
#define CMD_WRG_PWR_OUT     0x54    //WRG = WRONG
#define CMD_FW_AUTH         0x55
#define CMD_WRG_SPECT       0x56
#define CMD_LOW_PWR_OUT     0x57
#define CMD_RF_RET_LOSS     0xEE


#endif

