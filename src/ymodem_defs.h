#ifndef __YMODEM_DEFS_H__
#define __YMODEM_DEFS_H__

// YMODEM code set
typedef enum _YmodemCode_t {
    YM_NONE = 0x00,
    YM_SOH  = 0x01,
    YM_STX  = 0x02,
    YM_EOT  = 0x04,
    YM_ACK  = 0x06,
    YM_NAK  = 0x15,
    YM_CAN  = 0x18,
    YM_EOF  = 0x1A,
    YM_C    = 0x43,
    YM_K    = 0x4B,
    YM_G    = 0x47,
    YM_A1   = 0x41,
    YM_A2   = 0x61
} YmodemCode_t;


#endif /* __YMODEM_DEFS_H__ */
