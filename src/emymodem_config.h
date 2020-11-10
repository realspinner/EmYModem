#ifndef __EMYMODEM_CONFIG_H__
#define __EMYMODEM_CONFIG_H__


#define EMYMODEM_USE_1K
#define EMYMODEM_SEND_GENERAL_TIMEOUT   60  // in seconds
#define EMYMODEM_SEND_ACK_TIMEOUT        3  // in seconds
#define EMYMODEM_CANCEL_SEQUENCE         2  // Two consecutive CANs to abort
#define EMYMODEM_SEND_INPUT_QUEUE       16  // Send input queue

//#define EMYMODEM_LOW_MEMORY_PROFILE       // define to reduce memory footprint

#endif /* __EMYMODEM_CONFIG_H__ */
