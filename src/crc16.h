#ifndef __CRC16_H__
#define __CRC16_H__


#include <stdint.h>
#include <stddef.h>

/**
 * Calculate the CRC16 on given bytes.
 * @param crc - initial state of CRC16
 * @param c_ptr - pointer to the data buffer
 * @param len - number of bytes in data buffer
 * @return CRC16 for the given data buffer
 */
uint16_t CalculateCRC16(uint16_t crc, const void *c_ptr, size_t len);


#endif /* __CRC16_H__ */
