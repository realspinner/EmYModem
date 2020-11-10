/*
 * iSerialIO.h
 *
 *  Created on: Jul 17, 2020
 *      Author: spinner
 */

#ifndef ISERIALIO_H_
#define ISERIALIO_H_


#include <cstdint>
#include <cstddef>


class iSerialIO {
public:
    virtual ~iSerialIO() = default;
	virtual bool sendData(uint8_t const* data, size_t size) = 0;
	virtual bool sendString(char const* string) = 0;
	virtual void readData() = 0;
	virtual void onRead(uint8_t const* data, size_t size) = 0;
	virtual void flush() = 0;
};


#endif /* ISERIALIO_H_ */
