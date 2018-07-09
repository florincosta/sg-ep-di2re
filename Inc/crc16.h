/*
 * crc16.h
 *
 *  Created on: 9 Jul 2018
 *      Author: fcos
 */

#ifndef CRC16_H_
#define CRC16_H_

#include <stdint.h>

uint16_t CRC16_calc(const uint8_t* const data, const uint32_t length);

#endif /* CRC16_H_ */
