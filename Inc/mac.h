/*
 * mac.h
 *
 *  Created on: 9 Jul 2018
 *      Author: fcos
 */

#ifndef MAC_H_
#define MAC_H_

#include <stdint.h>

uint16_t MAC_hash(uint8_t* key, uint32_t key_length, uint8_t *message, uint32_t message_length);

#endif /* MAC_H_ */
