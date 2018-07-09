/*
 * mac.c
 *
 *  Created on: 9 Jul 2018
 *      Author: fcos
 */

#include "crc16.h"


uint16_t MAC_hash(uint8_t* key, uint32_t key_length, uint8_t *message, uint32_t message_length) {
    uint8_t buffer[128];
    uint32_t i, j;

    i = 0;
    for(j = 0; j < key_length && i < sizeof(buffer); j++) {
        buffer[i++] = key[j];
    }

    for(j = 0; j < message_length && i < sizeof(buffer); j++) {
        buffer[i++] = message[j];
    }

    return CRC16_calc(buffer, i);
}
