//
// Created by Admin on 2017-04-18.
//

#include "crc.h"

/**
 * @author https://github.com/olegv142/stm32-config/blob/master/common/crc16.c
 * @param crc
 * @param data
 * @return
 */
static inline uint16_t crc16_up_(uint16_t crc, uint8_t data)
{
    uint16_t t;
    data ^= crc & 0xff;
    data ^= data << 4;
    t = (uint16_t) (((uint16_t)data << 8) | ((crc >> 8) & 0xff));
    t ^= (unsigned char)(data >> 4);
    t ^= ((uint16_t)data << 3);
    return t;
}

uint16_t crc16(uint8_t * data, uint8_t size){
    uint16_t crc = 0xffff;
    for (int i = 0; i < size; ++i) {
        crc = crc16_up_(crc, data[i]);
    }
    return crc;
}