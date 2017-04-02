//
// Created by Admin on 2017-04-02.
//

#ifndef QSRC_STM32_ROT2PROG_H
#define QSRC_STM32_ROT2PROG_H

// thanks to http://ryeng.name/blog/3


#ifdef __cplusplus
#include <cstdint>
extern "C" {
#endif

static const uint8_t Rot2Prog_COMMAND_STOP = 0x0F;
static const uint8_t Rot2Prog_COMMAND_STATUS = 0x1F;
static const uint8_t Rot2Prog_COMMAND_SET = 0x2F;

typedef struct Rot2ProgCmd {
    uint8_t start_byte;
    uint8_t azimuth[4];
    uint8_t azimuth_resolution;
    uint8_t elevation[4];
    uint8_t elevation_resolution;
    uint8_t command;
    uint8_t end_byte;
} Rot2ProgCmd;

typedef struct Rot2ProgResponse {
    uint8_t start_byte;
    uint8_t azimuth[4];
    uint8_t azimuth_resolution;
    uint8_t elevation[4];
    uint8_t elevation_resolution;
    uint8_t end_byte;
} Rot2ProgResponse;

#ifdef __cplusplus
}
#endif
#endif //QSRC_STM32_ROT2PROG_H
