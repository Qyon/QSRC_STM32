//
// Created by Admin on 2017-04-17.
//

#ifndef QSRC_STM32_PROTOCOL_H
#define QSRC_STM32_PROTOCOL_H

#pragma pack(1)
static const uint32_t packetHeader = 0x51535243;

enum ProtocolCommand {
    cmdPing = 0x01,
    cmdPong = 0x02,
    cmdReadAzEl = 0x03,
    cmdReadAzElResponse = 0x04,
    cmdGoToAzEl = 0x05,
    cmdEmergencyStop = 0x06,
    cmdReadDateTime = 0x07,
    cmdReadDateTimeResponse = 0x08,
    cmdSetDateTime = 0x09,
    cmdReadEEPROM = 0x0a,
    cmdReadEEPROMResponse = 0x0b,
    cmdWriteEEPROM = 0x0c,
    cmdSetAuxOutput = 0x0d,
    cmdReadEncoders = 0x0e,
    cmdReadEncodersResponse = 0x0f,
    cmdSetAzEl = 0x10,
    cmdReadMaxSpeed = 0x11,
    cmdReadMaxSpeedResponse = 0x12,
    cmdSetMaxSpeed = 0x13,
    cmdGoToAzElResponse = 0x14,
    cmdSetAzElResponse = 0x15,

    cmdOkResponse = 0xfd,
    cmdErrorResponse = 0xfe,
    cmdLast = 0xff
};

struct CommandPayloadPing {
    uint32_t data;
};

struct CommandPayloadPong {
    uint32_t data;
};

struct CommandPayloadReadAzEl {
};

struct CommandPayloadReadAzElResponse {
    float az;
    float el;
};

struct CommandPayloadGoToAzEl {
    float az;
    float el;
};

struct CommandPayloadSetAzEl {
    float az;
    float el;
};

struct CommandPayloadEmergencyStop {
};

struct CommandPayloadReadDateTime {
};

struct CommandPayloadReadDateTimeResponse {
    uint32_t timestamp;
};

struct CommandPayloadSetDateTime {
    uint32_t timestamp;
};

struct CommandPayloadReadEEPROM {
    uint8_t address;
    uint8_t size;
};

struct CommandPayloadReadEEPROMResponse {
    uint8_t data[6];
};

struct CommandPayloadWriteEEPROM {
    uint8_t address;
    uint8_t size;
    uint8_t data[4];
};

struct CommandPayloadSetAuxOutput {
    uint8_t state;
};

struct CommandPayloadOkResponse {
};

struct CommandPayloadErrorResponse {
    uint32_t code;
};

struct CommandReadEncoders {
};

struct CommandReadEncodersResponse {
    uint16_t az;
    uint16_t el;
};

struct CommandPayloadReadMaxSpeed {
    float az;
    float el;
};

struct CommandPayloadSetMaxSpeed {
    float az;
    float el;
};


union CommandPayload {
    CommandPayloadPing ping;
    CommandPayloadPong pong;
    CommandPayloadReadAzEl readAzEl;
    CommandPayloadReadAzElResponse readAzElResponse;
    CommandPayloadGoToAzEl goToAzEl;
    CommandPayloadEmergencyStop emergencyStop;
    CommandPayloadReadDateTime readDateTime;
    CommandPayloadReadDateTimeResponse readDateTimeResponse;
    CommandPayloadSetDateTime setDateTime;
    CommandPayloadReadEEPROM readEEPROM;
    CommandPayloadReadEEPROMResponse readEEPROMResponse;
    CommandPayloadWriteEEPROM writeEEPROM;
    CommandPayloadSetAuxOutput setAuxOutput;
    CommandPayloadOkResponse okResponse;
    CommandPayloadErrorResponse errorResponse;
    CommandReadEncoders readEncoders;
    CommandReadEncodersResponse readEncodersResponse;
    CommandPayloadSetAzEl setAzEl;
    CommandPayloadReadMaxSpeed readMaxSpeed;
    CommandPayloadSetMaxSpeed setMaxSpeed;
};

/**
 * @brief size 15 bytes
 */
struct CommandPacket {
    uint32_t header; /*4 */
    uint8_t /*ProtocolCommand*/ command;
    CommandPayload payload; /* 8 */
    uint16_t crc; /* 2 */
};

#pragma pack()
#endif //QSRC_STM32_PROTOCOL_H
