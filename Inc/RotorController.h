//
// Created by Admin on 2017-04-02.
//

#ifndef QSRC_STM32_ROTORCONTROLLER_H
#define QSRC_STM32_ROTORCONTROLLER_H
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "MotionController.h"
#include <Rot2Prog.h>
#include "ds1307.h"
#include "protocol.h"
extern "C" {
    #include "crc.h"
};

class RotorController {
private:
    UART_HandleTypeDef *comm_uart;
    UART_HandleTypeDef *dbg_uart;
    SPI_HandleTypeDef *encoder_spi;
    GPIO_TypeDef* encoder_az_gpio;
    uint16_t encoder_az_pin;
    GPIO_TypeDef* encoder_el_gpio;
    uint16_t encoder_el_pin;

    MotionController *az;
    MotionController *el;

    GPIO_TypeDef* aux_gpio;
    uint16_t aux_pin;

    CommandPacket cmd_buffer;
    CommandPacket cmd_to_process;

    uint32_t serial_sync;


    bool debug_enabled = true;

    const uint16_t encoder_read_angle_command = 0xffff;
    volatile uint16_t raw_encoder_az;
    volatile uint16_t raw_encoder_el;
    volatile uint16_t raw_encoder_tmp;
    volatile uint16_t * raw_encoder_current = nullptr;
    bool encoder_spi_read = false;
    volatile bool encoder_spi_in_progress = false;

    uint8_t serial_buffer[256];
    uint8_t serial_buffer_debug[256];

    uint32_t tmp = 0;

    void encoderStartSPITransfer();
    void encoderStartSPITransferRead();


    void send_serial(UART_HandleTypeDef *uart_handle, const uint8_t *string, size_t len);

    void send_respose_status();

    void getRot2ProgAngle(float angle, uint8_t * angle_response);

    void process_set_command(Rot2ProgCmd *pCmd);

    float readRot2ProgAngle(uint8_t angle_data[4], uint8_t resolution);

    uint16_t getEncAz();
    uint16_t getEncEl();

    bool validateCommandPacket(CommandPacket *pPacket);
public:
    uint8_t serial_sync_tmp;
    RotorController(UART_HandleTypeDef *comm_uart, UART_HandleTypeDef *dbg_uart,
                        SPI_HandleTypeDef *encoder_spi, GPIO_TypeDef *encoder_az_gpio,
                        uint16_t encoder_az_pin, GPIO_TypeDef *encoder_el_gpio, uint16_t encoder_el_pin,
                        MotionController *az_mc, MotionController *el_mc, GPIO_TypeDef *aux_gpio,
                        uint16_t aux_pin);

    void loop();

    void onSPITxComplete(SPI_HandleTypeDef *pDef);
    void onSPIRxComplete(SPI_HandleTypeDef *pDef);


    volatile uint32_t s;

    void init();

    void onUSARTRxComplete(UART_HandleTypeDef *huart);

    void handleCommand(CommandPacket *pPacket, CommandPacket *pResponse);

    uint16_t getPacketCRC(const CommandPacket *pPacket) const;

    void onUSARTTxComplete(UART_HandleTypeDef *ptr);

    void debug(const char *string);
    void debug(const char *string, const size_t len);
    void debug(const uint32_t value);

};


#endif //QSRC_STM32_ROTORCONTROLLER_H
