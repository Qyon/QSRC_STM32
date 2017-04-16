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
public:
    RotorController(UART_HandleTypeDef *comm_uart, UART_HandleTypeDef *dbg_uart,
                        SPI_HandleTypeDef *encoder_spi, GPIO_TypeDef *encoder_az_gpio,
                        uint16_t encoder_az_pin, GPIO_TypeDef *encoder_el_gpio, uint16_t encoder_el_pin,
                        MotionController *az_mc, MotionController *el_mc, GPIO_TypeDef *aux_gpio,
                        uint16_t aux_pin);

    void loop();

    void debug(const char *string);
    void debug(const char *string, const size_t len);
    void debug(const uint32_t value);

    void send_serial(UART_HandleTypeDef *uart_handle, const uint8_t *string, size_t len);

    void send_respose_status();

    void getRot2ProgAngle(float angle, uint8_t * angle_response);

    void process_set_command(Rot2ProgCmd *pCmd);

    float readRot2ProgAngle(uint8_t angle_data[4], uint8_t resolution);

    void onSPITxComplete(SPI_HandleTypeDef *pDef);
    void onSPIRxComplete(SPI_HandleTypeDef *pDef);

    void encoderStartSPITransfer();
    void encoderStartSPITransferRead();

    volatile uint32_t s;
};


#endif //QSRC_STM32_ROTORCONTROLLER_H
