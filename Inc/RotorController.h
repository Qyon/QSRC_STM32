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

    bool debug_enabled = true;
public:
    RotorController(UART_HandleTypeDef *comm_uart, UART_HandleTypeDef *dbg_uart, SPI_HandleTypeDef *encoder_spi,
                        GPIO_TypeDef *encoder_az_gpio, uint16_t encoder_az_pin, GPIO_TypeDef *encoder_el_gpio,
                        uint16_t encoder_el_pin, MotionController *az_mc, MotionController *el_mc);

    void loop();

    void debug(const char *string);

    void send_serial(UART_HandleTypeDef *uart_handle, const uint8_t *string, size_t len);

    void send_respose_status();

    void getRot2ProgAngle(float angle, uint8_t * angle_response);

    void process_set_command(Rot2ProgCmd *pCmd);

    float readRot2ProgAngle(uint8_t angle_data[4], uint8_t resolution);
};


#endif //QSRC_STM32_ROTORCONTROLLER_H
