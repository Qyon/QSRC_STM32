//
// Created by Admin on 2017-04-02.
//

#ifndef QSRC_STM32_ROTORCONTROLLER_H
#define QSRC_STM32_ROTORCONTROLLER_H
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "MotionController.h"

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
public:
    RotorController(UART_HandleTypeDef *comm_uart, UART_HandleTypeDef *dbg_uart, SPI_HandleTypeDef *encoder_spi,
                        GPIO_TypeDef *encoder_az_gpio, uint16_t encoder_az_pin, GPIO_TypeDef *encoder_el_gpio,
                        uint16_t encoder_el_pin, MotionController *az_mc, MotionController *el_mc);

    void loop();
};


#endif //QSRC_STM32_ROTORCONTROLLER_H
