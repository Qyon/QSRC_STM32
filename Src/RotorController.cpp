//
// Created by Admin on 2017-04-02.
//

#include <cstring>
#include "RotorController.h"

RotorController::RotorController(UART_HandleTypeDef *comm_uart, UART_HandleTypeDef *dbg_uart, SPI_HandleTypeDef *encoder_spi,
                                 GPIO_TypeDef *encoder_az_gpio, uint16_t encoder_az_pin, GPIO_TypeDef *encoder_el_gpio,
                                 uint16_t encoder_el_pin, MotionController *az_mc, MotionController *el_mc)
    : comm_uart(comm_uart),
      dbg_uart(dbg_uart),
      encoder_spi(encoder_spi),
      encoder_az_gpio( encoder_az_gpio),
      encoder_az_pin( encoder_az_pin),
      encoder_el_gpio(encoder_el_gpio),
      encoder_el_pin(encoder_el_pin),
      az(az_mc),
      el(el_mc){

}

void RotorController::loop() {
    az->moveTo(10);
    el->moveTo(10);
    while (az->isRunning() || el->isRunning());
    az->moveTo(0);
    el->moveTo(0);
    while (az->isRunning() || el->isRunning());
    debug("It is alive!!!");
}

void RotorController::debug(const char *string) {
    this->send_serial(this->dbg_uart, string, strlen(string));
}

void RotorController::send_serial(UART_HandleTypeDef *uart_handle, const char *string, size_t len) {
    while(HAL_UART_Transmit_DMA(uart_handle, (uint8_t *) string, (uint16_t) len) != HAL_OK);
}
