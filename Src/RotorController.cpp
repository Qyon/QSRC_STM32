//
// Created by Admin on 2017-04-02.
//

#include <cstring>
#include <math.h>
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

//    HAL_Delay(200);
    Rot2ProgCmd cmd;
    if (HAL_UART_Receive(this->comm_uart, (uint8_t *) &cmd, sizeof(Rot2ProgCmd), 100) == HAL_OK){
        switch (cmd.command){
            case Rot2Prog_COMMAND_STATUS:
                this->send_respose_status();
                break;
            case Rot2Prog_COMMAND_SET:
                this->process_set_command(&cmd);
                break;
            case Rot2Prog_COMMAND_STOP:
                break;
            default:
                break;
        }
    }
}

void RotorController::debug(const char *string) {
    this->send_serial(this->dbg_uart, (const uint8_t *) string, strlen(string));
}

void RotorController::send_serial(UART_HandleTypeDef *uart_handle, const uint8_t *string, size_t len) {
    while(HAL_UART_Transmit_DMA(uart_handle, (uint8_t *) string, (uint16_t) len) != HAL_OK);
}
Rot2ProgResponse response = {.start_byte = 'W', .azimuth={0,0,0,0}, .azimuth_resolution=10, .elevation={0,0,0,0}, .elevation_resolution=10, .end_byte=' '};

void RotorController::send_respose_status() {
    getRot2ProgAngle(this->az->getAngle(), response.azimuth);
    getRot2ProgAngle(this->el->getAngle(), response.elevation);

    send_serial(this->comm_uart, (const uint8_t *) &response, sizeof(Rot2ProgResponse));
}

void RotorController::getRot2ProgAngle(float angle, uint8_t * angle_response) {
    uint16_t tmp = ((angle + 360.0f) * 10);
    tmp = (uint16_t) (tmp % 10000);
    angle_response[0] = (uint8_t) (tmp / 1000);
    tmp = (uint16_t) (tmp % 1000);
    angle_response[1] = (uint8_t) (tmp / 100);
    tmp = (uint16_t) (tmp % 100);
    angle_response[2] = (uint8_t) (tmp / 10);
    tmp = (uint16_t) (tmp % 10);
    angle_response[3] = (uint8_t) (tmp % 10);
}

void RotorController::process_set_command(Rot2ProgCmd *pCmd) {
    az->moveTo(readRot2ProgAngle(pCmd->azimuth, pCmd->azimuth_resolution));
    el->moveTo(readRot2ProgAngle(pCmd->elevation, pCmd->elevation_resolution));
}

uint8_t ascii_to_val(uint8_t ascii){
    return (uint8_t) (ascii >= '0' && ascii <= '9' ? ascii - '0' : 0);
}

float RotorController::readRot2ProgAngle(uint8_t *angle_data, uint8_t resolution) {
    uint16_t tmp = 0;
    tmp += ascii_to_val(angle_data[0]) * 1000;
    tmp += ascii_to_val(angle_data[1]) * 100;
    tmp += ascii_to_val(angle_data[2]) * 10;
    tmp += ascii_to_val(angle_data[3]);
    return (((float)tmp) / resolution) - 360.0f;
}
