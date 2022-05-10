//
// Created by Admin on 2017-04-02.
//

#include <cstring>
#include <math.h>
#include <rtc.h>
#include "RotorController.h"

DS1307 t;

RotorController::RotorController(UART_HandleTypeDef *comm_uart, UART_HandleTypeDef *dbg_uart,
                                 SPI_HandleTypeDef *encoder_spi, GPIO_TypeDef *encoder_az_gpio,
                                 uint16_t encoder_az_pin, GPIO_TypeDef *encoder_el_gpio, uint16_t encoder_el_pin,
                                 MotionController *az_mc, MotionController *el_mc, GPIO_TypeDef *aux_gpio,
                                 uint16_t aux_pin)
    : comm_uart(comm_uart),
      dbg_uart(dbg_uart),
      encoder_spi(encoder_spi),
      encoder_az_gpio( encoder_az_gpio),
      encoder_az_pin( encoder_az_pin),
      encoder_el_gpio(encoder_el_gpio),
      encoder_el_pin(encoder_el_pin),
      az(az_mc),
      el(el_mc), aux_gpio(aux_gpio), aux_pin(aux_pin) {
}

void RotorController::csEncoder(int encoder) const {
    if (encoder < 0){
        HAL_GPIO_WritePin(encoder_az_gpio, encoder_az_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(encoder_el_gpio, encoder_el_pin, GPIO_PIN_SET);
    } else if (encoder == 0){
        HAL_GPIO_WritePin(encoder_az_gpio, encoder_az_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(encoder_el_gpio, encoder_el_pin, GPIO_PIN_SET);
    }else if (encoder == 1){
        HAL_GPIO_WritePin(encoder_az_gpio, encoder_az_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(encoder_el_gpio, encoder_el_pin, GPIO_PIN_RESET);
    }

}


void RotorController::encoderStartSPITransfer() {
    current_encoder = (~current_encoder) & 1;
    csEncoder(current_encoder);

    if (raw_encoder_error_read_mode[current_encoder] == 1){
        HAL_SPI_Transmit_IT(encoder_spi, (uint8_t *) &encoder_read_error_register_command, 1);
    } else {
        HAL_SPI_Transmit_IT(encoder_spi, (uint8_t *) &encoder_read_angle_command, 1);
    }

    tmp ++;
}


void RotorController::encoderStartSPITransferRead() {
    csEncoder(-1);
    csEncoder(current_encoder);
    raw_encoder_tmp = 0xffff;
    HAL_SPI_Receive_IT(encoder_spi, (uint8_t *) &raw_encoder_tmp, 1);
    tmp ++;
}

void RotorController::encoderEndSPITransferRead() {
    csEncoder(-1);
    if (raw_encoder_tmp & ERROR_BIT){
        raw_encoder_error_read_mode[current_encoder] = 1;
    }
    if (current_encoder == 0){
        if (raw_encoder_error_read_mode[current_encoder] == 1){
            raw_encoder_last_error = raw_encoder_tmp;
            raw_encoder_error_read_mode[current_encoder] = 1;
        } else if (raw_encoder_error_read_mode[current_encoder] == 2){
            raw_encoder_last_error = raw_encoder_tmp;
            raw_encoder_error_read_mode[current_encoder] = 0;
        } else {
            raw_encoder_az_values[raw_encoder_az_values_ptr] = raw_encoder_tmp & 0x3fff;
            raw_encoder_az_values_ptr++;
            if (raw_encoder_az_values_ptr >= ENCODER_AVERAGES){
                raw_encoder_az_values_ptr = 0;
            }
        }
    } else {
        if (raw_encoder_error_read_mode[current_encoder] == 1){
            raw_encoder_last_error = raw_encoder_tmp;
            raw_encoder_error_read_mode[current_encoder] = 1;
        } else if (raw_encoder_error_read_mode[current_encoder] == 2){
            raw_encoder_last_error = raw_encoder_tmp;
            raw_encoder_error_read_mode[current_encoder] = 0;
        }
        else {
            raw_encoder_el_values[raw_encoder_el_values_ptr] = raw_encoder_tmp & 0x3fff;
            raw_encoder_el_values_ptr++;
            if (raw_encoder_el_values_ptr >= ENCODER_AVERAGES){
                raw_encoder_el_values_ptr = 0;
            }
        }
    }
}

char *array_to_str(char * str, int *array, unsigned int n) {
    int r;
    if (n == 0) return 0;
    if (n == 1) r = sprintf(str, "%d", array[0]);
    else        r = sprintf(str, "%d, ", array[0]);
    array_to_str(str + r, array + 1, n - 1);
    return str;
}

void RotorController::loop() {
    static CommandPacket response;
    static uint32_t last_valid_uart_rcv = HAL_GetTick();
    if (uart_has_data != 0){
        uart_has_data = 0;

        last_valid_uart_rcv = HAL_GetTick();
        handleCommand((CommandPacket *) &this->cmd_to_process, &response);
        debug((const char *) &response, sizeof(CommandPacket));
        this->cmd_to_process.header = 0;
        uart_rx_mode = 0;
        HAL_Delay(1);
        HAL_GPIO_WritePin(RTS_GPIO_Port, RTS_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_UART_Transmit(this->comm_uart, (uint8_t *) &response, sizeof(response), 200);
        HAL_Delay(1);
        HAL_GPIO_WritePin(RTS_GPIO_Port, RTS_Pin, GPIO_PIN_RESET);
    }
    if (uart_rx_mode == 0u){
        startUartRx();
    }
    /*
     * If no valid data from controller recived in given amount of time proceed to emergency stop
     */
    if (HAL_GetTick() - last_valid_uart_rcv > MAX_TIME_WITHOUT_VALID_RX){
        this->emergency_stop();
        //HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_SET);
    } else {
        //HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);
        this->emergency_stopped = false;
    }

    if (this->az->isRunning() || this->el->isRunning()){
        HAL_GPIO_WritePin(yellow_led_GPIO_Port, yellow_led_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(yellow_led_GPIO_Port, yellow_led_Pin, GPIO_PIN_RESET);
    }
}

void RotorController::startUartRx() {
    static uint32_t uart_timeout = HAL_GetTick();
    while(HAL_OK != HAL_UART_Receive_IT(comm_uart, const_cast<uint8_t *>(&cmd_in), 1)){
        if (HAL_GetTick() - uart_timeout > 1000){
            uart_rx_mode = 0;
            return;
        }
    }
    uart_rx_mode = 1;
}

void RotorController::debug(const char *string) {
    if (!debug_enabled){
        return;
    }
    this->send_serial(this->dbg_uart, (const uint8_t *) string, strlen(string));
}

void RotorController::debug(const char *string, const size_t len) {
    if (!debug_enabled){
        return;
    }
    this->send_serial(this->dbg_uart, (const uint8_t *) string, len);
}

void RotorController::debug(const uint32_t value) {
    if (!debug_enabled){
        return;
    }
    this->send_serial(this->dbg_uart, (const uint8_t *) &value, sizeof(value));
}

void RotorController::send_serial(UART_HandleTypeDef *uart_handle, const uint8_t *string, size_t len) {
    uint8_t * buffer = serial_buffer;
    if (uart_handle->Instance == dbg_uart->Instance){
        buffer = serial_buffer_debug;
    }
    while((uart_handle->gState == HAL_UART_STATE_BUSY_TX || uart_handle->gState == HAL_UART_STATE_BUSY_TX_RX));
    memset(buffer, 0, len);
    memcpy(buffer, string, len);
    while(HAL_UART_Transmit_DMA(uart_handle, (uint8_t *) buffer, (uint16_t) len) != HAL_OK);
}
Rot2ProgResponse response = {.start_byte = 'W', .azimuth={0,0,0,0}, .azimuth_resolution=10, .elevation={0,0,0,0}, .elevation_resolution=10, .end_byte=' '};

void RotorController::send_respose_status() {
    getRot2ProgAngle(this->az->getAngle(), response.azimuth);
    getRot2ProgAngle(this->el->getAngle(), response.elevation);

    send_serial(this->comm_uart, (const uint8_t *) &response, sizeof(Rot2ProgResponse));
}

void RotorController::getRot2ProgAngle(float angle, uint8_t * angle_response) {
    auto tmp = (uint16_t) ((angle + 360.0f) * 10);
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

void RotorController::onSPITxComplete(SPI_HandleTypeDef *pDef) {
    if (pDef->Instance != this->encoder_spi->Instance){
        return;
    }
    encoderStartSPITransferRead();
}


void RotorController::onSPIRxComplete(SPI_HandleTypeDef *pDef) {
    if (pDef->Instance != this->encoder_spi->Instance){
        return;
    }
    encoderEndSPITransferRead();
}

uint16_t RotorController::getEncAz() {
    return getAverageEncoderValue(const_cast<uint16_t *>(raw_encoder_az_values));
}

uint16_t RotorController::getEncEl() {
    return getAverageEncoderValue(const_cast<uint16_t *>(raw_encoder_el_values));
}

uint16_t RotorController::getAverageEncoderValue(const uint16_t  *rawEncoderValues) {
    uint32_t sum = 0;
    for (int i = 0; i < ENCODER_AVERAGES; ++i) {
        sum += rawEncoderValues[i];
    }
    return sum / ENCODER_AVERAGES;
}

void RotorController::init() {
    bool useBackedValue = true;

    HAL_PWR_EnableBkUpAccess();
    uint32_t rtcExBkupRead = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
    if (rtcExBkupRead != 0xbac0)
    {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xbac0);
        useBackedValue = false;
    }
    HAL_TIM_Base_Start_IT(&htim4);
    az->init(useBackedValue);
    el->init(useBackedValue);

    startUartRx();
    debug("Start!");
}

void RotorController::onUSARTRxComplete(UART_HandleTypeDef *huart) {
    if (huart->Instance != this->comm_uart->Instance){
        return;
    }
    cmd_buffer[cmd_buffer_index] = cmd_in;
    if (this->validateCommandPacket((CommandPacket *) &cmd_buffer)){
        memcpy((void *)&(cmd_to_process), (const void *) &cmd_buffer, sizeof(cmd_to_process));
        memset((void *)&this->cmd_buffer, 0, sizeof(this->cmd_buffer));
        uart_has_data = 1;
        cmd_buffer_index = 0;
    } else {
        cmd_buffer_index++;
        if (cmd_buffer_index >= sizeof(this->cmd_buffer)){
            cmd_buffer_index = sizeof(this->cmd_buffer) - 1;
            for (uint8_t i = 0; i < sizeof(this->cmd_buffer) - 1; ++i) {
                this->cmd_buffer[i] = this->cmd_buffer[i+1];
            }
        }
        if (this->validateCommandPacket((CommandPacket *) &cmd_buffer)){
            memcpy((void *)&(cmd_to_process), (const void *) &cmd_buffer, sizeof(cmd_to_process));
            memset((void *)&this->cmd_buffer, 0, sizeof(this->cmd_buffer));
            uart_has_data = 1;
            cmd_buffer_index = 0;
        } else {
            HAL_UART_Receive_IT(this->comm_uart, const_cast<uint8_t *>(&cmd_in), 1);
        }
    }
}

bool RotorController::validateCommandPacket(CommandPacket *pPacket) {
    if (pPacket->header != packetHeader){
        return false;
    }

    return getPacketCRC(pPacket) == pPacket->crc;
}

uint16_t RotorController::getPacketCRC(const CommandPacket *pPacket) const { return crc16((uint8_t *) pPacket, sizeof(CommandPacket) - sizeof(uint16_t)); }

void RotorController::handleCommand(CommandPacket *pPacket, CommandPacket *pResponse) {
    memset(pResponse, 0, sizeof(CommandPacket));
    pResponse->header = packetHeader;
    pResponse->command = cmdOkResponse;

    switch (pPacket->command){
        case cmdPing:
            pResponse->command = cmdPong;
            break;
        case cmdReadAzEl:
            pResponse->command = cmdReadAzElResponse;
            pResponse->payload.readAzElResponse.az = this->az->getAngle();
            pResponse->payload.readAzElResponse.el = this->el->getAngle();
            break;
        case cmdGoToAzEl:
            this->az->moveTo(pPacket->payload.goToAzEl.az);
            this->el->moveTo(pPacket->payload.goToAzEl.el);
            pResponse->command = cmdGoToAzElResponse;
            pResponse->payload.goToAzEl.az = pPacket->payload.goToAzEl.az;
            pResponse->payload.goToAzEl.el = pPacket->payload.goToAzEl.el;
            break;
        case cmdEmergencyStop:
            this->emergency_stop();
            break;
        case cmdReadDateTime:
            pResponse->command = cmdReadDateTimeResponse;
            pResponse->payload.readDateTimeResponse.timestamp = t.getDateTime().unixtime;
            break;
        case cmdSetDateTime:
            t.setDateTime(pPacket->payload.setDateTime.timestamp);
            break;
        case cmdReadEEPROM:break;
        case cmdWriteEEPROM:break;
        case cmdSetAuxOutput:
            HAL_GPIO_WritePin(aux_gpio, aux_pin, (GPIO_PinState) pPacket->payload.setAuxOutput.state);
            break;
        case cmdReadEncoders:
            pResponse->command = cmdReadEncodersResponse;
            pResponse->payload.readEncodersResponse.az = this->getEncAz();
            pResponse->payload.readEncodersResponse.el = this->getEncAz();
            break;
        case cmdSetAzEl:
            this->az->set(pPacket->payload.setAzEl.az);
            this->el->set(pPacket->payload.setAzEl.el);
            pResponse->command = cmdSetAzElResponse;
            pResponse->payload.setAzEl.az = pPacket->payload.setAzEl.az;
            pResponse->payload.setAzEl.el = pPacket->payload.setAzEl.el;
            break;
        case cmdSetMaxSpeed:
            this->az->setMaxSpeed(pPacket->payload.setMaxSpeed.az);
            this->el->setMaxSpeed(pPacket->payload.setMaxSpeed.el);
            break;
        case cmdReadMaxSpeed:
            pResponse->command = cmdReadMaxSpeedResponse;
            pResponse->payload.readMaxSpeed.az = this->az->getMaxSpeed();
            pResponse->payload.readMaxSpeed.el = this->el->getMaxSpeed();
            break;

        case cmdReadEEPROMResponse:
        case cmdOkResponse:
        case cmdErrorResponse:
        case cmdReadDateTimeResponse:
        case cmdReadAzElResponse:
        case cmdPong:
        case cmdLast:
        default:
            debug("Unknown");
            pResponse->command = cmdErrorResponse;
            break;
    }

    pResponse->crc = getPacketCRC(pResponse);
}

void RotorController::onUSARTTxComplete(UART_HandleTypeDef *huart) {
    if (huart->Instance != this->comm_uart->Instance){
        return;
    }
    HAL_GPIO_WritePin(RTS_GPIO_Port, RTS_Pin, GPIO_PIN_RESET);
}

void RotorController::onUSARTError(UART_HandleTypeDef *huart) {
    if (huart->ErrorCode == HAL_UART_ERROR_FE && cmd_to_process.header) {
        // ignore standard error
    } else if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
        __HAL_UART_CLEAR_OREFLAG(huart);
    } else {
        //this->onTxError(0);
    }
}

void RotorController::emergency_stop() {
    this->az->emergency_stop();
    this->el->emergency_stop();
    this->emergency_stopped = true;
}

void RotorController::writeTMCSPI(uint8 channel, uint8 *data, size_t length) {
    if (channel == 1){
        az->writeTMCSPI(data, length);
    } else {
        el->writeTMCSPI(data, length);
    }
}

float RotorController::getEncAzAngle() {
    return getEncAz();
}

float RotorController::getEncElAngle() {
    return 360.0f * ((float) ((0xffff & ((getEncEl() & 0x3fff) ^ (1 << 13))) - (1 << 13)) / 0x3fff);
}


