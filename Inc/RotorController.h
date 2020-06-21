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
    const uint16_t MAX_TIME_WITHOUT_VALID_RX = 1000;
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

    volatile uint8_t cmd_in;
    volatile uint8_t cmd_buffer[sizeof(CommandPacket) + 2];
    volatile uint8_t cmd_buffer_index = 0;

    volatile CommandPacket cmd_to_process;

    volatile uint8_t uart_has_data = 0;
    volatile uint8_t uart_rx_mode = 0;


    bool debug_enabled = false;

    const uint16_t encoder_read_angle_command = 0xffff;
    volatile uint16_t raw_encoder_az;
    volatile uint16_t raw_encoder_el;
    volatile uint16_t raw_encoder_tmp = 0;
    volatile uint16_t * raw_encoder_current = nullptr;
    bool encoder_spi_read = false;
    volatile bool encoder_spi_in_progress = false;
    bool emergency_stopped = false;

    uint8_t serial_buffer[256];
    uint8_t serial_buffer_debug[256];

    uint32_t tmp = 0;

    void encoderStartSPITransferRead();


    void send_serial(UART_HandleTypeDef *uart_handle, const uint8_t *string, size_t len);

    void send_respose_status();

    void getRot2ProgAngle(float angle, uint8_t * angle_response);

    void process_set_command(Rot2ProgCmd *pCmd);

    float readRot2ProgAngle(uint8_t angle_data[4], uint8_t resolution);

    uint16_t getEncAz();
    uint16_t getEncEl();

    bool validateCommandPacket(CommandPacket *pPacket);
    void emergency_stop();
public:
    volatile uint8_t serial_sync_tmp;
    RotorController(UART_HandleTypeDef *comm_uart, UART_HandleTypeDef *dbg_uart,
                        SPI_HandleTypeDef *encoder_spi, GPIO_TypeDef *encoder_az_gpio,
                        uint16_t encoder_az_pin, GPIO_TypeDef *encoder_el_gpio, uint16_t encoder_el_pin,
                        MotionController *az_mc, MotionController *el_mc, GPIO_TypeDef *aux_gpio,
                        uint16_t aux_pin);

    void loop();

    void onSPITxComplete(SPI_HandleTypeDef *pDef);
    void onSPIRxComplete(SPI_HandleTypeDef *pDef);
    void encoderStartSPITransfer();

    volatile uint32_t s;

    void init();

    void onUSARTRxComplete(UART_HandleTypeDef *huart);

    void handleCommand(CommandPacket *pPacket, CommandPacket *pResponse);

    uint16_t getPacketCRC(const CommandPacket *pPacket) const;

    void onUSARTTxComplete(UART_HandleTypeDef *ptr);
    void onUSARTError(UART_HandleTypeDef *huart);

    void debug(const char *string);
    void debug(const char *string, const size_t len);
    void debug(const uint32_t value);

    void startUartRx();

    void writeTMCSPI(uint8 channel, uint8 *data, size_t length);
};


#endif //QSRC_STM32_ROTORCONTROLLER_H
