//
// Created by Admin on 2017-04-01.
//

#ifndef QSRC_STM32_MOTIONCONTROLLER_H
#define QSRC_STM32_MOTIONCONTROLLER_H

#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
extern "C" {
    #include <TMC2160.h>
};
/**
 *
 */
class MotionController {
private:
    static const uint32_t STEPS_PER_MOTOR_ROTATION = 6400UL;
    static const uint32_t GEAR_RATIO = 80;
    constexpr static const uint32_t STEPS_PER_ROTATION = (STEPS_PER_MOTOR_ROTATION * GEAR_RATIO);
    constexpr static const float MAX_STEPS_PER_SECOND = (const float) (STEPS_PER_ROTATION / 50.0f);
    constexpr static const float DEGREES_ACC_MAX = 5.5f;
    float acc_max;
    float speed_max = MAX_STEPS_PER_SECOND;
    float speed_current = 0;

    GPIO_TypeDef* step_gpio;
    uint16_t step_pin;
    GPIO_TypeDef* dir_gpio;
    uint16_t dir_pin;
    GPIO_TypeDef* enable_gpio;
    uint16_t enable_pin;
    TIM_HandleTypeDef *htim;
    bool reverse_direction = false;
    uint16_t angle_minimum = 0;
    uint16_t angle_maximum = 360;

    volatile int32_t position_current = 0;
    volatile int32_t position_destination = 0;
    volatile uint8_t running = 0;

    TMC2160TypeDef tmc2160;
    SPI_HandleTypeDef *tmc2160_spi;
    GPIO_TypeDef* tmc2160_gpio;
    uint16_t tmc2160_pin;
    ConfigurationTypeDef tmc2160_config;

    static uint32_t degreesToSteps(float degrees);
public:
    MotionController(TIM_HandleTypeDef *htim, GPIO_TypeDef *step_gpio, uint16_t step_pin,
                         GPIO_TypeDef *dir_gpio, uint16_t dir_pin, GPIO_TypeDef *enable_gpio,
                         uint16_t enable_pin, bool reverse_direction, uint16_t angle_minimum,
                         uint16_t angle_maximum, SPI_HandleTypeDef *tmc2160_spi,
                         GPIO_TypeDef *tmc2160_gpio, uint16_t tmc2160_pin, uint8 channel);
    void init();
    void onTimer();
    void moveTo(float angle);

    bool isRunning();

    float getAngle();

    float stepsToDegrees(int32_t steps);

    void moveTo(uint32_t target_step);

    void set(float value);

    void emergency_stop();

    void writeTMCSPI(uint8 *data, size_t length);

    void setMaxSpeed(float max_speed);

    float getMaxSpeed();
};


#endif //QSRC_STM32_MOTIONCONTROLLER_H
