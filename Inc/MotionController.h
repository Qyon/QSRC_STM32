//
// Created by Admin on 2017-04-01.
//

#ifndef QSRC_STM32_MOTIONCONTROLLER_H
#define QSRC_STM32_MOTIONCONTROLLER_H


#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
static const uint16_t reload_test = 10000;

class MotionController {
private:
    static const uint32_t STEPS_PER_MOTOR_ROTATION = 6400UL;
    static const uint32_t GEAR_RATIO = 80;
    constexpr static const uint32_t STEPS_PER_ROTATION = (STEPS_PER_MOTOR_ROTATION * GEAR_RATIO);
    constexpr static const float MAX_STEPS_PER_SECOND = (const float) (STEPS_PER_ROTATION / 50.0f);
    constexpr static const float DEGREES_ACC_MAX = 0.3f;
    float acc_max;
    float speed_max = MAX_STEPS_PER_SECOND;
    float speed_current = 0;

    uint16_t test = reload_test;
    GPIO_TypeDef* gpio;
    uint16_t step_pin;
    uint16_t dir_pin;
    uint16_t enable_pin;
    TIM_HandleTypeDef *htim;
    bool reverse_direction = false;

    volatile int32_t position_current = 0;
    volatile int32_t position_destination = 0;
    volatile uint8_t running = 0;

    static uint32_t degreesToSteps(float degrees);
public:
    MotionController(TIM_HandleTypeDef *htim, GPIO_TypeDef *gpio, uint16_t step_pin, uint16_t dir_pin, uint16_t enable_pin, bool reverse_direction);
    void onTimer();
    void moveTo(float angle);

    bool isRunning();
};


#endif //QSRC_STM32_MOTIONCONTROLLER_H
