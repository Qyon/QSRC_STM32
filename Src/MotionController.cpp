//
// Created by Admin on 2017-04-01.
//

#include <stm32f1xx_hal_gpio.h>
#include <cmath>
#include <cstdlib>
#include "MotionController.h"


void MotionController::onTimer() {
    if (position_current != position_destination || speed_current){
        int32_t distance = position_destination - position_current;
        float speed_m;
        if (distance){
            speed_m = acc_max * sqrtf((2.0f*fabsf(distance)) / acc_max);
        } else {
            speed_m = 0;
        }
        if (distance < 0){
            speed_m = -speed_m;
        }
        float speed_delta = speed_m - speed_current;
        float time_delta = 0.1f;
        if (speed_current){
            time_delta = fabsf(1.0f / speed_current);
        }
        float acc_m = fabsf(speed_delta) / time_delta;
        if (acc_m > acc_max){
            if (speed_delta > 0){
                speed_delta = acc_max * time_delta;
            } else {
                speed_delta = -acc_max * time_delta;
            }
        }
        speed_current += speed_delta;
        if (fabsf(speed_current) > speed_max){
            if (speed_current > 0){
                speed_current = speed_max;
            } else {
                speed_current = - speed_max;
            }
        }

        if (speed_current > 0){
            position_current ++;
            HAL_GPIO_WritePin(gpio, dir_pin, GPIO_PIN_SET);
        }
        if (speed_current < 0){
            position_current --;
            HAL_GPIO_WritePin(gpio, dir_pin, GPIO_PIN_RESET);
        }
        HAL_GPIO_TogglePin(gpio, step_pin);
        HAL_GPIO_TogglePin(gpio, step_pin);

        uint16_t time = (uint16_t) ((1.0f / fabsf(speed_current)) * 40000);
        if (time > 16000){
            time = 16000;
        } else  if (time < 1){
            time = 1;
        }
        __HAL_TIM_SET_AUTORELOAD(htim, time);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    } else {
        HAL_GPIO_WritePin(gpio, enable_pin, GPIO_PIN_RESET);
        HAL_TIM_Base_Stop_IT(htim);
        HAL_TIM_Base_Stop(htim);
        running = 0;
    }
}

MotionController::MotionController(TIM_HandleTypeDef *htim, GPIO_TypeDef *gpio, uint16_t step_pin, uint16_t dir_pin, uint16_t enable_pin) : gpio(
    gpio), step_pin(step_pin), dir_pin(dir_pin), enable_pin(enable_pin), htim(htim) {
    acc_max = degreesToSteps(DEGREES_ACC_MAX);
}

void MotionController::moveTo(float angle) {
    position_destination = degreesToSteps(angle);
    if (!running){
        HAL_GPIO_WritePin(gpio, enable_pin, GPIO_PIN_SET);
        running = 1;
        __HAL_TIM_SET_AUTORELOAD(htim, 1);
        HAL_TIM_Base_Start_IT(htim);
        HAL_TIM_Base_Start(htim);
    }
}

uint32_t MotionController::degreesToSteps(float degrees) {
    return (uint32_t) ((degrees / 360.0f) * STEPS_PER_ROTATION);
}

bool MotionController::isRunning() {
    return running == 1;
}
