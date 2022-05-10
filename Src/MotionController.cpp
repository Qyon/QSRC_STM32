//
// Created by Admin on 2017-04-01.
//

#include <cmath>
#include <cstdlib>
#include <MotionController.h>
#include <climits>
#include <rtc.h>

#include "MotionController.h"


void MotionController::onTimer() {
    int32_t distance = position_destination - position_current;

    if (distance != 0 || speed_current){
        HAL_GPIO_TogglePin(step_gpio, step_pin);
        float speed_m;
        if (distance != 0){
            speed_m = acc_max * sqrtf((2.0f*abs(distance)) / acc_max);
        } else {
            speed_m = 0;
        }
        if (distance < 0){
            speed_m = -speed_m;
        }
        float speed_delta = speed_m - speed_current;
        float time_delta = 0.1f;
        if (speed_current != 0.0f){
            time_delta = fabsf(1.0f / speed_current);
        }
        float acc_m = fabsf(speed_delta) / time_delta;
        if ((distance && acc_m > acc_max) || (!distance && (acc_m > 5*acc_max))){
            if (speed_delta > 0){
                speed_delta = acc_max * time_delta;
            } else {
                speed_delta = -acc_max * time_delta;
            }
        }
        speed_current += speed_delta;

        if (!speed_current && !distance){
            HAL_TIM_Base_Stop_IT(htim);
            HAL_TIM_Base_Stop(htim);
            running = 0;
        } else {
            bool dir_positive = speed_current > 0;
            if (fabsf(speed_current) > speed_max){
                if (dir_positive){
                    speed_current = speed_max;
                } else {
                    speed_current = - speed_max;
                }
            }

            if (dir_positive){
                position_current ++;
                HAL_GPIO_WritePin(dir_gpio, dir_pin, reverse_direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
            } else {
                position_current --;
                HAL_GPIO_WritePin(dir_gpio, dir_pin, reverse_direction ? GPIO_PIN_RESET : GPIO_PIN_SET);
            }

            auto time = (uint32_t) ((500000.0f / fabsf(speed_current)));
            if (time > UINT_MAX){
                time = UINT_MAX;
            } else  if (time < 1){
                time = 1;
            }
            __HAL_TIM_SET_AUTORELOAD(htim, time);

            HAL_GPIO_TogglePin(step_gpio, step_pin);
        }

    } else {
        HAL_TIM_Base_Stop_IT(htim);
        HAL_TIM_Base_Stop(htim);
        running = 0;
    }
}

MotionController::MotionController(TIM_HandleTypeDef *htim, GPIO_TypeDef *step_gpio, uint16_t step_pin,
                                   GPIO_TypeDef *dir_gpio, uint16_t dir_pin, GPIO_TypeDef *enable_gpio,
                                   uint16_t enable_pin, bool reverse_direction, uint16_t angle_minimum,
                                   uint16_t angle_maximum, SPI_HandleTypeDef *tmc2160_spi,
                                   GPIO_TypeDef *tmc2160_gpio, uint16_t tmc2160_pin, uint8 channel,
                                   uint32_t backup_register_address)
    : step_gpio(step_gpio), step_pin(step_pin),
      dir_gpio(dir_gpio), dir_pin(dir_pin),
      enable_gpio(enable_gpio), enable_pin(enable_pin), htim(htim),
      reverse_direction(reverse_direction),
      angle_minimum(angle_minimum),
      angle_maximum(angle_maximum), tmc2160_spi(tmc2160_spi), tmc2160_gpio(tmc2160_gpio), tmc2160_pin(tmc2160_pin),
      backup_register_address(backup_register_address) {
    acc_max = degreesToSteps(DEGREES_ACC_MAX);
    tmc2160_config.channel = channel;
}

/**
 * Move to desired angle
 * @param angle
 */
void MotionController::moveTo(float angle) {
    if (angle < angle_minimum){
        angle = angle_minimum;
    } else if (angle > angle_maximum){
        angle = angle_maximum;
    }
    uint32_t steps = degreesToSteps(angle);
    moveTo(steps);
}

/**
 * Move to desired step
 * @param target_step
 */
void MotionController::moveTo(uint32_t target_step) {
    position_destination = target_step;
    if (running == 0u){
        HAL_GPIO_WritePin(enable_gpio, enable_pin, GPIO_PIN_RESET);
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

float MotionController::getAngle() {
    HAL_RTCEx_BKUPWrite(&hrtc, backup_register_address, position_current);
    return stepsToDegrees(position_current);
}

float MotionController::stepsToDegrees(int32_t steps) {
    return   (360.0f * steps / STEPS_PER_ROTATION);
}

void MotionController::set(float value) {
    uint32_t steps = degreesToSteps(value);
    while(running != 0u);
    position_destination = steps;
    position_current = steps;
}

void MotionController::emergency_stop() {
    this->position_destination = this->position_current;
    this->speed_current = 0;
}

void MotionController::init(bool useBackedValue) {
    if (useBackedValue) {
        set(stepsToDegrees((int32_t)HAL_RTCEx_BKUPRead(&hrtc, backup_register_address)));
    }
    if (tmc2160_spi != nullptr){
        tmc2160_init(&tmc2160, tmc2160_config.channel, &tmc2160_config, tmc2160_defaultRegisterResetState);
        tmc2160_reset(&tmc2160);
        tmc2160.registerResetState[TMC2160_CHOPCONF] = FIELD_SET(tmc2160.registerResetState[TMC2160_CHOPCONF], TMC2160_INTPOL_MASK, TMC2160_INTPOL_SHIFT, 1);
        tmc2160.registerResetState[TMC2160_CHOPCONF] = FIELD_SET(tmc2160.registerResetState[TMC2160_CHOPCONF], TMC2160_MRES_MASK, TMC2160_MRES_SHIFT, 0b0011);
        tmc2160.registerResetState[TMC2160_IHOLD_IRUN] = FIELD_SET(tmc2160.registerResetState[TMC2160_IHOLD_IRUN], TMC2160_IRUN_MASK, TMC2160_IRUN_SHIFT, 18);
        tmc2160.registerResetState[TMC2160_IHOLD_IRUN] = FIELD_SET(tmc2160.registerResetState[TMC2160_IHOLD_IRUN], TMC2160_IHOLD_MASK, TMC2160_IHOLD_SHIFT, 5);
        tmc2160.registerResetState[TMC2160_GCONF] = FIELD_SET(tmc2160.registerResetState[TMC2160_GCONF], TMC2160_EN_PWM_MODE_MASK, TMC2160_EN_PWM_MODE_SHIFT, 1);
        while (tmc2160.config->state != CONFIG_READY) {
            tmc2160_periodicJob(&tmc2160, 0);
        }
    }
}

void MotionController::writeTMCSPI(uint8 *data, size_t length) {
    if (tmc2160_spi != nullptr){
        HAL_GPIO_WritePin(tmc2160_gpio, tmc2160_pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(tmc2160_spi, data, data, static_cast<uint16_t>(length), 1000);
        HAL_GPIO_WritePin(tmc2160_gpio, tmc2160_pin, GPIO_PIN_SET);
    }
}

void MotionController::setMaxSpeed(float max_speed) {
    this->speed_max = max_speed;
}

float MotionController::getMaxSpeed() {
    return this->speed_max;
}
