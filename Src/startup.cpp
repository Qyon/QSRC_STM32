//
// Created by Admin on 2017-04-01.
//

#include <MotionController.h>
#include <RotorController.h>
#include <spi.h>
#include "startup.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

MotionController az(&htim2, az_enable_GPIO_Port, az_step_Pin, az_dir_Pin, az_enable_Pin, false, 0, 360);
MotionController el(&htim3, el_enable_GPIO_Port, el_step_Pin, el_dir_Pin, el_enable_Pin, true, 0, 90);
RotorController controller(&huart1, &huart1, &hspi1, az_encoder_cs_GPIO_Port, az_encoder_cs_Pin,
                           el_encoder_cs_GPIO_Port, el_encoder_cs_Pin, &az, &el);
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2){
        az.onTimer();
    }
    else if (htim->Instance == TIM3){
        el.onTimer();
    }
}


/*

 */
void startup() {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1){
        controller.loop();
    }
#pragma clang diagnostic pop
}
