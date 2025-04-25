/**
 * Title: my_power_distribution.c
 * Author: RichJ233
 * Date: 2025-04-12
 * Version: 1.1
 * Include: my_power_distribution.h
 * Description: This file contains the implementation of the PWM interface for controlling motors.
 * License: This code is licensed under the MIT License.
 */

#include "my_pwm.h"
#include "stm32f4xx_tim.h"
#include "my_power_distribution.h"
#include "my_controller.h"

void pwm_test(uint32_t data1, uint32_t data2, uint32_t data3, uint32_t data4, uint32_t data5,
    uint32_t data6, uint32_t data7, uint32_t data8, uint32_t data9, uint32_t data10,
    uint32_t data11, uint32_t data12)
{
    motor_ratio_ctrl(data1, TIM2, 1); // U R +
    motor_ratio_ctrl(data2, TIM2, 2); // F U +
    motor_ratio_ctrl(data3, TIM2, 3); // U L +
    motor_ratio_ctrl(data4, TIM2, 4); // L F +
    motor_ratio_ctrl(data5, TIM4, 3); // F D +
    motor_ratio_ctrl(data6, TIM3, 1); // B D +
    motor_ratio_ctrl(data7, TIM3, 2); // B U +
    motor_ratio_ctrl(data8, TIM9, 1); // D R +
    motor_ratio_ctrl(data9, TIM9, 2); // D L +
    motor_ratio_ctrl(data10, TIM8, 1); // R B +
    motor_ratio_ctrl(data11, TIM13, 1); // L B +
    motor_ratio_ctrl(data12, TIM14, 1); // R F +
}

