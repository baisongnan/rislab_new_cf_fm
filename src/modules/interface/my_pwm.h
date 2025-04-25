#ifndef __MY_PWM_H__
#define __MY_PWM_H__

#include "motors.h"

#define FULL_ACTUATED_PWM_NUM 12

extern const MotorPerifDef* motors_def[FULL_ACTUATED_PWM_NUM];

/**
 * Pin Map:
 * PA2 -> TIM9 CH1 // Initialization: TIM9C1_init();
 * PA3 -> TIM9 CH2 // Initialization: TIM9C2_init();
 * 
 * PB8 -> TIM4 CH3 // Initialization: TIM4C3_init();
 * PB7 -> TIM4 CH2 // Caustion: Will be deinitialized by cfclient, Initialization: TIM4C2C1_init();
 * PB6 -> TIM4 CH1 // Caustion: Will be deinitialized by cfclient, Initialization: TIM4C2C1_init();
 * 
 * PB5 -> TIM3 CH2 // Initialization: TIM3C2_init();
 * PB4 -> TIM3 CH1 // Initialization: TIM3C1_init();
 * 
 * PA6 -> TIM13 CH1 // Initialization: TIM13C1_init();
 * PA7 -> TIM14 CH1 // Initialization: TIM14C1_init();
 * 
 * PA5 -> TIM8 CH1N // Initialization: TIM8C1N_init();
 */

void PWM_interface_init(void); // Initialize *ALL* PWM channels
void PWM_ratio_ctrl(uint32_t pwm_ratio, TIM_TypeDef *TIMx, uint8_t channel);
void motor_ratio_ctrl(uint32_t motor_ratio, TIM_TypeDef *TIMx, uint8_t channel);

void TIM9C1_init(void);
void TIM9C2_init(void);

void TIM4C3_init(void);
void TIM4C2C1_init(void);

void TIM3C1_init(void);
void TIM3C2_init(void);

void TIM13C1_init(void);

void TIM14C1_init(void);

void TIM8C1N_init(void);

#endif