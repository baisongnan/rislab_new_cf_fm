/**
 * Title: my_pwm.c
 * Author: RichJ233
 * Date: 2025-04-12
 * Version: 1.1
 * Include: my_pwm.h
 * Description: This file contains the implementation of the PWM interface for controlling motors.
 * * Steps:   1. Include this file in your project.
 * *          2. Call PWM_interface_init() to initialize all PWM channels.
 * *          3-1. Use motor_ratio_ctrl() to set the duty cycle for each motor.
 * *          3-2. OR Use PWM_ratio_ctrl() to set the duty cycle for each PWM Channel.
 * * Pin Map:
 * * PA2 -> TIM9 CH1
 * * PA3 -> TIM9 CH2
 * * PB8 -> TIM4 CH3
 * * PB7 -> TIM4 CH2 // Caustion: Will be deinitialized by cfclient
 * * PB6 -> TIM4 CH1 // Caustion: Will be deinitialized by cfclient
 * * PB5 -> TIM3 CH2
 * * PB4 -> TIM3 CH1
 * * PA6 -> TIM13 CH1
 * * PA7 -> TIM14 CH1
 * * PA5 -> TIM8 CH1N
 * License: This code is licensed under the MIT License.
 * Addtional Information: Cload: CLOAD_CMDS="-w radio://0/80/2M" make cload
 */

#include "my_pwm.h"
#include "motors.h"

#include "stm32fxxx.h"
#include "stm32fxxx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

/**
 * Parameters of the PWM interface
 * PA2->TIM9 CH1
 * PA3->TIM9 CH2
 */
static const MotorPerifDef MOTORS_PA2_T9C1 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_2,
    .gpioPinSource = GPIO_PinSource2,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM9,
    .tim           = TIM9,
    .channel       = 1,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};
static const MotorPerifDef MOTORS_PA3_T9C2 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_3,
    .gpioPinSource = GPIO_PinSource3,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM9,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .tim           = TIM9,
    .channel       = 2,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

//Initialize TIM9 CH1(PA2).
void TIM9C1_init(void)
{
  TIM_Cmd(TIM9, DISABLE);
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

  // Configure the GPIO for the timer output
  //PA2
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = MOTORS_PA2_T9C1.gpioOType;
  GPIO_InitStructure.GPIO_Pin = MOTORS_PA2_T9C1.gpioPin;
  GPIO_Init(MOTORS_PA2_T9C1.gpioPort, &GPIO_InitStructure);

  //Map timers to alternate functions
  //PA2
  GPIO_PinAFConfig(MOTORS_PA2_T9C1.gpioPort, MOTORS_PA2_T9C1.gpioPinSource, MOTORS_PA2_T9C1.gpioAF);
  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_BL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure Output Compare for PWM
  //PA2
  MOTORS_PA2_T9C1.ocInit(TIM9, &TIM_OCInitStructure);
  MOTORS_PA2_T9C1.preloadConfig(TIM9, TIM_OCPreload_Enable);

  // Set the initial duty cycle to thrust 0
  motor_ratio_ctrl(0, TIM9, 1);

  // Start the timer
  TIM_Cmd(TIM9, ENABLE);
}

//Initialize TIM9 CH2(PA3).
void TIM9C2_init(void)
{
  TIM_Cmd(TIM9, DISABLE);
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

  // Configure the GPIO for the timer output
  //PA3
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = MOTORS_PA3_T9C2.gpioOType;
  GPIO_InitStructure.GPIO_Pin = MOTORS_PA3_T9C2.gpioPin;
  GPIO_Init(MOTORS_PA3_T9C2.gpioPort, &GPIO_InitStructure);

  //Map timers to alternate functions
 //PA3
  GPIO_PinAFConfig(MOTORS_PA3_T9C2.gpioPort, MOTORS_PA3_T9C2.gpioPinSource, MOTORS_PA3_T9C2.gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_BL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure Output Compare for PWM
  //PA3
  MOTORS_PA3_T9C2.ocInit(TIM9, &TIM_OCInitStructure);
  MOTORS_PA3_T9C2.preloadConfig(TIM9, TIM_OCPreload_Enable);

  // Set the initial duty cycle to thrust 0
  motor_ratio_ctrl(0, TIM9, 2);

  // Start the timer
  TIM_Cmd(TIM9, ENABLE);
}

/**
 * Parameters of the PWM interface
 * PB8->TIM4 CH3
 * PB7->TIM4 CH2 // Caustion: Will be deinitialized by cfclient
 * PB6->TIM4 CH1 // Caustion: Will be deinitialized by cfclient
 */
static const MotorPerifDef MOTORS_PB8_T4C3 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_8,
    .gpioPinSource = GPIO_PinSource8,
    .gpioAF        = GPIO_AF_TIM4,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .tim           = TIM4,
    .channel       = 3,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};
static const MotorPerifDef MOTORS_PB7_T4C2 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_7,
    .gpioPinSource = GPIO_PinSource7,
    .gpioAF        = GPIO_AF_TIM4,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .tim           = TIM4,
    .channel       = 2,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};
static const MotorPerifDef MOTORS_PB6_T4C1 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_6,
    .gpioPinSource = GPIO_PinSource6,
    .gpioAF        = GPIO_AF_TIM4,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .tim           = TIM4,
    .channel       = 1,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

//Initialize TIM4 CH3(PB8).
void TIM4C3_init(void)
{
  TIM_Cmd(TIM4, DISABLE);
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  // Configure the GPIO for the timer output
  //PB8
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = MOTORS_PB8_T4C3.gpioPin;
  GPIO_Init(MOTORS_PB8_T4C3.gpioPort, &GPIO_InitStructure);

  //Map timers to alternate functions
  //PB8
  GPIO_PinAFConfig(MOTORS_PB8_T4C3.gpioPort, MOTORS_PB8_T4C3.gpioPinSource, MOTORS_PB8_T4C3.gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_BL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_BL_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure Output Compare for PWM
  //PB8
  MOTORS_PB8_T4C3.ocInit(TIM4, &TIM_OCInitStructure);
  MOTORS_PB8_T4C3.preloadConfig(TIM4, TIM_OCPreload_Enable);

  // Set the initial duty cycle to thrust 0
  motor_ratio_ctrl(0, TIM4, 3);

  // Start the timer
  TIM_Cmd(TIM4, ENABLE);
}

//Initialize TIM4 CH2(PB7) CH3(PB6).
void TIM4C2C1_init(void)
{
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //PB7
  GPIO_InitStructure.GPIO_Pin = MOTORS_PB7_T4C2.gpioPin;
  GPIO_Init(MOTORS_PB7_T4C2.gpioPort, &GPIO_InitStructure);
  //PB6
  GPIO_InitStructure.GPIO_Pin = MOTORS_PB6_T4C1.gpioPin;
  GPIO_Init(MOTORS_PB6_T4C1.gpioPort, &GPIO_InitStructure);

  //Map timers to alternate functions
  //PB7
  GPIO_PinAFConfig(MOTORS_PB7_T4C2.gpioPort, MOTORS_PB7_T4C2.gpioPinSource, MOTORS_PB7_T4C2.gpioAF);
  //PB6
  GPIO_PinAFConfig(MOTORS_PB6_T4C1.gpioPort, MOTORS_PB6_T4C1.gpioPinSource, MOTORS_PB6_T4C1.gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_BL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_BL_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure Output Compare for PWM
  //PB7
  MOTORS_PB7_T4C2.ocInit(TIM4, &TIM_OCInitStructure);
  MOTORS_PB7_T4C2.preloadConfig(TIM4, TIM_OCPreload_Enable);
  //PB6
  MOTORS_PB6_T4C1.ocInit(TIM4, &TIM_OCInitStructure); 
  MOTORS_PB6_T4C1.preloadConfig(TIM4, TIM_OCPreload_Enable);

  // Set the initial duty cycle to thrust 0
  motor_ratio_ctrl(0, TIM4, 2);
  motor_ratio_ctrl(0, TIM4, 1);

  // Start the timer
  TIM_Cmd(TIM4, ENABLE);
}

/**
 * Parameters of the PWM interface
 * PB5->TIM3 CH2
 * PB4->TIM3 CH1
 */
static const MotorPerifDef MOTORS_PB5_T3C2 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_5,
    .gpioPinSource = GPIO_PinSource5,
    .gpioAF        = GPIO_AF_TIM3,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .tim           = TIM3,
    .channel       = 2,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};
static const MotorPerifDef MOTORS_PB4_T3C1 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_4,
    .gpioPinSource = GPIO_PinSource4,
    .gpioAF        = GPIO_AF_TIM3,
    .tim           = TIM3,
    .channel       = 1,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

//Initialize TIM3 CH1(PB4).
void TIM3C1_init(void)
{
  TIM_Cmd(TIM3, DISABLE);
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  // Configure the GPIO for the timer output
  //PB4
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = MOTORS_PB4_T3C1.gpioPin;
  GPIO_Init(MOTORS_PB4_T3C1.gpioPort, &GPIO_InitStructure);

  //Map timers to alternate functions
  //PB4
  GPIO_PinAFConfig(MOTORS_PB4_T3C1.gpioPort, MOTORS_PB4_T3C1.gpioPinSource, MOTORS_PB4_T3C1.gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_BL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_BL_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure Output Compare for PWM
  //PB4
  MOTORS_PB4_T3C1.ocInit(TIM3, &TIM_OCInitStructure);
  MOTORS_PB4_T3C1.preloadConfig(TIM3, TIM_OCPreload_Enable);

  // Set the initial duty cycle to 0
  motor_ratio_ctrl(0, TIM3, 1);

  // Start the timer
  TIM_Cmd(TIM3, ENABLE);
}

//Initialize TIM3 CH2(PB5).
void TIM3C2_init(void)
{
  TIM_Cmd(TIM3, DISABLE);
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  // Configure the GPIO for the timer output
  //PB5
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = MOTORS_PB5_T3C2.gpioPin;
  GPIO_Init(MOTORS_PB5_T3C2.gpioPort, &GPIO_InitStructure);

  //Map timers to alternate functions
  //PB5
  GPIO_PinAFConfig(MOTORS_PB5_T3C2.gpioPort, MOTORS_PB5_T3C2.gpioPinSource, MOTORS_PB5_T3C2.gpioAF);
  
  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_BL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_BL_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure Output Compare for PWM
  //PB5
  MOTORS_PB5_T3C2.ocInit(TIM3, &TIM_OCInitStructure);
  MOTORS_PB5_T3C2.preloadConfig(TIM3, TIM_OCPreload_Enable);

  // Set the initial duty cycle to 0
  motor_ratio_ctrl(0, TIM3, 2);

  // Start the timer
  TIM_Cmd(TIM3, ENABLE);
}

/**
 * Parameters of the PWM interface
 * PA6->TIM13 CH1
 */
static const MotorPerifDef MOTORS_PA6_T13C1 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_6,
    .gpioPinSource = GPIO_PinSource6,
    .gpioAF        = GPIO_AF_TIM13,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .tim           = TIM13,
    .channel       = 1,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

//Initialize TIM13 CH1(PA6).
void TIM13C1_init(void)
{
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = MOTORS_PA6_T13C1.gpioPin;
  GPIO_Init(MOTORS_PA6_T13C1.gpioPort, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(MOTORS_PA6_T13C1.gpioPort, MOTORS_PA6_T13C1.gpioPinSource, MOTORS_PA6_T13C1.gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_BL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_BL_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure Output Compare for PWM
  //PA6
  MOTORS_PA6_T13C1.ocInit(TIM13, &TIM_OCInitStructure);
  MOTORS_PA6_T13C1.preloadConfig(TIM13, TIM_OCPreload_Enable);

  // Set the initial duty cycle to 0
  motor_ratio_ctrl(0, TIM13, 1);

  // Start the timer
  TIM_Cmd(TIM13, ENABLE);
}

/**
 * Parameters of the PWM interface
 * PA7->TIM14 CH1
 */
static const MotorPerifDef MOTORS_PA7_T14C1 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_7,
    .gpioPinSource = GPIO_PinSource7,
    .gpioAF        = GPIO_AF_TIM14,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .tim           = TIM14,
    .channel       = 1,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

//Initialize TIM14 CH1(PA7).
void TIM14C1_init(void)
{
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = MOTORS_PA7_T14C1.gpioPin;
  GPIO_Init(MOTORS_PA7_T14C1.gpioPort, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(MOTORS_PA7_T14C1.gpioPort, MOTORS_PA7_T14C1.gpioPinSource, MOTORS_PA7_T14C1.gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_BL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_BL_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure Output Compare for PWM
  //PA6
  MOTORS_PA7_T14C1.ocInit(TIM14, &TIM_OCInitStructure);
  MOTORS_PA7_T14C1.preloadConfig(TIM14, TIM_OCPreload_Enable);

  // Set the initial duty cycle to 0
  motor_ratio_ctrl(0, TIM14, 1);

  // Start the timer
  TIM_Cmd(TIM14, ENABLE);
}


/**
 * Parameters of the PWM interface
 * PA7->TIM8 CH1N
 */
static const MotorPerifDef MOTORS_PA5_T8C1N =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_5,
    .gpioPinSource = GPIO_PinSource5,
    .gpioAF        = GPIO_AF_TIM8,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .tim           = TIM8,
    .channel       = 1,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

//Initialize TIM8 CH1N(PA5).
void TIM8C1N_init(void)
{
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = MOTORS_PA5_T8C1N.gpioPin;
  GPIO_Init(MOTORS_PA5_T8C1N.gpioPort, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(MOTORS_PA5_T8C1N.gpioPort, MOTORS_PA5_T8C1N.gpioPinSource, MOTORS_PA5_T8C1N.gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_BL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Reset);
  TIM_SelectMasterSlaveMode(TIM8, TIM_MasterSlaveMode_Disable);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  // // Configure the Break and Dead Time
  // TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
  // TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
  // TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  // TIM_BDTRInitStructure.TIM_DeadTime = 0;
  // TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  // TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  // TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

  TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);

  // Configure Output Compare for PWM
  MOTORS_PA5_T8C1N.ocInit(TIM8, &TIM_OCInitStructure);
  MOTORS_PA5_T8C1N.preloadConfig(TIM8, TIM_OCPreload_Enable);

  // Set the initial duty cycle to 0
  motor_ratio_ctrl(0, TIM8, 1);

  // Start the timer
  TIM_Cmd(TIM8, ENABLE);
}

/**
 * Parameters of the PWM interface
 * PA1->TIM2 CH2
 * 
 */
static const MotorPerifDef MOTORS_PA15_T2C1 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_15,
    .tim           = TIM2,
    .channel       = 1,
    .setCompare    = TIM_SetCompare1,
};
static const MotorPerifDef MOTORS_PA1_T2C2 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_1,
    .tim           = TIM2,
    .channel       = 2,
    .setCompare    = TIM_SetCompare2,
};
static const MotorPerifDef MOTORS_PB10_T2C3 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_10,
    .tim           = TIM2,
    .channel       = 3,
    .setCompare    = TIM_SetCompare3,
};
static const MotorPerifDef MOTORS_PB11_T2C4 =
{
    .drvType       = BRUSHLESS,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_11,
    .tim           = TIM2,
    .channel       = 4,
    .setCompare    = TIM_SetCompare4,
};

/**
 * Initialize the PWM interface
 * @attention: This function initializes the PWM interface for all motors.
 */
void PWM_interface_init(void)
{
  //Init PA2 TIM9 CH1
  TIM9C1_init();
  //Init PA3 TIM9 CH2
  TIM9C2_init();
  //Init PB8 TIM4 CH3
  TIM4C3_init();
  //Init PB7 TIM4 CH2, PB6 TIM4 CH1
  //Caustion: Will be deinitialized by cfclient
  //TIM4C2C1_init();
  //Init PB4 TIM3 CH1
  TIM3C1_init();
  //Init PB5 TIM3 CH2
  TIM3C2_init();
  //Init PA6 TIM13 CH1
  TIM13C1_init();
  //Init PA7 TIM14 CH1
  TIM14C1_init();
  //Init PA5 TIM8 CH1N
  TIM8C1N_init();
}

/**
 * Set the duty cycle of the PWM signal
 * @param pwm_ratio: The duty cycle ratio (0-65535)
 * @param TIMx: The timer to use
 * @param channel: The channel to use
 */
void PWM_ratio_ctrl(uint32_t pwm_ratio, TIM_TypeDef *TIMx, uint8_t channel)
{
  uint32_t timPeriod;
  float resolution;
  //Set the duty cycle
  if (channel > 4)
    return;
  if (pwm_ratio > 65535)
    pwm_ratio = 65535;

  timPeriod = TIMx->ARR;
  resolution = (float)timPeriod / 65535.0f;
  switch (channel)
  {
    case 1:
      TIMx->CCR1 = (uint32_t)(pwm_ratio * resolution);
      break;
    
    case 2:
      TIMx->CCR2 = (uint32_t)(pwm_ratio * resolution);
    break;  

    case 3: 
      TIMx->CCR3 = (uint32_t)(pwm_ratio * resolution);
    break;

    case 4:
      TIMx->CCR4 = (uint32_t)(pwm_ratio * resolution);
    break;
  }
}

/**
 * Set the duty cycle of the motor signal
 * @param pwm_ratio: The thrust ratio (0-65535)
 * @param TIMx: The timer to use
 * @param channel: The channel to use
 * @attention: thrust is limited in 25%-50% duty cycle
 */
void motor_ratio_ctrl(uint32_t motor_ratio, TIM_TypeDef *TIMx, uint8_t channel)
{
  uint32_t timPeriod;
  float resolution;
  //Set the duty cycle
  if (channel > 4)
    return;
  if (motor_ratio > 65535)
    motor_ratio = 65535;

  timPeriod = TIMx->ARR / 4;
  resolution = (float)timPeriod / 65535.0f;
  switch (channel)
  {
    case 1:
      TIMx->CCR1 = (uint32_t)(motor_ratio * resolution + timPeriod);
      break;
    
    case 2:
      TIMx->CCR2 = (uint32_t)(motor_ratio * resolution + timPeriod);
    break;  

    case 3: 
      TIMx->CCR3 = (uint32_t)(motor_ratio * resolution + timPeriod);
    break;

    case 4:
      TIMx->CCR4 = (uint32_t)(motor_ratio * resolution + timPeriod);
    break;
  }
}

/**
 *          1       2
 * Arm1 (+T4C3  -T2C2) : positive x - point z
 * Arm2 (+T3C1  -T3C2) : negative x - point z
 * Arm3 (+T13C1 -T2C4) : positive y - point x
 * Arm4 (+T8C1N -T14C1): negative y - point x
 * Arm5 (+T2C1  -T2C3) : positive z - point y
 * Arm6 (+T9C1  -T9C2) : negative z - point y
*/
const MotorPerifDef* motors_def[FULL_ACTUATED_PWM_NUM] = {
  &MOTORS_PB8_T4C3,
  &MOTORS_PA1_T2C2,

  &MOTORS_PB4_T3C1,
  &MOTORS_PB5_T3C2,
  
  &MOTORS_PA6_T13C1,
  &MOTORS_PB11_T2C4,

  &MOTORS_PA5_T8C1N,
  &MOTORS_PA7_T14C1,

  &MOTORS_PA15_T2C1,
  &MOTORS_PB10_T2C3,

  &MOTORS_PA2_T9C1,
  &MOTORS_PA3_T9C2,
};


