/*
 * Driver for the FOC motor
 */
#define DEBUG_MODULE "FOC"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "param.h"
#include "system.h"
#include "queue.h"
#include "deck.h"
#include "uart2.h"
#include "focmotor.h"
#include "stabilizer.h"

#define VELOCITY_STREAMING
// #define DEBUGING_MODE // show sample time

#ifdef DEBUGING_MODE
uint64_t t = 0;
uint64_t t_delay = 0;
static uint16_t dt = 0;
uint8_t error_flag = 0;
#endif

static bool isInit = false;
static TaskHandle_t xHandle = NULL;

const uint32_t baudrate = 115200;
bool motor_ready = false;

uint8_t tempbuffer;
uint8_t motorQE[6];
uint8_t motorTE[6];
static uint8_t enable_foc_motor = 0;
static uint8_t leg_pitch_absolute = 0;

void foc_enable()
{
    enable_foc_motor = 1;
}

void foc_disable()
{
    enable_foc_motor = 0;
}

uint16_t tick = 0;

static union
{
    float a;
    unsigned char bytes[4];
} data_to_uart;

static union
{
    float a;
    unsigned char bytes[4];
} motor_angle_t;

static union
{
    float a;
    unsigned char bytes[4];
} motor_current_t;

#ifdef VELOCITY_STREAMING
static union
{
    float a;
    unsigned char bytes[4];
} motor_velocity_t;
float get_leg_veloicity()
{
    return motor_velocity_t.a;
}
#endif

float get_vq()
{
    return motor_current_t.a;
}

float foc_get_leg_angle()
{
    return motor_angle_t.a;
}

void send_foc_target(float tg)
{
    data_to_uart.a = tg;
    motorTE[1] = data_to_uart.bytes[0];
    motorTE[2] = data_to_uart.bytes[1];
    motorTE[3] = data_to_uart.bytes[2];
    motorTE[4] = data_to_uart.bytes[3];
    uart2SendData(6, motorTE);
}

void disable_foc()
{
    uart2SendData(6, motorQE);
}

void focTask(void *param)
{
    motorQE[0] = (uint8_t)'Q';
    motorQE[1] = (uint8_t)'E';
    motorQE[2] = (uint8_t)'E';
    motorQE[3] = (uint8_t)'E';
    motorQE[4] = (uint8_t)'E';
    motorQE[5] = (uint8_t)'E';

    motorTE[0] = (uint8_t)'T';
    motorTE[5] = (uint8_t)'E';

    uart2Init(baudrate);
    systemWaitStart();
    if (uart2Test())
        DEBUG_PRINT("UART2 ready.\n");

// check motor condition
#ifdef VELOCITY_STREAMING
    while (motor_angle_t.bytes[0] != 'M' || motor_angle_t.bytes[1] != 'V')
    {
        uart2GetData(4, motor_angle_t.bytes);
        vTaskDelay(M2T(100));
    }
#else
    while (motor_angle_t.bytes[0] != 'M' || motor_angle_t.bytes[1] != 'R')
    {
        uart2GetData(4, motor_angle_t.bytes);
        vTaskDelay(M2T(100));
    }
#endif

    motor_ready = true;
    DEBUG_PRINT("motor ready.\n");

    // main loop
    while (1)
    {
        vTaskDelay(M2T(2));

        if (enable_foc_motor)
        {
            if (leg_pitch_absolute)
            {
                send_foc_target(get_leg_angle() - get_body_pitch());
            }
            else
            {
                send_foc_target(get_leg_angle());
            }
        }
        else
            disable_foc();

        uart2GetCharWithTimeout(&tempbuffer, M2T(100));
        if (tempbuffer == 'A')
        {
            uart2GetDataWithTimeout(4, motor_angle_t.bytes, M2T(100));
            uart2GetDataWithTimeout(4, motor_current_t.bytes, M2T(100));
#ifdef VELOCITY_STREAMING
            uart2GetDataWithTimeout(4, motor_velocity_t.bytes, M2T(100));
#endif

#ifdef DEBUGING_MODE
            error_flag = 0;
#endif
        }
        else
        {
#ifdef DEBUGING_MODE
            error_flag = 1;
#endif
        }

#ifdef DEBUGING_MODE
        t_delay = t;
        t = usecTimestamp();
        dt = t - t_delay;
#endif
    }
}

static void focInit(DeckInfo *info)
{
    if (isInit)
        return;

    DEBUG_PRINT("Initialize.\n");

    xTaskCreate(focTask, "FOC_TASK",
                configMINIMAL_STACK_SIZE, NULL, 2, &xHandle);

    isInit = true;
}

static bool focTest()
{
    if (!isInit)
        return false;

    DEBUG_PRINT("Test passed.\n");

    return true;
}

static const DeckDriver foc_motor = {
    .vid = 0,
    .pid = 0,
    .name = "focmotor",
    .usedGpio = 0,
    .usedPeriph = DECK_USING_UART2,
    .init = focInit,
    .test = focTest,
};

DECK_DRIVER(foc_motor);

PARAM_GROUP_START(foc_motor)
PARAM_ADD(PARAM_UINT8, efm, &enable_foc_motor)
PARAM_ADD(PARAM_UINT8, lpa, &leg_pitch_absolute)
PARAM_GROUP_STOP(foc_motor)

LOG_GROUP_START(foc_motorlog)
LOG_ADD(LOG_FLOAT, angle, &motor_angle_t.a)
LOG_ADD(LOG_FLOAT, curr, &motor_current_t.a)
#ifdef VELOCITY_STREAMING
LOG_ADD(LOG_FLOAT, velo, &motor_velocity_t.a)
#endif
#ifdef DEBUGING_MODE
LOG_ADD(LOG_UINT16, dt, &dt)
LOG_ADD(LOG_UINT8, error_f, &error_flag)
#endif
LOG_GROUP_STOP(foc_motorlog)