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

static bool isInit = false;
static TaskHandle_t xHandle = NULL;

const uint32_t baudrate = 115200;
bool motor_ready = false;

uint8_t tempbuffer;
uint8_t motorQE[6];
uint8_t motorTE[6];
static uint8_t enable_foc_motor = 0;

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

float get_vq()
{
    return motor_current_t.a;
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
    while (motor_angle_t.bytes[0] != 'M' || motor_angle_t.bytes[1] != 'R')
    {
        uart2GetData(4, motor_angle_t.bytes);
        vTaskDelay(M2T(100));
    }
    motor_ready = true;
    DEBUG_PRINT("motor ready.\n");

    // main loop
    while (1)
    {   
        vTaskDelay(M2T(2));

        if (enable_foc_motor)
            send_foc_target(get_leg_angle());
        else
            disable_foc();

        uart2GetCharWithTimeout(&tempbuffer, M2T(100));
        if (tempbuffer == 'A')
        {
            uart2GetDataWithTimeout(4, motor_angle_t.bytes, M2T(100));
            uart2GetDataWithTimeout(4, motor_current_t.bytes, M2T(100));
        }
    }
}

static void focInit(DeckInfo *info)
{
    if (isInit)
        return;

    DEBUG_PRINT("Initialize.\n");

    xTaskCreate(focTask, "FOC_TASK",
                configMINIMAL_STACK_SIZE, NULL, 1, &xHandle);

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
PARAM_GROUP_STOP(foc_motor)

LOG_GROUP_START(foc_motorlog)
LOG_ADD(LOG_FLOAT, angle, &motor_angle_t.a)
LOG_ADD(LOG_FLOAT, curr, &motor_current_t.a)
LOG_GROUP_STOP(foc_motorlog)