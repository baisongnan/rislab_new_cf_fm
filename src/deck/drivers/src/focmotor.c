/*
 * Driver for the SDM15 ToF sensor
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

static bool isInit = false;
static TaskHandle_t xHandle = NULL;

const uint32_t baudrate = 115200;
bool motor_ready = false;
static int16_t FOC_target = 0; // unit: 0.01 rad
static uint8_t FOC_control_mode = 0;
static char data2uart[10] = "T10\r\n";
// static char data_from_uart[4] = "\r\n";

uint8_t tempbuffer;

static union
{
    float a;
    unsigned char bytes[4];
} data_from_uart;

static union
{
    float a;
    unsigned char bytes[4];
} motor_angle;

static union
{
    float a;
    unsigned char bytes[4];
} motor_velocity;
// #define VEL_FILTER
#ifdef VEL_FILTER
static float motor_velocity_f = 0;
static float motor_velocity_f_gain = 0.05;
#endif

static union
{
    float a;
    unsigned char bytes[4];
} motor_voltage_q;

float get_vq()
{
    return motor_voltage_q.a;
}

uint64_t Timestamp = 1;
uint64_t Timestamp_old = 0;
static uint32_t inToOutLatency;

void FOC_send_torque_target(int16_t target)
{
    *data2uart = 'Q';
    char *h = itoa(target, data2uart + 1, 10);
    while (*h != '\0')
        h++;
    *h = '\r';
    h++;
    *h = '\n';
    uart2SendData((h - data2uart + 1), data2uart);
}

void FOC_send_torque_target_callback()
{
    if (motor_ready)
    {
        FOC_send_torque_target(0);
    }
}

void FOC_send_angle_target(int16_t target)
{
    *data2uart = 'A';
    char *h = itoa(target, data2uart + 1, 10);
    while (*h != '\0')
        h++;
    *h = '\r';
    h++;
    *h = '\n';
    uart2SendData((h - data2uart + 1), data2uart);
}

void FOC_send_angle_target_callback()
{
    if (motor_ready)
    {
        FOC_send_angle_target(FOC_target);
        FOC_control_mode = 2;
    }
}

void tofTask(void *param)
{
    data_from_uart.bytes[0] = 'a';
    data_from_uart.bytes[1] = 'a';

    uart2Init(baudrate);
    systemWaitStart();
    if (uart2Test())
        DEBUG_PRINT("UART2 ready.\n");

    // check motor condition
    while (data_from_uart.bytes[0] != 'M' || data_from_uart.bytes[1] != 'R')
    {
        uart2GetData(4, data_from_uart.bytes);
        vTaskDelay(M2T(100));
    }
    motor_ready = true;
    DEBUG_PRINT("motor ready.\n");

    // main loop
    while (1)
    {
        uart2GetData(1, &tempbuffer);
        if (tempbuffer == 'A')
        {
            uart2GetData(4, data_from_uart.bytes);
            uart2GetData(1, &tempbuffer);
            if (data_from_uart.bytes[1] == tempbuffer)
                motor_angle.a = data_from_uart.a;
        }
        else
            uart2GetData(1, &tempbuffer);
        
        uart2GetData(1, &tempbuffer);
        if (tempbuffer == 'V')
        {
            uart2GetData(4, data_from_uart.bytes);
            uart2GetData(1, &tempbuffer);
            if (data_from_uart.bytes[1] == tempbuffer)
                motor_velocity.a = data_from_uart.a;
        }

        uart2GetData(1, &tempbuffer);
        if (tempbuffer == 'Q')
        {
            uart2GetData(4, data_from_uart.bytes);
            uart2GetData(1, &tempbuffer);
            if (data_from_uart.bytes[1] == tempbuffer)
            {
                motor_voltage_q.a = data_from_uart.a;
                
                Timestamp_old = Timestamp;
                Timestamp = usecTimestamp();
                inToOutLatency = Timestamp - Timestamp_old;
            }
        }
#ifdef VEL_FILTER
        motor_velocity_f = motor_velocity_f * (1-motor_velocity_f_gain) + motor_velocity.a * motor_velocity_f_gain;
#endif
    }
}

static void tofInit(DeckInfo *info)
{
    if (isInit)
        return;

    DEBUG_PRINT("Initialize.\n");

    xTaskCreate(tofTask, "TOF_TASK",
                configMINIMAL_STACK_SIZE, NULL, 1, &xHandle);

    isInit = true;
}

static bool tofTest()
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
    .init = tofInit,
    .test = tofTest,
};

DECK_DRIVER(foc_motor);

PARAM_GROUP_START(foc_motor)
PARAM_ADD_WITH_CALLBACK(PARAM_INT16, tg, &FOC_target, &FOC_send_angle_target_callback)
PARAM_ADD_WITH_CALLBACK(PARAM_INT8, mode, &FOC_control_mode, &FOC_send_torque_target_callback)
#ifdef VEL_FILTER
PARAM_ADD(PARAM_FLOAT, mvfg, &motor_velocity_f_gain)
#endif
PARAM_GROUP_STOP(foc_motor)

LOG_GROUP_START(foc_motorlog)
LOG_ADD(LOG_UINT32, in2out, &inToOutLatency)
LOG_ADD(LOG_FLOAT, angle, &motor_angle.a)
LOG_ADD(LOG_FLOAT, vel, &motor_velocity.a)
#ifdef VEL_FILTER
LOG_ADD(LOG_FLOAT, velf, &motor_velocity_f)
#endif
LOG_ADD(LOG_FLOAT, v_q, &motor_voltage_q.a)
LOG_ADD(LOG_FLOAT, tg, &FOC_target)
LOG_GROUP_STOP(foc_motorlog)