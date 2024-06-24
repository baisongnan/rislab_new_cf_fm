/*
 * Driver for the SDM15 ToF sensor
 */
#define DEBUG_MODULE "TOF"

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
#include "uart1.h"

#define PACKET_SIZE 9

static bool isInit = false;
static TaskHandle_t xHandle = NULL;
static uint16_t tof_distance = 0;
static float tof_frequency = 0.0f;

uint8_t calculateChecksum(const uint8_t *packet, uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len - 1; i++)
    {
        checksum += packet[i];
    }
    return checksum;
}

bool isPacketSame(const uint8_t *packet1, const uint8_t *packet2, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        if (packet1[i] != packet2[i])
            return false;
    }
    return true;
}

void uart1Send(uint8_t *packet, uint8_t len)
{
    packet[len - 1] = calculateChecksum(packet, len);
    uint32_t size = len * sizeof(uint8_t);
    uart1SendData(size, packet);
    // for (uint8_t i = 0; i < len; i++)
    // {
    //     uart1Putchar(packet[i]);
    // }
}

void uart1Read(uint8_t *packet, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        uart1GetDataWithDefaultTimeout(packet + i);
        // uart1Getchar(packet + i);
    }
}

void tofTask(void *param)
{
    const uint32_t baudrate = 460800;
    uint8_t cmdStartMeasure[5] = {0xAA, 0x55, 0x60, 0x00, 0x5F};
    uint8_t cmdStopMeasure[5] = {0xAA, 0x55, 0x61, 0x00, 0x60};
    uint8_t cmdSetFrequency[6] = {0xAA, 0x55, 0x64, 0x01, 0x03, 0x67}; // {00:10, 01:100, 02:200, 03:500, 04:1000, 05:1800} Hz
    // uint8_t repsFreq[6] = {0};
    uint8_t response[PACKET_SIZE] = {0};

    uart1Init(baudrate);
    systemWaitStart();
    DEBUG_PRINT("Baud rate: %d\n", (int)baudrate);

    uart1Send(cmdStopMeasure, 5);
    vTaskDelay(M2T(500));
    // uart1ResetBuffer();

    uart1Send(cmdSetFrequency, 6);
    vTaskDelay(M2T(500));
    // uart1Read(repsFreq, 6);
    // if (!isPacketSame(repsFreq, cmdSetFrequency, 6))
    // {
        // DEBUG_PRINT("Error setting frequency.\n");
        // DEBUG_PRINT("Task deleted.\n");
        // vTaskDelay(M2T(500));
        // vTaskDelete(xHandle);
    // }

    uart1Send(cmdStartMeasure, 5);
    // uart1Read(response, 5);
    uint32_t last_tick = xTaskGetTickCount();
    uint32_t current_tick = xTaskGetTickCount();
    uint32_t n = 0;
    float time_elapsed = 0.0f;
    while (1)
    {
        uart1Read(response, 2);
        if (response[0] == 0xAA && response[1] == 0x55)
        {
            uart1Read(response + 2, PACKET_SIZE - 2);
            if (response[PACKET_SIZE - 1] != calculateChecksum(response, PACKET_SIZE))
            {
                DEBUG_PRINT("Error checksum.\n");
                continue;
            }

            tof_distance = (response[5] << 8) | response[4];

            n++;
            current_tick = xTaskGetTickCount();
            time_elapsed = ((float)(current_tick - last_tick)) / configTICK_RATE_HZ;
            if (time_elapsed > 1.0f)
            {
                tof_frequency = n / time_elapsed;
                last_tick = current_tick;
                n = 0;
                // DEBUG_PRINT("Frequency: %d Hz, Distance: %d mm\n", (int)tof_frequency, tof_distance);
            }
        }
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

static const DeckDriver tof_deck = {
    .vid = 0,
    .pid = 0,
    .name = "tofsensor",
    .usedGpio = 0,
    .usedPeriph = DECK_USING_UART1,
    .init = tofInit,
    .test = tofTest,
};

DECK_DRIVER(tof_deck);

LOG_GROUP_START(tof)
LOG_ADD(LOG_UINT16, distance, &tof_distance)
LOG_ADD(LOG_FLOAT, frequency, &tof_frequency)
LOG_GROUP_STOP(tof)