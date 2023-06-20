/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#else
#define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
static float armLength = 0.046f; // m;
static float thrustToTorque = 0.005964552f;

// thrust = a * pwm^2 + b * pwm
static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;
static uint16_t min_thrust = 3000;

int powerDistributionMotorType(uint32_t id)
{
  return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  return 0;
}

void powerDistributionInit(void)
{
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}


void powerDistribution(const control_t *control, motors_thrust_uncapped_t *motorThrustUncapped)
{
  // 8 propeller robot thrust compensation
  int32_t att[4];
  att[0] = +control->roll - control->pitch + control->yaw;
  att[1] = +control->roll + control->pitch - control->yaw;
  att[2] = -control->roll + control->pitch + control->yaw;
  att[3] = -control->roll - control->pitch - control->yaw;

  // for x-config quadcopter
  int32_t min = att[0];
  for (int mi = 1; mi < 4; mi++)
  {
    if (att[mi] < min)
      min = att[mi];
  }
  int32_t thrust;
  if (control->thrust < -min)
    thrust = -min;
  else
    thrust = (uint32_t)control->thrust;

  if (thrust > 100)
    thrust = thrust + min_thrust;

  att[0] = att[0] + thrust;
  att[1] = att[1] + thrust;
  att[2] = att[2] + thrust;
  att[3] = att[3] + thrust;

  motorThrustUncapped->motors.m1 = limitUint16(att[0]);
  motorThrustUncapped->motors.m2 = limitUint16(att[1]);
  motorThrustUncapped->motors.m3 = limitUint16(att[2]);
  motorThrustUncapped->motors.m4 = limitUint16(att[3]);
  
}


void powerDistributionCap(const motors_thrust_uncapped_t *motorThrustBatCompUncapped, motors_thrust_pwm_t *motorPwm)
{
  // const int32_t maxAllowedThrust = UINT16_MAX;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    motorPwm->list[motorIndex] = motorThrustBatCompUncapped->list[motorIndex];
  }
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_ADD_CORE(PARAM_UINT16, mt, &min_thrust)
PARAM_GROUP_STOP(powerDist)

/**
 * System identification parameters for quad rotor
 */
PARAM_GROUP_START(quadSysId)

PARAM_ADD(PARAM_FLOAT, thrustToTorque, &thrustToTorque)
PARAM_ADD(PARAM_FLOAT, pwmToThrustA, &pwmToThrustA)
PARAM_ADD(PARAM_FLOAT, pwmToThrustB, &pwmToThrustB)

/**
 * @brief Length of arms (m)
 *
 * The distance from the center to a motor
 */
PARAM_ADD(PARAM_FLOAT, armLength, &armLength)
PARAM_GROUP_STOP(quadSysId)
