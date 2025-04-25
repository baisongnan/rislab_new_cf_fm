/**
 * Title: my_power_distribution.c
 * Author: RichJ233
 * Date: 2025-04-20
 * Version: 1.0
 * Include: my_power_distribution.h
 * Description: This file contains the implementation of the PWM interface for controlling motors.
 * License: This code is licensed under the MIT License.
 */

#include "my_pwm.h"
#include "my_power_distribution.h"

static uint16_t min_thrust = 4000;

uint32_t my_limitUint16(uint32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  return value;
}

/**
 * @brief Power distribution calculation
 * @param fx: Force in x direction
 * @param fy: Force in y direction
 * @param fz: Force in z direction
 * @param tx: Torque in x direction
 * @param ty: Torque in y direction
 * @param tz: Torque in z direction
 * @return None
 * @note This function calculates the power distribution for each arm
 * Arm No.:
 *       ^    ^
 *       5   1
 *       z  x
 *       | /
 * <3-y--|----y-4
 *     / |
 *    x  z
 *   2   6
 * Arm1 (+T4C3  -T2C2) : positive x - point z
 * Arm2 (+T3C1  -T3C2) : negative x - point z
 * Arm3 (+T13C1 -T2C4) : positive y - point x
 * Arm4 (+T8C1N -T14C1): negative y - point x
 * Arm5 (+T2C1  -T2C3) : positive z - point y
 * Arm6 (+T9C1  -T9C2) : negative z - point y
 */
void power_distribution_calc(float fx, float fy, float fz, float tx, float ty, float tz)
{
  float force_calc_temp1 = 0;
  float force_calc_temp2 = 0;
  float force_calc_temp3 = 0;
  float force_calc_temp4 = 0;
  float force_calc_temp5 = 0;
  float force_calc_temp6 = 0;

  // Calculate the power distrubution for each arm
  force_calc_temp1 = fz - 0.01002f * tx - 0.10517f * ty + 0.00095f* tz;
  force_calc_temp2 = fz + 0.01002f * tx + 0.10517f * ty - 0.00095f * tz;
  force_calc_temp3 = fx - 0.00095f * tx - 0.01002f * ty - 0.10517f * tz;
  force_calc_temp4 = fx + 0.00095f * tx + 0.01002f * ty + 0.10517f * tz;
  force_calc_temp5 = fy - 0.10517f * tx + 0.00095f * ty + 0.01002f * tz;
  force_calc_temp6 = fy + 0.10517f * tx - 0.00095f * ty - 0.01002f * tz;

  // Apply the calculated forces to the motors
  arm_force2pwm(force_calc_temp1, motors_def[0], motors_def[1]);   // Arm1
  arm_force2pwm(force_calc_temp2, motors_def[2], motors_def[3]);   // Arm2
  arm_force2pwm(force_calc_temp3, motors_def[4], motors_def[5]);   // Arm3
  arm_force2pwm(force_calc_temp4, motors_def[6], motors_def[7]);   // Arm4
  arm_force2pwm(force_calc_temp5, motors_def[8], motors_def[9]);   // Arm5
  arm_force2pwm(force_calc_temp6, motors_def[10], motors_def[11]); // Arm6
}

/**
 * @brief Apply force to arm
 * @param force: The force to be applied to the arm
 * @param motor_up: The motor on the upper side of the arm
 * @param motor_down: The motor on the lower side of the arm
 * @return None
 */
void arm_force2pwm(float force, const MotorPerifDef *motor_up, const MotorPerifDef *motor_down)
{
  // motor_ratio_ctrl((uint32_t)force, motor_up->tim, motor_up->channel);
  // motor_ratio_ctrl((uint32_t)force, motor_down->tim, motor_down->channel);

  if (force >= 0)
  {
    motor_ratio_ctrl((uint32_t)(force+min_thrust), motor_up->tim, motor_up->channel);
    motor_ratio_ctrl((uint32_t)min_thrust, motor_down->tim, motor_down->channel);
  }
  else
  {
    motor_ratio_ctrl((uint32_t)min_thrust, motor_up->tim, motor_up->channel);
    motor_ratio_ctrl((uint32_t)(-force+min_thrust), motor_down->tim, motor_down->channel);
  }
}
