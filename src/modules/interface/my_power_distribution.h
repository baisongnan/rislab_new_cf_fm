#ifndef __MY_POWER_DISTRIBUTION_H__
#define __MY_POWER_DISTRIBUTION_H__


void power_distribution_calc(float fx, float fy, float fz, float tx, float ty, float tz, uint8_t stop_f);
void arm_force2pwm(float force, const MotorPerifDef* motor_up, const MotorPerifDef* motor_down, uint8_t stop_f);


#endif