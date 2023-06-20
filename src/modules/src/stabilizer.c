/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#define DEBUG_MODULE "STAB"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"
#include "platform.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "crtp_localization_service.h"
#include "controller.h"
#include "power_distribution.h"
// #include "collision_avoidance.h"
#include "health.h"
// #include "supervisor.h"

#include "estimator.h"
// #include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"
#include "rateSupervisor.h"

static bool isInit;
static bool emergencyStop = false;
// static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

static uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static float attitude_control_limit;
static float idle_thrust;
bool thrust_flag;

static motors_thrust_uncapped_t motorThrustUncapped;
static motors_thrust_uncapped_t motorThrustBatCompUncapped;
static motors_thrust_pwm_t motorPwm;

static StateEstimatorType estimatorType;
static ControllerType controllerType;

static STATS_CNT_RATE_DEFINE(stabilizerRate, 500);
static rateSupervisor_t rateSupervisorContext;
static bool rateWarningDisplayed = false;

static float kp_xy = 6000;
static float kp_xy_temp = 6000;
static float kp_z = 6000;
static float kp_z_temp = 6000;

// static float angle_error_threshold = 1.57f;
// static float angle_error_velocity = 300.0f;

static float kd_xy = 10;
static float kd_z = 10;

static float tau_x_offset = 0.0f;
static float tau_y_offset = 0.0f;
static float tau_z_offset = 0.0f;

static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;

static float omega_x = 0.0f;
static float omega_y = 0.0f;
static float omega_z = 0.0f;

static float qw_desired = 1.0f;
static float qx_desired = 0.0f;
static float qy_desired = 0.0f;
static float qz_desired = 0.0f;

static float qw_desired_delay = 1.0f;
static float qx_desired_delay = 0.0f;
static float qy_desired_delay = 0.0f;
static float qz_desired_delay = 0.0f;

uint32_t timestamp_setpoint = 0;

static float external_loop_freq = 100.0f;
// static uint32_t time_gap_setpoint = 10000;

float limint16(float in)
{
  if (in > 32000.0f)
    return 32000.0f;
  else if (in < -32000.0f)
    return -32000.0f;
  else
    return in;
}

float lim_num(float in, float num)
{
  if (in > num)
    return num;
  else if (in < -num)
    return -num;
  else
    return in;
}

// Function to determine if two quaternions represent the same attitude
bool same_attitude(float w, float x, float y, float z, float w_d, float x_d, float y_d, float z_d)
{
  // Calculate the dot product of the two quaternions
  float dot_product = w * w_d + x * x_d + y * y_d + z * z_d;
  // Check if the absolute value of the dot product is close to 1
  return fabsf(dot_product) > 0.999999f;
}

// void pcontrol(float w, float x, float y, float z, float w_d, float x_d,
//               float y_d, float z_d, float *tau_x, float *tau_y, float *tau_z)
// {
//   if (same_attitude(w, x, y, z, w_d, x_d, y_d, z_d))
//   {
//     *tau_x = 0.0F;
//     *tau_y = 0.0F;
//     *tau_z = 0.0F;
//   }
//   else
//   {
//     float b_temp2_tmp;
//     float temp2_tmp;
//     float wwd;
//     float x2;
//     float xd2;
//     float xxd;
//     float y2;
//     float yd2;
//     float yyd;
//     float z2;
//     float zd2;
//     float zzd;
//     wwd = w * w_d;
//     xxd = x * x_d;
//     yyd = y * y_d;
//     zzd = z * z_d;
//     x2 = x * x;
//     y2 = y * y;
//     z2 = z * z;
//     xd2 = x_d * x_d;
//     yd2 = y_d * y_d;
//     zd2 = z_d * z_d;
//     temp2_tmp = 2.0F * xxd;
//     b_temp2_tmp = 2.0F * wwd;
//     x2 = (((((((((((((((((((-2.0F * x2 * xd2 - x2 * yd2) - x2 * zd2) + x2) -
//                          temp2_tmp * yyd) -
//                         temp2_tmp * zzd) -
//                        b_temp2_tmp * xxd) -
//                       xd2 * y2) -
//                      xd2 * z2) +
//                     xd2) -
//                    2.0F * y2 * yd2) -
//                   y2 * zd2) +
//                  y2) -
//                 2.0F * yyd * zzd) -
//                b_temp2_tmp * yyd) -
//               yd2 * z2) +
//              yd2) -
//             2.0F * z2 * zd2) +
//            z2) -
//           b_temp2_tmp * zzd) +
//          zd2;
//     if (x2 <= 0.0F)
//     {
//       *tau_x = 0.0F;
//       *tau_y = 0.0F;
//       *tau_z = 0.0F;
//     }
//     else
//     {
//       x2 = 2.0F * acosf(((wwd + xxd) + yyd) + zzd) / sqrtf(x2);
//       *tau_x = x2 * (((w * x_d - w_d * x) - y * z_d) + y_d * z);
//       *tau_y = x2 * (((w * y_d - w_d * y) + x * z_d) - x_d * z);
//       *tau_z = x2 * (((w * z_d - w_d * z) - x * y_d) + x_d * y);
//     }
//   }
// }

void pcontrol(float w, float x, float y, float z, float w_d, float x_d,
              float y_d, float z_d, float *tau_x, float *tau_y, float *tau_z)
{

  // if (w*w_d<0.0f)
  // {
  //   w_d = -w_d;
  //   x_d = -x_d;
  //   y_d = -y_d;
  //   z_d = -z_d;
  // }

  if (same_attitude(w, x, y, z, w_d, x_d, y_d, z_d))
  {
    *tau_x = 0.0F;
    *tau_y = 0.0F;
    *tau_z = 0.0F;
  }
  else
  {
    float axang1;
    float axang2;
    float axang3;
    float axang4;
    float rot1;
    float temp_1;

    temp_1 = ((w * w_d + x * x_d) + y * y_d) + z * z_d;

    if (temp_1 == 1.0F)
    {
      /*  axang = [0, 0, 1, 0]; */
      axang1 = 0.0F;
      axang2 = 0.0F;
      axang3 = 1.0F;
      axang4 = 0.0F;
    }
    else
    {

      axang4 = sqrtf(1.0F - temp_1 * temp_1);
      axang1 = (((w * x_d - w_d * x) + y * z_d) - y_d * z) / axang4;
      axang2 = (((w * y_d - w_d * y) - x * z_d) + x_d * z) / axang4;
      axang3 = (((w * z_d - w_d * z) + x * y_d) - x_d * y) / axang4;
      axang4 = 2.0F * acosf(temp_1);
    }

    if (axang4 > 3.1415926535897931f)
    {
      axang4 = 6.28318548F - axang4;
      axang1 = -axang1;
      axang2 = -axang2;
      axang3 = -axang3;
    }
    else if (axang4 < -3.1415926535897931f)
    {
      axang4 += 6.28318548F + axang4;
      axang1 = -axang1;
      axang2 = -axang2;
      axang3 = -axang3;
    }

    rot1 = axang1 * axang4;
    axang1 = axang2 * axang4;
    temp_1 = axang3 * axang4;

    axang4 = (rot1 * x + axang1 * y) + temp_1 * z;
    axang2 = (rot1 * w - temp_1 * y) + axang1 * z;
    axang3 = (axang1 * w + temp_1 * x) - rot1 * z;
    temp_1 = (temp_1 * w - axang1 * x) + rot1 * y;

    *tau_x = ((w * axang2 - y * temp_1) + x * axang4) + z * axang3;
    *tau_y = ((w * axang3 + x * temp_1) + y * axang4) - z * axang2;
    *tau_z = ((w * temp_1 - x * axang3) + y * axang2) + z * axang4;
  }
}

static struct
{
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressed;

// static struct {
//   // position - mm
//   int16_t x;
//   int16_t y;
//   int16_t z;
//   // velocity - mm / sec
//   int16_t vx;
//   int16_t vy;
//   int16_t vz;
//   // acceleration - mm / sec^2
//   int16_t ax;
//   int16_t ay;
//   int16_t az;
// } setpointCompressed;

STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void *param);

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

void eul2quat_my(float yaw, float pitch, float roll,
                 float *w, float *x, float *y, float *z)
{
  float c1c2;
  float c_idx_0;
  float c_idx_1;
  float c_idx_2;
  float s1s2;
  float s_idx_0;
  float s_idx_1;
  float s_idx_2;
  s_idx_0 = yaw / 2.0F;
  s_idx_1 = pitch / 2.0F;
  s_idx_2 = roll / 2.0F;
  c_idx_0 = cosf(s_idx_0);
  s_idx_0 = sinf(s_idx_0);
  c_idx_1 = cosf(s_idx_1);
  s_idx_1 = sinf(s_idx_1);
  c_idx_2 = cosf(s_idx_2);
  s_idx_2 = sinf(s_idx_2);
  c1c2 = c_idx_0 * c_idx_1;
  s1s2 = s_idx_0 * s_idx_1;
  c_idx_0 *= s_idx_1;
  s_idx_1 = s_idx_0 * c_idx_1;
  *w = c1c2 * c_idx_2 + s1s2 * s_idx_2;
  *x = c1c2 * s_idx_2 - s1s2 * c_idx_2;
  *y = c_idx_0 * c_idx_2 + s_idx_1 * s_idx_2;
  *z = s_idx_1 * c_idx_2 - c_idx_0 * s_idx_2;
}

static void compressState()
{
  // stateCompressed.x = state.position.x * 1000.0f;
  // stateCompressed.y = state.position.y * 1000.0f;
  // stateCompressed.z = state.position.z * 1000.0f;

  // stateCompressed.vx = state.velocity.x * 1000.0f;
  // stateCompressed.vy = state.velocity.y * 1000.0f;
  // stateCompressed.vz = state.velocity.z * 1000.0f;

  // stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
  // stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
  // stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;

  stateCompressed.ax = sensorData.acc.x * 9810.0f;
  stateCompressed.ay = sensorData.acc.y * 9810.0f;
  stateCompressed.az = (sensorData.acc.z - 1) * 9810.0f;

  float const q[4] = {
      state.attitudeQuaternion.x,
      state.attitudeQuaternion.y,
      state.attitudeQuaternion.z,
      state.attitudeQuaternion.w};
  stateCompressed.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
}

// static void compressSetpoint()
// {
//   setpointCompressed.x = setpoint.position.x * 1000.0f;
//   setpointCompressed.y = setpoint.position.y * 1000.0f;
//   setpointCompressed.z = setpoint.position.z * 1000.0f;
//   setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
//   setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
//   setpointCompressed.vz = setpoint.velocity.z * 1000.0f;
//   setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
//   setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
//   setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
// }

void stabilizerInit(StateEstimatorType estimator)
{
  if (isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAutoSelect);
  powerDistributionInit();
  motorsInit(platformConfigGetMotorMapping());
  // collisionAvoidanceInit();
  estimatorType = stateEstimatorGetType();
  controllerType = controllerGetType();

  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();
  pass &= motorsTest();
  // pass &= collisionAvoidanceTest();

  return pass;
}

// static void checkEmergencyStopTimeout()
// {
//   if (emergencyStopTimeout >= 0) {
//     emergencyStopTimeout -= 1;
//     if (emergencyStopTimeout == 0) {
//       emergencyStop = true;
//     }
//   }
// }

static void batteryCompensation(const motors_thrust_uncapped_t *motorThrustUncapped, motors_thrust_uncapped_t *motorThrustBatCompUncapped)
{
  float supplyVoltage = pmGetBatteryVoltage();

  for (int motor = 0; motor < STABILIZER_NR_OF_MOTORS; motor++)
  {
    motorThrustBatCompUncapped->list[motor] = motorsCompensateBatteryVoltage(motor, motorThrustUncapped->list[motor], supplyVoltage);
  }
}

static void setMotorRatios(const motors_thrust_pwm_t *motorPwm)
{
  motorsSetRatio(MOTOR_M1, motorPwm->motors.m1);
  motorsSetRatio(MOTOR_M2, motorPwm->motors.m2);
  motorsSetRatio(MOTOR_M3, motorPwm->motors.m3);
  motorsSetRatio(MOTOR_M4, motorPwm->motors.m4);
}

/* The stabilizer loop runs at 1kHz. It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */
static void stabilizerTask(void *param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void *)TASK_STABILIZER_ID_NBR);

  // Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  DEBUG_PRINT("Wait for sensor calibration...\n");

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while (!sensorsAreCalibrated())
  {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

  DEBUG_PRINT("Ready to fly.\n");

  idle_thrust = 1500.0f;

  attitude_control_limit = 1300.0f;
  thrust_flag = true;

  while (1)
  {
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();

    // update sensorData struct (for logging variables)
    sensorsAcquire(&sensorData, tick);

    if (healthShallWeRunTest())
    {
      healthRunTests(&sensorData);
    }
    else
    {
      stateEstimator(&state, tick);
      compressState();
      commanderGetSetpoint(&setpoint, &state);
      // controller(&control, &setpoint, &sensorData, &state, tick);

      // disable P controller when thrust is equal to attitude_control_limit
      if (fabsf(setpoint.thrust - attitude_control_limit) < 10.0f)
      {
        kp_xy_temp = 0.0f;
        kp_z_temp = 0.0f;
      }
      else
      {
        kp_xy_temp = kp_xy;
        kp_z_temp = kp_z;
      }

      if (timestamp_setpoint == setpoint.timestamp)
      {
        // no control input is received
        ;
      }
      else
      {
        // control input is received
        if (setpoint.timestamp > timestamp_setpoint)
          // time_gap_setpoint = setpoint.timestamp - timestamp_setpoint;

          timestamp_setpoint = setpoint.timestamp;

        qw_desired_delay = qw_desired;
        qx_desired_delay = qx_desired;
        qy_desired_delay = qy_desired;
        qz_desired_delay = qz_desired;

        // compute desired quat
        eul2quat_my(setpoint.attitudeRate.yaw * -0.0174532925199433f,
                    setpoint.attitude.pitch * -0.0174532925199433f,
                    setpoint.attitude.roll * 0.0174532925199433f,
                    &qw_desired,
                    &qx_desired,
                    &qy_desired,
                    &qz_desired);

        pcontrol(qw_desired_delay,
                 qx_desired_delay,
                 qy_desired_delay,
                 qz_desired_delay,
                 qw_desired,
                 qx_desired,
                 qy_desired,
                 qz_desired,
                 &omega_x, &omega_y, &omega_z);

        // // desired angular rate in degrees
        omega_x = omega_x * 57.2957795130823f * external_loop_freq;
        omega_y = omega_y * 57.2957795130823f * external_loop_freq;
        omega_z = omega_z * 57.2957795130823f * external_loop_freq;

        omega_x = lim_num(omega_x, 300);
        omega_y = lim_num(omega_y, 300);
        omega_z = lim_num(omega_z, 300);
      }
      
      if (fabsf(setpoint.thrust - idle_thrust) < 10.0f)
      {
        control.thrust = 500.0f;
        control.roll = 0.0f;
        control.pitch = 0.0f;
        control.yaw = 0.0f;
      }
      else if (setpoint.thrust >= 10.0f)
      {

        pcontrol(state.attitudeQuaternion.w,
                 state.attitudeQuaternion.x,
                 state.attitudeQuaternion.y,
                 state.attitudeQuaternion.z,
                 qw_desired,
                 qx_desired,
                 qy_desired,
                 qz_desired,
                 &tau_x, &tau_y, &tau_z);

        // float angle_error = sqrtf(tau_x * tau_x + tau_y * tau_y + tau_z * tau_z);

        // if (angle_error > angle_error_threshold)
        // {
        //   omega_x = (tau_x/angle_error) * angle_error_velocity;
        //   omega_y = (tau_y/angle_error) * angle_error_velocity;
        //   omega_z = (tau_z/angle_error) * angle_error_velocity;
        // }
        // else
        // {
        //   omega_x = 0.0f;
        //   omega_y = 0.0f;
        //   omega_z = 0.0f;
        // }

        tau_x = tau_x + tau_x_offset;
        tau_y = tau_y + tau_y_offset;
        tau_z = tau_z + tau_z_offset;

        control.thrust = setpoint.thrust;
        control.roll = (int16_t)limint16(tau_x * kp_xy_temp + (omega_x - sensorData.gyro.x) * kd_xy);
        control.pitch = -(int16_t)limint16(tau_y * kp_xy_temp + (omega_y - sensorData.gyro.y) * kd_xy);
        control.yaw = -(int16_t)limint16(tau_z * kp_z + (omega_z - sensorData.gyro.z) * kd_z);
      }
      else
      {
        control.thrust = 0.0f;
        control.roll = 0.0f;
        control.pitch = 0.0f;
        control.yaw = 0.0f;
      }

      if (emergencyStop || (systemIsArmed() == false))
      {
        motorsStop();
      }
      else
      {
        powerDistribution(&control, &motorThrustUncapped);
        batteryCompensation(&motorThrustUncapped, &motorThrustBatCompUncapped);
        powerDistributionCap(&motorThrustBatCompUncapped, &motorPwm);
        setMotorRatios(&motorPwm);
      }

      calcSensorToOutputLatency(&sensorData);
      tick++;
      STATS_CNT_RATE_EVENT(&stabilizerRate);

      if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount()))
      {
        if (!rateWarningDisplayed)
        {
          DEBUG_PRINT("WARNING: stabilizer loop rate is off (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
          rateWarningDisplayed = true;
        }
      }
    }
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
    motorsBurstDshot();
#endif
  }
}

/**
 * Parameters to set the estimator and controller type
 * for the stabilizer module, or to do an emergency stop
 */
PARAM_GROUP_START(stabilizer)
/**
 * @brief Estimator type Auto select(0), complementary(1), extended kalman(2), **unscented kalman(3)  (Default: 0)
 *
 * ** Experimental, needs to be enabled in kbuild
 */
PARAM_ADD_CORE(PARAM_UINT8, estimator, &estimatorType)
/**
 * @brief Controller type Auto select(0), PID(1), Mellinger(2), INDI(3), Brescianini(4) (Default: 0)
 */
PARAM_ADD_CORE(PARAM_UINT8, controller, &controllerType)
/**
 * @brief If set to nonzero will turn off power
 */
PARAM_ADD_CORE(PARAM_UINT8, stop, &emergencyStop)

PARAM_ADD(PARAM_FLOAT, acl, &attitude_control_limit)

PARAM_ADD(PARAM_FLOAT, kpxy, &kp_xy)
PARAM_ADD(PARAM_FLOAT, kpz, &kp_z)
PARAM_ADD(PARAM_FLOAT, kdxy, &kd_xy)
PARAM_ADD(PARAM_FLOAT, kdz, &kd_z)
PARAM_ADD(PARAM_FLOAT, exfreq, &external_loop_freq)

// PARAM_ADD(PARAM_FLOAT, aet, &angle_error_threshold)
// PARAM_ADD(PARAM_FLOAT, aev, &angle_error_velocity)

PARAM_ADD(PARAM_FLOAT, qxo, &tau_x_offset)
PARAM_ADD(PARAM_FLOAT, qyo, &tau_y_offset)
PARAM_ADD(PARAM_FLOAT, qzo, &tau_z_offset)

PARAM_GROUP_STOP(stabilizer)

/**
 * Log group for the current controller target
 *
 * Note: all members may not be updated depending on how the system is used
 */
LOG_GROUP_START(ctrltarget)

/**
 * @brief Desired position X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &setpoint.position.x)

/**
 * @brief Desired position Y [m]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &setpoint.position.y)

/**
 * @brief Desired position X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &setpoint.position.z)

/**
 * @brief Desired velocity X [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &setpoint.velocity.x)

/**
 * @brief Desired velocity Y [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vy, &setpoint.velocity.y)

/**
 * @brief Desired velocity Z [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vz, &setpoint.velocity.z)

/**
 * @brief Desired acceleration X [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &setpoint.acceleration.x)

/**
 * @brief Desired acceleration Y [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ay, &setpoint.acceleration.y)

/**
 * @brief Desired acceleration Z [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, az, &setpoint.acceleration.z)

/**
 * @brief Desired attitude, roll [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &setpoint.attitude.roll)

/**
 * @brief Desired attitude, pitch [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, pitch, &setpoint.attitude.pitch)

/**
 * @brief Desired attitude rate, yaw rate [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

/**
 * Log group for the current controller target, compressed format.
 * This flavour of the controller target logs are defined with types
 * that use less space and makes it possible to add more logs to a
 * log configuration.
 *
 * Note: all members may not be updated depending on how the system is used
 */

// LOG_GROUP_START(ctrltargetZ)
// /**
//  * @brief Desired position X [mm]
//  */
// LOG_ADD(LOG_INT16, x, &setpointCompressed.x)
// /**
//  * @brief Desired position Y [mm]
//  */
// LOG_ADD(LOG_INT16, y, &setpointCompressed.y)
// /**
//  * @brief Desired position Z [mm]
//  */
// LOG_ADD(LOG_INT16, z, &setpointCompressed.z)
// /**
//  * @brief Desired velocity X [mm/s]
//  */
// LOG_ADD(LOG_INT16, vx, &setpointCompressed.vx)
// /**
//  * @brief Desired velocity Y [mm/s]
//  */
// LOG_ADD(LOG_INT16, vy, &setpointCompressed.vy)
// /**
//  * @brief Desired velocity Z [mm/s]
//  */
// LOG_ADD(LOG_INT16, vz, &setpointCompressed.vz)
// /**
//  * @brief Desired acceleration X [mm/s^2]
//  */
// LOG_ADD(LOG_INT16, ax, &setpointCompressed.ax)
// /**
//  * @brief Desired acceleration Y [mm/s^2]
//  */
// LOG_ADD(LOG_INT16, ay, &setpointCompressed.ay)
// /**
//  * @brief Desired acceleration Z [mm/s^2]
//  */
// LOG_ADD(LOG_INT16, az, &setpointCompressed.az)
// LOG_GROUP_STOP(ctrltargetZ)

/**
 * Logs to set the estimator and controller type
 * for the stabilizer module
 */
LOG_GROUP_START(stabilizer)

// LOG_ADD(LOG_FLOAT, omx, &omega_x)
// LOG_ADD(LOG_FLOAT, omy, &omega_y)
// LOG_ADD(LOG_FLOAT, omz, &omega_z)

// LOG_ADD(LOG_FLOAT, taux, &tau_x)
// LOG_ADD(LOG_FLOAT, tauy, &tau_y)

/**
 * @brief Estimated roll
 *   Note: Same as stateEstimate.roll
 */
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
/**
 * @brief Estimated pitch
 *   Note: Same as stateEstimate.pitch
 */
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
/**
 * @brief Estimated yaw
 *   Note: same as stateEstimate.yaw
 */
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
/**
 * @brief Current thrust
 */
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)
/**
 * @brief Rate of stabilizer loop
 */
STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
/**
 * @brief Latency from sampling of sensor to motor output
 *    Note: Used for debugging but could also be used as a system test
 */
LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
LOG_GROUP_STOP(stabilizer)

/**
 * Log group for accelerometer sensor measurement, based on body frame.
 * Compensated for a miss-alignment by gravity at startup.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what accelerometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(acc)

/**
 * @brief Acceleration in X [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.acc.x)

/**
 * @brief Acceleration in Y [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.acc.y)

/**
 * @brief Acceleration in Z [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

/**
 * Log group for the barometer.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what barometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(baro)

/**
 * @brief Altitude above Sea Level [m]
 */
LOG_ADD_CORE(LOG_FLOAT, asl, &sensorData.baro.asl)

/**
 * @brief Temperature [degrees Celsius]
 */
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)

/**
 * @brief Air preassure [mbar]
 */
LOG_ADD_CORE(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

/**
 * Log group for gyroscopes.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what gyroscope sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(gyro)

/**
 * @brief Angular velocity (rotation) around the X-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.gyro.x)

/**
 * @brief Angular velocity (rotation) around the Y-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.gyro.y)

/**
 * @brief Angular velocity (rotation) around the Z-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

/**
 * Log group for magnetometer.
 *
 * Currently only present on Crazyflie 2.0
 */
LOG_GROUP_START(mag)
/**
 * @brief Magnetometer X axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.mag.x)
/**
 * @brief Magnetometer Y axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.mag.y)
/**
 * @brief Magnetometer Z axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

/**
 * Log group for the state estimator, the currently estimated state of the platform.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
LOG_GROUP_START(stateEstimate)

/**
 * @brief The estimated position of the platform in the global reference frame, X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &state.position.x)

/**
 * @brief The estimated position of the platform in the global reference frame, Y [m]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &state.position.y)

/**
 * @brief The estimated position of the platform in the global reference frame, Z [m]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &state.position.z)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, X [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &state.velocity.x)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Y [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vy, &state.velocity.y)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Z [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vz, &state.velocity.z)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, X [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &state.acc.x)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, Y [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, ay, &state.acc.y)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, without considering gravity, Z [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, az, &state.acc.z)

/**
 * @brief Attitude, roll angle [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &state.attitude.roll)

/**
 * @brief Attitude, pitch angle (legacy CF2 body coordinate system, where pitch is inverted) [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, pitch, &state.attitude.pitch)

/**
 * @brief Attitude, yaw angle [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, yaw, &state.attitude.yaw)

/**
 * @brief Attitude as a quaternion, x
 */
LOG_ADD_CORE(LOG_FLOAT, qx, &state.attitudeQuaternion.x)

/**
 * @brief Attitude as a quaternion, y
 */
LOG_ADD_CORE(LOG_FLOAT, qy, &state.attitudeQuaternion.y)

/**
 * @brief Attitude as a quaternion, z
 */
LOG_ADD_CORE(LOG_FLOAT, qz, &state.attitudeQuaternion.z)

/**
 * @brief Attitude as a quaternion, w
 */
LOG_ADD_CORE(LOG_FLOAT, qw, &state.attitudeQuaternion.w)
LOG_GROUP_STOP(stateEstimate)

/**
 * Log group for the state estimator, compressed format. This flavour of the
 * estimator logs are defined with types that use less space and makes it possible to
 * add more logs to a log configuration.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
LOG_GROUP_START(stateEstimateZ)

/**
 * @brief The position of the Crazyflie in the global reference frame, X [mm]
 */
LOG_ADD(LOG_INT16, x, &stateCompressed.x)

/**
 * @brief The position of the Crazyflie in the global reference frame, Y [mm]
 */
LOG_ADD(LOG_INT16, y, &stateCompressed.y)

/**
 * @brief The position of the Crazyflie in the global reference frame, Z [mm]
 */
LOG_ADD(LOG_INT16, z, &stateCompressed.z)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, X [mm/s]
 */
LOG_ADD(LOG_INT16, vx, &stateCompressed.vx)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Y [mm/s]
 */
LOG_ADD(LOG_INT16, vy, &stateCompressed.vy)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Z [mm/s]
 */
LOG_ADD(LOG_INT16, vz, &stateCompressed.vz)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, X [mm/s]
 */
LOG_ADD(LOG_INT16, ax, &stateCompressed.ax)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, Y [mm/s]
 */
LOG_ADD(LOG_INT16, ay, &stateCompressed.ay)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, including gravity, Z [mm/s]
 */
LOG_ADD(LOG_INT16, az, &stateCompressed.az)

/**
 * @brief Attitude as a compressed quaternion, see see quatcompress.h for details
 */
LOG_ADD(LOG_UINT32, quat, &stateCompressed.quat)

/**
 * @brief Roll rate (angular velocity) [milliradians / sec]
 */
LOG_ADD(LOG_INT16, rateRoll, &stateCompressed.rateRoll)

/**
 * @brief Pitch rate (angular velocity) [milliradians / sec]
 */
LOG_ADD(LOG_INT16, ratePitch, &stateCompressed.ratePitch)

/**
 * @brief Yaw rate (angular velocity) [milliradians / sec]
 */
LOG_ADD(LOG_INT16, rateYaw, &stateCompressed.rateYaw)
LOG_GROUP_STOP(stateEstimateZ)

LOG_GROUP_START(motor)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m1req, &motorThrustBatCompUncapped.motors.m1)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m2req, &motorThrustBatCompUncapped.motors.m2)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m3req, &motorThrustBatCompUncapped.motors.m3)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m4req, &motorThrustBatCompUncapped.motors.m4)
LOG_GROUP_STOP(motor)
