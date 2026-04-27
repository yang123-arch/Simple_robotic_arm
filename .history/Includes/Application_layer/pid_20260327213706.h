#ifndef PID_H
#define PID_H

#include "stm32g4xx_hal.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

#define PERIOD 10
// 单位ms
#define UNIT_OF_TIME 1000
#define FREQUENCY (UNIT_OF_TIME / PERIOD)
#define ENCODER_PRESCALER 4
#define REDUCTION_RADIO 20.409f
#define ENCODER_ACCURACY 13
#define WHEEL_SPEED 500

typedef struct {
  float KP;
  float KI;
  float KD;

  float error_current; // 当前误差
  float target_last_time; // 上次目标值
  float t_filter_coeff;//目标滤波系数
  float error_last_time; // 上次误差
  float error_total;     // 积分项

  float pwm_max;       // 正向最大值
  float pwm_min;       // 反向最大值
  float pwm_dead_zone; // pwm死区（绝对值）

  float I_max; // 积分最大值
  float I_min; // 反向积分最大值

  float d_filter_coeff; // 微分项滤波系数（0-1）
  float prev_d_term;    // 上一次的微分项值，用于滤波

} PID_CONTROL;

extern PID_CONTROL pid_motor_1; // 左前电机PID
extern PID_CONTROL pid_motor_2; // 右前电机PID
extern PID_CONTROL pid_motor_3; // 左后电机PID
extern PID_CONTROL pid_motor_4; // 右后电机PID
extern PID_CONTROL pid_motor_PD; // 位置环:回正底盘
extern PID_CONTROL pid_pitch_PD;
extern PID_CONTROL pid_pitch_PI;
extern PID_CONTROL pid_yaw_PD;
extern PID_CONTROL pid_yaw_PI;

void establish_pid(PID_CONTROL *PID, float kp, float ki, float kd, float I_MAX,
              float I_MIN, float PWM_MAX, float PWM_MIN, float PWM_DEADZONE,
              float t_filter_coeff,float D_FILTER_COEFF) ;
float PID_control(float measurement, float PRM_target, PID_CONTROL *pid);
void init_pid_motor(void);

#endif