#include "pid.h"
#include "Define.h"
#include "Motor_DaMiao.h"

PID_CONTROL pid_motor_1; // 左前电机PID
PID_CONTROL pid_motor_2; // 右前电机PID
PID_CONTROL pid_motor_3; // 左后电机PID
PID_CONTROL pid_motor_4; // 右后电机PID
PID_CONTROL pid_motor_PD; 


PID_CONTROL pid_pitch_PD;
PID_CONTROL pid_pitch_PI;
PID_CONTROL pid_yaw_PD;
PID_CONTROL pid_yaw_PI;



uint8_t pid_error = 0;
/**
 * @brief PID参数初始化
 *
 * @param PID
 * @param kp
 * @param ki
 * @param kd
 * @param I_MAX
 * @param I_MIN
 * @param PWM_MAX
 * @param PWM_MIN
 * @param PWM_DEADZONE
 * @param D_FILTER_COEFF
 * @note 参数注释在pid.h
 */
void establish_pid(PID_CONTROL *PID, float kp, float ki, float kd, float I_MAX,
                   float I_MIN, float PWM_MAX, float PWM_MIN, float PWM_DEADZONE,float t_filter_coeff,
                   float D_FILTER_COEFF)
{
  if (PID == NULL)
  {
    return;
  }

  PID->KP = kp;
  PID->KI = ki;
  PID->KD = kd;
  PID->t_filter_coeff = t_filter_coeff;
  PID->target_last_time = 0;

  PID->error_current = PID->error_last_time = PID->error_total =
      PID->prev_d_term = 0.0f;

  PID->I_max = 1.0f * I_MAX;
  PID->I_min = 1.0f * I_MIN;

  PID->pwm_max = PWM_MAX;
  PID->pwm_min = PWM_MIN;
  PID->pwm_dead_zone = PWM_DEADZONE;

  PID->d_filter_coeff = D_FILTER_COEFF;
}

/**
 * @brief 计算PID
 *
 * @param measurement
 * @param PRM_target
 * @param pid
 * @return float
 */
float PID_control(float measurement, float PRM_target, PID_CONTROL *pid)
{
  if (pid == NULL)
  {
    return 0.0f;
  }

  float target = pid->t_filter_coeff * PRM_target +
                 (1 - pid->t_filter_coeff) * pid->target_last_time;
  pid->target_last_time = target;
  pid->error_current = target - measurement;


  float p_out = pid->KP * pid->error_current;

  pid->error_total += pid->error_current * pid->KI;
  if (pid->error_total > pid->I_max)
  {
    pid->error_total = pid->I_max;
  }
  if (pid->error_total < pid->I_min)
  {
    pid->error_total = pid->I_min;
  }

  float d_current_out = pid->KD * (pid->error_current - pid->error_last_time);
  float d_out = pid->d_filter_coeff * d_current_out +
                (1 - pid->d_filter_coeff) * pid->prev_d_term;

  pid->prev_d_term = d_out;
  pid->error_last_time = pid->error_current;

  float output = p_out + pid->error_total + d_out;

  if (output > pid->pwm_max)
  {
    output = pid->pwm_max;
  }
  if (output < pid->pwm_min)
  {
    output = pid->pwm_min;
  }
  if (fabs(output) < pid->pwm_dead_zone)
  {
    output = 0.0f;
  }

  return output;
}

void init_pid_motor(void)
{

  establish_pid(&pid_pitch_PD, 1, 0, 1, 5.0f, -5.0f, 15, -15, 0.0f, 0.8f, 0.5f);
  establish_pid(&pid_pitch_PI, 2, 0, 0, 10.0f, -10.0f, M6020_CURRENT_MAX, -M6020_CURRENT_MAX, 0.0f, 0.8f, 0.1f);
  establish_pid(&pid_yaw_PD, 10, 0, 12, 3.0f, -3.0f,  T_MAX, T_MIN, 0.0f, 0.8f, 0.8f);
  establish_pid(&pid_yaw_PI, 5, 0.5, 0, 3.0f, -3.0f, T_MAX, T_MIN, 0.0f, 0.8f, 0.1f);


    establish_pid(&pid_motor_1, 1.0f, 0, 0, 10.0f, -10.0f,
 M3508_CURRENT_MAX, -M3508_CURRENT_MAX, 0.1f, 0.8f, 0.0f);
    establish_pid(&pid_motor_2, 1.0f, 0, 0, 10.0f, -10.0f,
 M3508_CURRENT_MAX, -M3508_CURRENT_MAX, 0.1f, 0.8f, 0.0f);
    establish_pid(&pid_motor_3, 1.0f, 0, 0, 10.0f, -10.0f,
 M3508_CURRENT_MAX, -M3508_CURRENT_MAX, 0.1f, 0.8f, 0.0f);
     establish_pid(&pid_motor_4, 1.0f, 0, 0, 10.0f, -10.0f,
 M3508_CURRENT_MAX, -M3508_CURRENT_MAX, 0.1f, 0.8f,
 0.0f);

            
  establish_pid(&pid_motor_PD, 0.1, 0, 0, 10.0f, -10.0f,2, -2, 0.1f, 0.8f, 0.0f);
 }
