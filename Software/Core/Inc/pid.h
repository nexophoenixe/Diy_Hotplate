#include "main.h"

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct PI_f {
  float kp;
  float ki;
  float scale;
  float i_mem;
  float setpoint;
  float measurement;
  float clamp_min;
  float clamp_max;
  float output;
} PI_f;

typedef struct PID_f {
  float kp;
  float ki;
  float kd;
  float i_mem;
  float setpoint;
  float measurement;
  float previous_measurement;
  float clamp_min;
  float clamp_max;
  float output;
} PID_f;

void pi_begin(PI_f *pi_f_x, float kp, float ki, float scale, float clamp_max, float clamp_min, float i_mem); //has to be called at least once at the start

void pid_begin(PID_f *pid_f_x, float kp, float ki, float kd, float clamp_max, float clamp_min,
    float previous_measurement, float i_mem); //has to be called at least once at the start

float pi_series(float command, float current, float kp, float ki, float *i_mem, float clamp_max, float clamp_min,
    float elapsed_time);

float pi_series_t(PI_f *x, float elapsed_time);
float pi_parallel_t(PI_f *x, float elapsed_time);
float pid_parallel(float command, float current, float kp, float ki, float kd, float *cur_mem, float *i_mem,
    float clamp_max, float clamp_min, float elapsed_time);

float pid_parallel_t(PID_f *x, float elapsed_time);

#endif /* INC_PID_H_ */
