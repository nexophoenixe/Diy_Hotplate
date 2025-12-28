#include "pid.h"

void pi_begin(PI_f *pi_f_x, float kp, float ki, float scale, float clamp_max, float clamp_min, float i_mem) {
  pi_f_x->clamp_max = clamp_max;
  pi_f_x->clamp_min = clamp_min;
  pi_f_x->kp = kp;
  pi_f_x->ki = ki;
  pi_f_x->scale = scale;
  pi_f_x->i_mem = i_mem;
}

void pid_begin(PID_f *pid_f_x, float kp, float ki, float kd, float clamp_max, float clamp_min,float previous_measurement, float i_mem) {
  pid_f_x->clamp_max = clamp_max;
  pid_f_x->clamp_min = clamp_min;
  pid_f_x->kp = kp;
  pid_f_x->ki = ki;
  pid_f_x->kd = kd;
  pid_f_x->previous_measurement = previous_measurement;
  pid_f_x->i_mem = i_mem;
  pid_f_x->output = 0;
}

float pi_series(float command, float current, float kp, float ki, float *i_mem, float clamp_max, float clamp_min,
    float elapsed_time) { //TODO: test
  float error = command - current;
  float error_p = error * kp;
  *i_mem = *i_mem + error_p * ki * elapsed_time;
  if (*i_mem > clamp_max) *i_mem = clamp_max;
  if (*i_mem < clamp_min) *i_mem = clamp_min;
  return (*i_mem);
}

float pi_series_t(PI_f *x, float elapsed_time) { //TODO: test
  float error = x->setpoint - x->measurement;
  float error_p = error * x->kp;
  x->i_mem = x->i_mem + error_p * x->ki * elapsed_time;
  if (x->i_mem > x->clamp_max) x->i_mem = x->clamp_max;
  if (x->i_mem < x->clamp_min) x->i_mem = x->clamp_min;
  x->output = x->i_mem;
  return (x->output);
}

float pi_parallel_t(PI_f *x, float elapsed_time) { //TODO: test
  float error = x->setpoint - x->measurement;
  float error_p = error * x->kp * x->scale;
  x->i_mem = x->i_mem + error * x->ki * x->scale * elapsed_time;
  if (x->i_mem > x->clamp_max) x->i_mem = x->clamp_max;
  if (x->i_mem < x->clamp_min) x->i_mem = x->clamp_min;
  x->output = error_p + x->i_mem;
  if (x->output > x->clamp_max) x->output = x->clamp_max;
  if (x->output < x->clamp_min) x->output = x->clamp_min;
  return (x->output);
}

float pid_parallel(float command, float current, float kp, float ki, float kd, float *cur_mem, float *i_mem,
    float clamp_max, float clamp_min, float elapsed_time) { //TODO: test
  float error = command - current;
  float error_p = error * kp;
  *i_mem = *i_mem + error * ki * elapsed_time;
  if (*i_mem > clamp_max) *i_mem = clamp_max;
  if (*i_mem < clamp_min) *i_mem = clamp_min;
  float error_d = -(current - *cur_mem) * kd;
  float output = error_p + *i_mem + error_d;
  if (output > clamp_max) output = clamp_max;
  if (output < clamp_min) output = clamp_min;

  *cur_mem = current;

  return output;
}

float pid_parallel_t(PID_f *x, float elapsed_time) { //TODO: test

  float error = x->setpoint - x->measurement;
  float error_p = error * x->kp;
  x->i_mem = x->i_mem + error * x->ki * elapsed_time;
  if (x->i_mem > x->clamp_max) x->i_mem = x->clamp_max;
  if (x->i_mem < x->clamp_min) x->i_mem = x->clamp_min;
  float error_d = -(x->measurement - x->previous_measurement) * x->kd;
  x->output = error_p + x->i_mem + error_d;
  if (x->output > x->clamp_max) x->output = x->clamp_max;
  if (x->output < x->clamp_min) x->output = x->clamp_min;

  x->previous_measurement = x->measurement;

  return (x->output);
}
