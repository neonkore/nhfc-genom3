/*
 * Copyright (c) 2015-2016 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *					Anthony Mallet on Tue Aug 11 2015
 */
#include "acnhfc.h"

#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "nhfc_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */


/** Codel nhfc_main_start of task main.
 *
 * Triggered by nhfc_start.
 * Yields to nhfc_control.
 */
genom_event
nhfc_main_start(nhfc_ids *ids, genom_context self)
{
  ids->servo.sat.x = .1;
  ids->servo.sat.v = .1;

  ids->servo.gain.Kpxy = 25.;
  ids->servo.gain.Kpz = 25.;
  ids->servo.gain.Kqxy = 3.;
  ids->servo.gain.Kqz = .3;
  ids->servo.gain.Kvxy = 12.;
  ids->servo.gain.Kvz = 12.;
  ids->servo.gain.Kwxy = .3;
  ids->servo.gain.Kwz = .03;

  ids->servo.mass = 1.0;

  ids->servo.vmax = 90.;
  ids->servo.vmin = 16.;

  ids->servo.d = 0.25;
  ids->servo.kf = 6.5e-4;
  ids->servo.c = 0.0154;

  nhfc_set_vlimit(ids->servo.vmin, ids->servo.vmax, &ids->servo, self);

  ids->desired.pos._present = false;
  ids->desired.pos_cov._present = false;
  ids->desired.vel._present = false;
  ids->desired.vel_cov._present = false;
  ids->desired.acc._present = false;
  ids->desired.acc_cov._present = false;

  /* init logging */
  ids->log = malloc(sizeof(*ids->log));
  if (!ids->log) abort();
  ids->log->f = NULL;

  return nhfc_control;
}


/** Codel nhfc_main_control of task main.
 *
 * Triggered by nhfc_control.
 * Yields to nhfc_pause_control, nhfc_stop.
 */
genom_event
nhfc_main_control(const nhfc_ids_servo_s *servo,
                  const or_pose_estimator_state *desired,
                  const nhfc_state *state, const nhfc_log_s *log,
                  const nhfc_propeller_input *propeller_input,
                  genom_context self)
{
  const or_pose_estimator_state *state_data;
  or_rotorcraft_input *input_data;
  double thrust, torque[3];
  struct timeval tv;
  double f[4] = { 0., 0., 0., 0. };
  int i, s;

  /* current state */
  if (state->read(self) || !(state_data = state->data(self)))
    goto output;
  gettimeofday(&tv, NULL);
  if (tv.tv_sec + 1e-6 * tv.tv_usec >
      0.5 + state_data->ts.sec + 1e-9 * state_data->ts.nsec)
    goto output;


  /* controller */
  s = nhfc_controller(servo, state_data, desired, log, &thrust, torque);
  if (s) return nhfc_pause_control;

  /* thrust limitation */
  if (thrust < 0.) thrust = 0.;
  if (thrust > 4*servo->fmax) thrust = 4*servo->fmax;

  /* forces */
  f[0] = thrust/4 - torque[1]/servo->d/2. + torque[2]/servo->c/4;
  f[1] = thrust/4 + torque[0]/servo->d/2. - torque[2]/servo->c/4;
  f[2] = thrust/4 + torque[1]/servo->d/2. + torque[2]/servo->c/4;
  f[3] = thrust/4 - torque[0]/servo->d/2. - torque[2]/servo->c/4;

  /* torque limitation */
  if(f[0] < servo->fmin || f[0] > servo->fmax ||
     f[1] < servo->fmin || f[1] > servo->fmax ||
     f[2] < servo->fmin || f[2] > servo->fmax ||
     f[3] < servo->fmin || f[3] > servo->fmax) {
    double kmin = 0.;
    double kmax = 1.;
    double k;

    while(kmax - kmin > 1e-3) {
      k = (kmin + kmax)/2.;
      f[0] = thrust/4 - k * torque[1]/servo->d/2. + k * torque[2]/servo->c/4;
      f[1] = thrust/4 + k * torque[0]/servo->d/2. - k * torque[2]/servo->c/4;
      f[2] = thrust/4 + k * torque[1]/servo->d/2. + k * torque[2]/servo->c/4;
      f[3] = thrust/4 - k * torque[0]/servo->d/2. - k * torque[2]/servo->c/4;

      if(f[0] < servo->fmin || f[0] > servo->fmax ||
         f[1] < servo->fmin || f[1] > servo->fmax ||
         f[2] < servo->fmin || f[2] > servo->fmax ||
         f[3] < servo->fmin || f[3] > servo->fmax)
        kmax = k;
      else
        kmin = k;
    }

    torque[0] *= k;
    torque[1] *= k;
    torque[2] *= k;
  }

  /* output */
output:
  input_data = propeller_input->data(self);
  if (!input_data) return nhfc_pause_control;

  if (state_data) {
    input_data->ts = state_data->ts;
  } else {
    input_data->ts.sec = tv.tv_sec;
    input_data->ts.nsec = tv.tv_usec * 1000;
  }

  input_data->w._length = 4;
  for(i = 0; i < 4; i++)
    input_data->w._buffer[i] = (f[i] > 0.) ? sqrt(f[i]/servo->kf) : 0.;

  propeller_input->write(self);

  return nhfc_pause_control;
}


/** Codel mk_main_stop of task main.
 *
 * Triggered by nhfc_stop.
 * Yields to nhfc_ether.
 */
genom_event
mk_main_stop(const nhfc_propeller_input *propeller_input,
             genom_context self)
{
  or_rotorcraft_input *input_data;
  struct timeval tv;
  int i;

  input_data = propeller_input->data(self);
  if (!input_data) return nhfc_ether;

  gettimeofday(&tv, NULL);
  input_data->ts.sec = tv.tv_sec;
  input_data->ts.nsec = tv.tv_usec * 1000;

  input_data->w._length = 4;
  for(i = 0; i < 4; i++)
    input_data->w._buffer[i] = 0.;

  propeller_input->write(self);
  return nhfc_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel nhfc_servo_main of activity servo.
 *
 * Triggered by nhfc_start.
 * Yields to nhfc_pause_start, nhfc_ether.
 * Throws nhfc_e_input.
 */
FILE *flog;

genom_event
nhfc_servo_main(const nhfc_reference *reference,
                or_pose_estimator_state *desired, genom_context self)
{
  const or_pose_estimator_state *ref_data;

  if (reference->read(self)) return nhfc_e_input(self);
  ref_data = reference->data(self);
  if (!ref_data) return nhfc_e_input(self);

  *desired = *ref_data;
  return nhfc_pause_start;
}


/* --- Activity set_current_position ------------------------------------ */

/** Codel nhfc_set_current_position of activity set_current_position.
 *
 * Triggered by nhfc_start.
 * Yields to nhfc_ether.
 * Throws nhfc_e_input.
 */
genom_event
nhfc_set_current_position(const nhfc_state *state,
                          or_pose_estimator_state *desired,
                          genom_context self)
{
  const or_pose_estimator_state *state_data;
  double qw, qx, qy, qz;
  double yaw;

  if (state->read(self)) return nhfc_e_input(self);
  state_data = state->data(self);
  if (!state_data) return nhfc_e_input(self);
  if (!state_data->pos._present) return nhfc_e_input(self);

  qw = state_data->pos._value.qw;
  qx = state_data->pos._value.qx;
  qy = state_data->pos._value.qy;
  qz = state_data->pos._value.qz;
  yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

  desired->ts = state_data->ts;

  desired->pos._present = true;
  desired->pos._value.x = state_data->pos._value.x;
  desired->pos._value.y = state_data->pos._value.y;
  desired->pos._value.z = state_data->pos._value.z;
  desired->pos._value.qw = cos(yaw/2.);
  desired->pos._value.qx = 0.;
  desired->pos._value.qy = 0.;
  desired->pos._value.qz = sin(yaw/2.);

  desired->vel._present = true;
  desired->vel._value.vx = 0.;
  desired->vel._value.vy = 0.;
  desired->vel._value.vz = 0.;
  desired->vel._value.wx = 0.;
  desired->vel._value.wy = 0.;
  desired->vel._value.wz = 0.;

  desired->acc._present = true;
  desired->acc._value.ax = 0.;
  desired->acc._value.ay = 0.;
  desired->acc._value.az = 0.;

  return nhfc_ether;
}
