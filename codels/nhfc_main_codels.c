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

#include "nhfc_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */


/** Codel nhfc_main_start of task main.
 *
 * Triggered by nhfc_start.
 * Yields to nhfc_ether.
 */
genom_event
nhfc_main_start(nhfc_ids_servo_s *servo, genom_context self)
{
  servo->gain.Kx = 1.;
  servo->gain.Kq = 1.;
  servo->gain.Kv = 1.;
  servo->gain.Kw = 1.;

  servo->mass = 0.950;

  servo->desired.pos._present = true;
  servo->desired.pos._value.x = 0.;
  servo->desired.pos._value.y = 0.;
  servo->desired.pos._value.z = 0.;
  servo->desired.pos._value.qw = 1.;
  servo->desired.pos._value.qx = 0.;
  servo->desired.pos._value.qy = 0.;
  servo->desired.pos._value.qz = 0.;

  servo->pulsedes = servo->raddes = 0.;

  servo->vmax = 90.;
  servo->vmin = 15.;

  return nhfc_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel nhfc_servo_start of activity servo.
 *
 * Triggered by nhfc_start.
 * Yields to nhfc_step.
 * Throws nhfc_e_input.
 */
FILE *flog;

genom_event
nhfc_servo_start(const nhfc_state *state,
                 or_pose_estimator_state *desired, genom_context self)
{
  const or_pose_estimator_state *state_d;

  flog = fopen("/tmp/nhfc.log", "w");
  fprintf(flog, "v1 v2 v3 v4\n");

  if (state->read(self)) return nhfc_e_input(self);
  state_d = state->data(self);
  if (!state_d || !state_d->pos._present) return nhfc_e_input(self);

  desired->pos = state_d->pos;
  return nhfc_step;
}

/** Codel nhfc_servo_step of activity servo.
 *
 * Triggered by nhfc_step.
 * Yields to nhfc_pause_step.
 * Throws nhfc_e_input.
 */
genom_event
nhfc_servo_step(const nhfc_ids_servo_s *servo, const nhfc_state *state,
                const nhfc_reference *reference,
                const nhfc_propeller_input *propeller_input,
                genom_context self)
{
  const or_pose_estimator_state *state_d;
  /* coming soon: const or_pose_estimator_state *reference_d; */
  or_rotorcraft_input *input_data;
  or_pose_estimator_state desired;
  double thrust, torque[3];

  double tlimit;
  double f[4];
  int i;

  static double t;
  const double dt = 0.001;
  const double d = 0.25;
  const double kf = 6.5e-4;
  const double c = 0.0154;

  struct timeval tv;
  int s;

  /* circle */
  t += dt;
  desired.pos._present = true;
  desired.pos._value.x = servo->desired.pos._value.x
    + servo->raddes * (cos(servo->pulsedes * t) - 1);
  desired.pos._value.y = servo->desired.pos._value.y
    + servo->raddes * sin(servo->pulsedes * t);
  desired.pos._value.z = servo->desired.pos._value.z
    + 0*servo->raddes * sin(servo->pulsedes * t);

  desired.pos._value.qw = servo->desired.pos._value.qw;
  desired.pos._value.qx = servo->desired.pos._value.qx;
  desired.pos._value.qy = servo->desired.pos._value.qy;
  desired.pos._value.qz = servo->desired.pos._value.qz;

  desired.vel._present = true;
  desired.vel._value.vx =
    - servo->raddes * servo->pulsedes * sin(servo->pulsedes * t);
  desired.vel._value.vy =
    + servo->raddes * servo->pulsedes * cos(servo->pulsedes * t);
  desired.vel._value.vz =
    0 * servo->raddes * servo->pulsedes * cos(servo->pulsedes * t);

  desired.vel._value.wx = 0.;
  desired.vel._value.wy = 0.;
  desired.vel._value.wz = 0.;

  desired.acc._present = true;
  desired.acc._value.ax = - servo->raddes *
    servo->pulsedes * servo->pulsedes * cos(servo->pulsedes * t);
  desired.acc._value.ay = - servo->raddes *
    servo->pulsedes * servo->pulsedes * sin(servo->pulsedes * t);
  desired.acc._value.az = - 0 * servo->raddes *
    servo->pulsedes * servo->pulsedes * sin(servo->pulsedes * t);


  /* controller */
  if (state->read(self)) return nhfc_e_input(self);
  state_d = state->data(self);
  if (!state_d) return nhfc_e_input(self);

  s = nhfc_controller(servo, state_d, &desired, &thrust, torque);
  if (s) return nhfc_pause_step;


  /* torque limitation */
  tlimit = servo->vmax * servo->vmax * kf - thrust/4;
  if (thrust/4 - servo->vmin * servo->vmin * kf < tlimit)
    tlimit = thrust/4 - servo->vmin * servo->vmin * kf;
  if (tlimit < 0.) tlimit = 0.;
  tlimit = d * tlimit;

  if (fabs(torque[0]) > tlimit) torque[0] = copysign(tlimit, torque[0]);
  if (fabs(torque[1]) > tlimit) torque[1] = copysign(tlimit, torque[1]);
  if (fabs(torque[2]) > 0.2) torque[2] = copysign(0.2, torque[2]);

  /* forces */
  f[0] = thrust/4 - torque[1]/d/2. + torque[2]/c/4;
  f[1] = thrust/4 + torque[1]/d/2. + torque[2]/c/4;
  f[2] = thrust/4 - torque[0]/d/2. - torque[2]/c/4;
  f[3] = thrust/4 + torque[0]/d/2. - torque[2]/c/4;


  /* output */
  input_data = propeller_input->data(self);
  if (!input_data) return nhfc_e_input(self);

  gettimeofday(&tv, NULL);
  input_data->ts.sec = tv.tv_sec;
  input_data->ts.nsec = tv.tv_usec * 1000;

  input_data->w._length = 4;
  for(i = 0; i < 4; i++)
    input_data->w._buffer[i] = (f[i] > 0.) ? sqrt(f[i]/kf) : 0.;

  propeller_input->write(self);

#if 1
  printf("v1: %2.3f v2: %2.3f v3: %2.3f v4: %2.3f\n",
         input_data->w._buffer[0], input_data->w._buffer[1],
         input_data->w._buffer[2], input_data->w._buffer[3]);
#endif
  fprintf(flog, "%f %f %f %f %f\n",
          tv.tv_sec + 1e-6*tv.tv_usec,
          input_data->w._buffer[0], input_data->w._buffer[1],
          input_data->w._buffer[2], input_data->w._buffer[3]);

  return nhfc_pause_step;
}

/** Codel mk_servo_stop of activity servo.
 *
 * Triggered by nhfc_stop.
 * Yields to nhfc_ether.
 * Throws nhfc_e_input.
 */
genom_event
mk_servo_stop(const nhfc_propeller_input *propeller_input,
              genom_context self)
{
  fclose(flog);
  return nhfc_ether;
}
