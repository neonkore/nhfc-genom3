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
  fprintf(flog, "ts fz tx ty tz\n");

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
                const nhfc_cmd_wrench *cmd_wrench, genom_context self)
{
  const or_pose_estimator_state *state_d;
  /* coming soon: const or_pose_estimator_state *reference_d; */
  or_rotorcraft_ts_wrench *cmd_wrench_d;
  or_pose_estimator_state desired;

  const double dt = 0.001;
  static double t;

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

  cmd_wrench_d = cmd_wrench->data(self);

  s = nhfc_controller(servo, state_d, &desired, &cmd_wrench_d->w);
  if (s) return nhfc_pause_step;

  gettimeofday(&tv, NULL);
  cmd_wrench_d->ts.sec = tv.tv_sec;
  cmd_wrench_d->ts.nsec = tv.tv_usec * 1000;
  cmd_wrench->write(self);

#if 1
  printf("fz: %2.3f tx: %2.3f ty: %2.3f tz: %2.3f\n",
         cmd_wrench_d->w.f.z,
         cmd_wrench_d->w.t.x, cmd_wrench_d->w.t.y, cmd_wrench_d->w.t.z);
#endif
  fprintf(flog, "%f %f %f %f %f\n",
          tv.tv_sec + 1e-6*tv.tv_usec,
          cmd_wrench_d->w.f.z,
          cmd_wrench_d->w.t.x, cmd_wrench_d->w.t.y, cmd_wrench_d->w.t.z);

  return nhfc_pause_step;
}

/** Codel mk_servo_stop of activity servo.
 *
 * Triggered by nhfc_stop.
 * Yields to nhfc_ether.
 * Throws nhfc_e_input.
 */
genom_event
mk_servo_stop(const nhfc_cmd_wrench *cmd_wrench, genom_context self)
{
  fclose(flog);
  return nhfc_ether;
}
