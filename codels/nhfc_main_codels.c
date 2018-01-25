/*
 * Copyright (c) 2015-2018 LAAS/CNRS
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
 * Yields to nhfc_init.
 */
genom_event
nhfc_main_start(nhfc_ids *ids, const genom_context self)
{
  static const double kf = 6.5e-4;
  static const double c = 0.0154;
  static const double d = 0.23;

  ids->body = (nhfc_ids_body_s){
    /* mikrokopter quadrotors defaults */
    .G = {
         0.,    0.,   0.,    0.,   0., 0., 0., 0.,
         0.,    0.,   0.,    0.,   0., 0., 0., 0.,
         kf,    kf,   kf,    kf,   0., 0., 0., 0.,

         0.,  d*kf,   0., -d*kf,   0., 0., 0., 0.,
      -d*kf,    0., d*kf,    0.,   0., 0., 0., 0.,
       c*kf, -c*kf, c*kf, -c*kf,   0., 0., 0., 0.
    },

    .J = {
      0.015,    0.,    0.,
      0.,    0.015,    0.,
      0.,       0., 0.015
    },

    .mass = 1.0,

    .wmax = 90., .wmin = 16.
  };
  nhfc_set_geom(ids->body.G, &ids->body, self);
  nhfc_set_wlimit(ids->body.wmin, ids->body.wmax, &ids->body, self);

  ids->servo = (nhfc_ids_servo_s){
    .sat = { .x = 0.10, .v = 0.1, .ix = 0.10 },
    .gain = {
      .Kpxy = 14., .Kvxy = 7., .Kpz = 20., .Kvz = 10.,
      .Kqxy = 2.3, .Kwxy = .23, .Kqz = .2, .Kwz = .02,
      .Kixy = 0., .Kiz = 0.
    },

    .ramp = 3,
    .scale = 0.,

    .emerg = {
      .descent = .1,
      .dx = 0.05 * 0.05 /9.,
      .dq = 5. * 5. * M_PI*M_PI/180./180./9.,
      .dv = 0.2 * 0.2 /9.,
      .dw = 20. * 20. * M_PI*M_PI/180./180./9.
    }
  };

  ids->af = (nhfc_ids_af_s){
    .enable = false,

    .mass = ids->body.mass,
    .B = { 8., 8., 8., 8., 8., 8. },
    .K = { 10., 10., 10., 10., 10., 10. },
    .J =  {
      ids->body.J[0],             0.,             0.,
                  0., ids->body.J[4],             0.,
                  0.,             0., ids->body.J[8]
    },

    .force = { 0. },
    .torque = { 0. },
  };
  nhfc_adm_J(ids->af.J);

  ids->reference = (or_pose_estimator_state){
    .pos._present = false,
    .pos_cov._present = false,
    .vel._present = false,
    .vel_cov._present = false,
    .acc._present = false,
    .acc_cov._present = false
  };

  ids->desired = (or_pose_estimator_state){
    .pos._present = false,
    .pos_cov._present = false,
    .vel._present = false,
    .vel_cov._present = false,
    .acc._present = false,
    .acc_cov._present = false
  };

  /* init logging */
  ids->log = malloc(sizeof(*ids->log));
  if (!ids->log) abort();
  *ids->log = (nhfc_log_s){
    .req = {
      .aio_fildes = -1,
      .aio_offset = 0,
      .aio_buf = ids->log->buffer,
      .aio_nbytes = 0,
      .aio_reqprio = 0,
      .aio_sigevent = { .sigev_notify = SIGEV_NONE },
      .aio_lio_opcode = LIO_NOP
    },
    .pending = false, .skipped = false,
    .decimation = 1, .missed = 0, .total = 0
  };

  return nhfc_init;
}


/** Codel nhfc_main_init of task main.
 *
 * Triggered by nhfc_init.
 * Yields to nhfc_pause_init, nhfc_control.
 */
genom_event
nhfc_main_init(const or_pose_estimator_state *reference,
               const nhfc_state *state,
               const nhfc_rotor_input *rotor_input,
               const genom_context self)
{
  or_rotorcraft_input *input_data;
  struct timeval tv;
  int i;

  /* switch to servo mode upon reception of the first valid position */
  if (reference->pos._present) return nhfc_control;

  /* update current state - used by the wo task */
  state->read(self);

  /* output zero (minimal) velocity */
  input_data = rotor_input->data(self);
  if (!input_data) return nhfc_pause_init;

  gettimeofday(&tv, NULL);
  input_data->ts.sec = tv.tv_sec;
  input_data->ts.nsec = tv.tv_usec * 1000;
  input_data->control = or_rotorcraft_velocity;

  input_data->desired._length = 4; /* XXX assumes a quadrotor */
  for(i = 0; i < 4; i++)
    input_data->desired._buffer[i] = 0.;

  rotor_input->write(self);

  return nhfc_pause_init;
}


/** Codel nhfc_main_control of task main.
 *
 * Triggered by nhfc_control.
 * Yields to nhfc_pause_control.
 */
genom_event
nhfc_main_control(const nhfc_ids_body_s *body, nhfc_ids_servo_s *servo,
                  nhfc_ids_af_s *af, const nhfc_state *state,
                  const nhfc_external_wrench *external_wrench,
                  or_pose_estimator_state *reference,
                  or_pose_estimator_state *desired, nhfc_log_s **log,
                  const nhfc_rotor_input *rotor_input,
                  const genom_context self)
{
  const or_pose_estimator_state *state_data = NULL;
  or_wrench_estimator_state *wrench_data;
  or_rotorcraft_input *input_data;
  struct timeval tv;
  size_t i;
  int s;

  gettimeofday(&tv, NULL);

  /* reset propeller velocities by default */
  input_data = rotor_input->data(self);
  if (!input_data) return nhfc_pause_control;
  for(i = 0; i < input_data->desired._length; i++)
    input_data->desired._buffer[i] = 0.;

  /* current state */
  if (state->read(self) || !(state_data = state->data(self)))
    goto output;
  if (tv.tv_sec + 1e-6 * tv.tv_usec >
      0.5 + state_data->ts.sec + 1e-9 * state_data->ts.nsec)
    goto output;

  /* deal with obsolete reference */
  if (tv.tv_sec + 1e-6 * tv.tv_usec >
      0.5 + reference->ts.sec + 1e-9 * reference->ts.nsec) {
    reference->vel._present = false;
    reference->acc._present = false;
  }

  /* admittance filter  */
  wrench_data = external_wrench->data(self);
  if (af->enable) {
    s = nhfc_adm_filter(body, af, reference, wrench_data, desired);
    if (s) return nhfc_pause_control;
  } else {
    *desired = *reference;
  }

  /* position controller */
  s = nhfc_controller(body, servo, state_data, desired, wrench_data,
                      *log, &input_data->desired);
  if (s) return nhfc_pause_control;

  /* output */
output:
  if (state_data) {
    input_data->ts = state_data->ts;
  } else {
    input_data->ts.sec = tv.tv_sec;
    input_data->ts.nsec = tv.tv_usec * 1000;
  }

  if (servo->scale < 1.) {
    for(i = 0; i < input_data->desired._length; i++)
      input_data->desired._buffer[i] *= servo->scale;

    servo->scale += 1e-3 * nhfc_control_period_ms / servo->ramp;
  }

  rotor_input->write(self);

  return nhfc_pause_control;
}


/** Codel mk_main_stop of task main.
 *
 * Triggered by nhfc_stop.
 * Yields to nhfc_ether.
 */
genom_event
mk_main_stop(const nhfc_rotor_input *rotor_input,
             const genom_context self)
{
  or_rotorcraft_input *input_data;
  struct timeval tv;
  int i;

  input_data = rotor_input->data(self);
  if (!input_data) return nhfc_ether;

  gettimeofday(&tv, NULL);
  input_data->ts.sec = tv.tv_sec;
  input_data->ts.nsec = tv.tv_usec * 1000;
  input_data->control = or_rotorcraft_velocity;

  input_data->desired._length = 4;
  for(i = 0; i < 4; i++)
    input_data->desired._buffer[i] = 0.;

  rotor_input->write(self);
  return nhfc_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel nhfc_servo_main of activity servo.
 *
 * Triggered by nhfc_start.
 * Yields to nhfc_pause_start, nhfc_ether.
 * Throws nhfc_e_input.
 */
genom_event
nhfc_servo_main(const nhfc_reference *in,
                or_pose_estimator_state *reference,
                const genom_context self)
{
  const or_pose_estimator_state *ref_data;

  if (in->read(self)) return nhfc_e_input(self);
  ref_data = in->data(self);
  if (!ref_data) return nhfc_e_input(self);

  /* check if timestamps have changed */
  if (reference->ts.nsec != ref_data->ts.nsec ||
      reference->ts.sec != ref_data->ts.sec)
    *reference = *ref_data;

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
                          or_pose_estimator_state *reference,
                          const genom_context self)
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

  reference->ts = state_data->ts;

  reference->pos._present = true;
  reference->pos._value.x = state_data->pos._value.x;
  reference->pos._value.y = state_data->pos._value.y;
  reference->pos._value.z = state_data->pos._value.z;
  reference->pos._value.qw = cos(yaw/2.);
  reference->pos._value.qx = 0.;
  reference->pos._value.qy = 0.;
  reference->pos._value.qz = sin(yaw/2.);

  reference->vel._present = true;
  reference->vel._value.vx = 0.;
  reference->vel._value.vy = 0.;
  reference->vel._value.vz = 0.;
  reference->vel._value.wx = 0.;
  reference->vel._value.wy = 0.;
  reference->vel._value.wz = 0.;

  reference->acc._present = true;
  reference->acc._value.ax = 0.;
  reference->acc._value.ay = 0.;
  reference->acc._value.az = 0.;

  return nhfc_ether;
}
