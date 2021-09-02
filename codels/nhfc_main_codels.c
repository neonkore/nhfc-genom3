/*
 * Copyright (c) 2015-2021 LAAS/CNRS
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
#include <err.h>
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
nhfc_main_start(nhfc_ids *ids, const nhfc_rotor_input *rotor_input,
                const genom_context self)
{
  or_rotorcraft_input *input_data;

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

  ids->reference = (or_rigid_body_state){
    .pos._present = false,
    .att._present = false,
    .vel._present = false,
    .avel._present = false,
    .acc._present = false,
    .aacc._present = false,
    .jerk._present = false,
    .snap._present = false
  };

  ids->desired = (or_rigid_body_state){
    .pos._present = false,
    .att._present = false,
    .vel._present = false,
    .avel._present = false,
    .acc._present = false,
    .aacc._present = false,
    .jerk._present = false,
    .snap._present = false,
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

  input_data = rotor_input->data(self);
  if (input_data) input_data->desired._length = ids->body.rotors;

  return nhfc_init;
}


/** Codel nhfc_main_init of task main.
 *
 * Triggered by nhfc_init.
 * Yields to nhfc_pause_init, nhfc_control.
 */
genom_event
nhfc_main_init(const or_rigid_body_state *reference,
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

  for(i = 0; i < input_data->desired._length; i++)
    input_data->desired._buffer[i] = 0.;

  rotor_input->write(self);

  return nhfc_pause_init;
}


/** Codel nhfc_main_control of task main.
 *
 * Triggered by nhfc_control.
 * Yields to nhfc_pause_control, nhfc_emergency.
 */
genom_event
nhfc_main_control(const nhfc_ids_body_s *body, nhfc_ids_servo_s *servo,
                  nhfc_ids_af_s *af, const nhfc_state *state,
                  const nhfc_external_wrench *external_wrench,
                  or_rigid_body_state *reference,
                  or_rigid_body_state *desired, nhfc_log_s **log,
                  const nhfc_rotor_input *rotor_input,
                  const genom_context self)
{
  or_pose_estimator_state *state_data = NULL;
  or_wrench_estimator_state *wrench_data = external_wrench->data(self);
  or_rotorcraft_input *input_data = rotor_input->data(self);
  struct timeval tv;
  size_t i;
  int e;

  gettimeofday(&tv, NULL);

  /* read current state */
  if (state->read(self) || !(state_data = state->data(self))) {
    warnx("emergency stop");
    return nhfc_emergency;
  }

  /* check state */
  e = nhfc_state_check(tv, servo, state_data, reference);
  if (e) {
    if (e & NHFC_ETS) warnx("obsolete state");
    if (e & NHFC_EPOS) warnx("uncertain position");
    if (e & NHFC_EATT) warnx("uncertain orientation");
    if (e & NHFC_EVEL) warnx("uncertain velocity");
    if (e & NHFC_EAVEL) warnx("uncertain angular velocity");
    warnx("emergency descent");
    return nhfc_emergency;
  }

  /* admittance filter  */
  if (af->enable)
    nhfc_adm_filter(body, af, reference, wrench_data, desired);
  else
    *desired = *reference;

  /* position controller */
  nhfc_controller(body, servo, state_data, desired, wrench_data,
                  *log, &input_data->desired);

  /* output */
  input_data->ts = state_data->ts;
  if (servo->scale < 1.) {
    for(i = 0; i < input_data->desired._length; i++)
      input_data->desired._buffer[i] *= servo->scale;

    servo->scale += 1e-3 * nhfc_control_period_ms / servo->ramp;
  }

  rotor_input->write(self);
  return nhfc_pause_control;
}


/** Codel nhfc_main_emergency of task main.
 *
 * Triggered by nhfc_emergency.
 * Yields to nhfc_pause_emergency, nhfc_control.
 */
genom_event
nhfc_main_emergency(const nhfc_ids_body_s *body,
                    nhfc_ids_servo_s *servo, nhfc_ids_af_s *af,
                    const nhfc_state *state,
                    const nhfc_external_wrench *external_wrench,
                    or_rigid_body_state *reference,
                    or_rigid_body_state *desired, nhfc_log_s **log,
                    const nhfc_rotor_input *rotor_input,
                    const genom_context self)
{
  or_pose_estimator_state *state_data = NULL;
  or_wrench_estimator_state *wrench_data = external_wrench->data(self);
  or_rotorcraft_input *input_data = rotor_input->data(self);
  struct timeval tv;
  size_t i;
  int e;

  gettimeofday(&tv, NULL);

  /* read current state */
  if (state->read(self) || !(state_data = state->data(self))) {
    input_data->ts.sec = tv.tv_sec;
    input_data->ts.nsec = tv.tv_usec * 1000;
    input_data->desired._length = body->rotors;
    for(i = 0; i < input_data->desired._length; i++)
      input_data->desired._buffer[i] = 0.;

    rotor_input->write(self);
    return nhfc_pause_emergency;
  }

  /* check state */
  e = nhfc_state_check(tv, servo, state_data, reference);
  if (!e) {
    warnx("recovered from emergency");
    return nhfc_control;
  }

  /* admittance filter  */
  if (af->enable)
    nhfc_adm_filter(body, af, reference, wrench_data, desired);
  else
    *desired = *reference;

  /* position controller */
  nhfc_controller(body, servo, state_data, desired, wrench_data,
                  *log, &input_data->desired);

  /* output */
  input_data->ts = state_data->ts;
  if (servo->scale < 1.) {
    for(i = 0; i < input_data->desired._length; i++)
      input_data->desired._buffer[i] *= servo->scale;

    servo->scale += 1e-3 * nhfc_control_period_ms / servo->ramp;
  }

  rotor_input->write(self);
  return nhfc_pause_emergency;
}


/** Codel nhfc_main_stop of task main.
 *
 * Triggered by nhfc_stop.
 * Yields to nhfc_ether.
 */
genom_event
nhfc_main_stop(const nhfc_rotor_input *rotor_input,
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

  for(i = 0; i < input_data->desired._length; i++)
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
                or_rigid_body_state *reference,
                const genom_context self)
{
  const or_rigid_body_state *ref_data;

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
                          or_rigid_body_state *reference,
                          const genom_context self)
{
  const or_pose_estimator_state *state_data;
  double qw, qx, qy, qz;
  double yaw;

  if (state->read(self)) return nhfc_e_input(self);
  state_data = state->data(self);
  if (!state_data) return nhfc_e_input(self);
  if (!state_data->pos._present) return nhfc_e_input(self);
  if (!state_data->att._present) return nhfc_e_input(self);

  qw = state_data->att._value.qw;
  qx = state_data->att._value.qx;
  qy = state_data->att._value.qy;
  qz = state_data->att._value.qz;
  yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

  reference->ts = state_data->ts;

  reference->pos._present = true;
  reference->pos._value.x = state_data->pos._value.x;
  reference->pos._value.y = state_data->pos._value.y;
  reference->pos._value.z = state_data->pos._value.z;

  reference->att._present = true;
  reference->att._value.qw = cos(yaw/2.);
  reference->att._value.qx = 0.;
  reference->att._value.qy = 0.;
  reference->att._value.qz = sin(yaw/2.);

  reference->vel._present = true;
  reference->vel._value.vx = 0.;
  reference->vel._value.vy = 0.;
  reference->vel._value.vz = 0.;

  reference->avel._present = true;
  reference->avel._value.wx = 0.;
  reference->avel._value.wy = 0.;
  reference->avel._value.wz = 0.;

  reference->acc._present = true;
  reference->acc._value.ax = 0.;
  reference->acc._value.ay = 0.;
  reference->acc._value.az = 0.;

  reference->jerk._present = false;
  reference->snap._present = false;

  return nhfc_ether;
}
