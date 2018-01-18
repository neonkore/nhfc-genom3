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

#include "nhfc_c_types.h"

#include "codels.h"


/* --- Task wo ---------------------------------------------------------- */


/** Codel nhfc_wo_start of task wo.
 *
 * Triggered by nhfc_start.
 * Yields to nhfc_main.
 */
genom_event
nhfc_wo_start(nhfc_ids *ids,
              const nhfc_external_wrench *external_wrench,
              const genom_context self)
{
  or_wrench_estimator_state *wrench_data;

  ids->wo = (nhfc_ids_wo_s){
    .K = { 1., 1., 1., 1., 1., 1. }
  };

  /* output zero (minimal) velocity */
  wrench_data = external_wrench->data(self);

  wrench_data->ts.sec = 0;
  wrench_data->ts.nsec = 0;
  wrench_data->intrinsic = false;

  wrench_data->force._present = false;
  wrench_data->force_cov._present = false;

  wrench_data->torque._present = false;
  wrench_data->torque_cov._present = false;

  external_wrench->write(self);

  return nhfc_main;
}


/** Codel nhfc_wo_main of task wo.
 *
 * Triggered by nhfc_main.
 * Yields to nhfc_pause_main, nhfc_stop.
 */
genom_event
nhfc_wo_main(const nhfc_ids_body_s *body, const nhfc_ids_wo_s *wo,
             const nhfc_state *state,
             const nhfc_rotor_measure *rotor_measure,
             const nhfc_external_wrench *external_wrench,
             const genom_context self)
{
  const or_pose_estimator_state *state_data;
  const or_rotorcraft_output *rotor_data;

  or_wrench_estimator_state *wrench_data;
  double wprop[or_rotorcraft_max_rotors];
  double xF[3], xT[3];
  struct timeval tv;
  double now;
  size_t i;
  int s;

  gettimeofday(&tv, NULL);
  now = tv.tv_sec + 1e-6 * tv.tv_usec;

  /* rotor measurements */
  if (rotor_measure->read(self)) return nhfc_pause_main;
  rotor_data = rotor_measure->data(self);
  if (!rotor_data) return nhfc_pause_main;

  for(i = 0; i < rotor_data->rotor._length; i++) {
    if (now > 0.1 +
        rotor_data->rotor._buffer[i].ts.sec +
        1e-9 * rotor_data->rotor._buffer[i].ts.nsec)
      return nhfc_pause_main;

    wprop[i] = rotor_data->rotor._buffer[i].velocity;
  }


  /* current state -- assumes it is read() by main task */
  state_data = state->data(self);
  if (!state_data) return nhfc_pause_main;

  if (now > 0.1 + state_data->ts.sec + 1e-9 * state_data->ts.nsec)
    return nhfc_pause_main;


  /* external wrench */
  s = nhfc_wrench_observer(body, wo, state_data, wprop, xF, xT);
  if (s) return nhfc_pause_main;

  wrench_data = external_wrench->data(self);

  wrench_data->ts = state_data->ts;

  wrench_data->force._present = true;
  wrench_data->force._value.x = xF[0];
  wrench_data->force._value.y = xF[1];
  wrench_data->force._value.z = xF[2];

  wrench_data->torque._present = true;
  wrench_data->torque._value.x = xT[0];
  wrench_data->torque._value.y = xT[1];
  wrench_data->torque._value.z = xT[2];

  external_wrench->write(self);

  return nhfc_pause_main;
}


/** Codel mk_wo_stop of task wo.
 *
 * Triggered by nhfc_stop.
 * Yields to nhfc_ether.
 */
genom_event
mk_wo_stop(nhfc_ids *ids, const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return nhfc_ether;
}
