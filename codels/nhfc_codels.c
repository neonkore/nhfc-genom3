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


/* --- Function set_state ----------------------------------------------- */

/** Codel nhfc_set_state of function set_state.
 *
 * Returns genom_ok.
 */
genom_event
nhfc_set_state(const or_t3d_pos *pos, const or_t3d_vel *vel,
               const or_t3d_acc *acc, or_pose_estimator_state *desired,
               genom_context self)
{
  struct timeval tv;

  gettimeofday(&tv, NULL);
  desired->ts.sec = tv.tv_sec;
  desired->ts.nsec = tv.tv_usec * 1000.;

  if (isnan(pos->x) || isnan(pos->qw))
    desired->pos._present = false;
  else {
    desired->pos._present = true;
    desired->pos._value = *pos;
  }

  if (isnan(vel->vx) || isnan(vel->wx))
    desired->vel._present = false;
  else {
    desired->vel._present = true;
    desired->vel._value = *vel;
  }

  if (isnan(acc->ax))
    desired->acc._present = false;
  else {
    desired->acc._present = true;
    desired->acc._value = *acc;
  }

  return genom_ok;
}


/* --- Function set_position -------------------------------------------- */

/** Codel nhfc_set_position of function set_position.
 *
 * Returns genom_ok.
 */
genom_event
nhfc_set_position(double x, double y, double z, double yaw,
                  or_pose_estimator_state *desired,
                  genom_context self)
{
  struct timeval tv;

  gettimeofday(&tv, NULL);
  desired->ts.sec = tv.tv_sec;
  desired->ts.nsec = tv.tv_usec * 1000.;

  desired->pos._present = true;
  desired->pos._value.x = x;
  desired->pos._value.y = y;
  desired->pos._value.z = z;
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

  return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel nhfc_log of function log.
 *
 * Returns genom_ok.
 * Throws nhfc_e_sys.
 */
genom_event
nhfc_log(const char path[64], nhfc_log_s **log, genom_context self)
{
  FILE *f;

  f = fopen(path, "w");
  if (!f) return nhfc_e_sys_error(path, self);
  fprintf(f, nhfc_log_header_fmt "\n");

  if ((*log)->f) fclose((*log)->f);
  (*log)->f = f;
  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel nhfc_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
nhfc_log_stop(nhfc_log_s **log, genom_context self)
{
  if ((*log)->f) fclose((*log)->f);
  (*log)->f = NULL;

  return genom_ok;
}
