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
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "nhfc_c_types.h"

#include "codels.h"


/* --- Attribute set_wlimit --------------------------------------------- */

/** Validation codel nhfc_set_wlimit of attribute set_wlimit.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
nhfc_set_wlimit(double wmin, double wmax, nhfc_ids_body_s *body,
                const genom_context self)
{
  double f[6];
  int i;

  nhfc_Gw2(body->G, wmin, f);
  for(i = 0; i < 3; i++) body->thrust_min[i] = f[i];

  nhfc_Gw2(body->G, wmax, f);
  for(i = 0; i < 3; i++) body->thrust_max[i] = f[i];

  return genom_ok;
}


/* --- Attribute set_geom ----------------------------------------------- */

/** Validation codel nhfc_set_geom of attribute set_geom.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
nhfc_set_geom(const double G[48], nhfc_ids_body_s *body,
              const genom_context self)
{
  double f[6];
  int i;

  nhfc_invert_G(G, body->iG);

  nhfc_Gw2(G, body->wmin, f);
  for(i = 0; i < 3; i++) body->thrust_min[0] = f[0];

  nhfc_Gw2(G, body->wmax, f);
  for(i = 0; i < 3; i++) body->thrust_max[0] = f[0];

  return genom_ok;
}


/* --- Attribute set_emerg ---------------------------------------------- */

/** Validation codel nhfc_set_emerg of attribute set_emerg.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
nhfc_set_emerg(nhfc_ids_servo_s_emerg_s *emerg,
               const genom_context self)
{
  emerg->dx = emerg->dx * emerg->dx / 9.;
  emerg->dq = emerg->dq * emerg->dq / 9.;
  emerg->dv = emerg->dv * emerg->dv / 9.;
  emerg->dw = emerg->dw * emerg->dw / 9.;
  return genom_ok;
}


/* --- Function set_state ----------------------------------------------- */

/** Codel nhfc_set_state of function set_state.
 *
 * Returns genom_ok.
 */
genom_event
nhfc_set_state(const or_t3d_pos *pos, const or_t3d_vel *vel,
               const or_t3d_acc *acc, or_pose_estimator_state *desired,
               const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

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
                  const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

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


/* --- Function stop ---------------------------------------------------- */

/** Codel nhfc_servo_stop of function stop.
 *
 * Returns genom_ok.
 */
genom_event
nhfc_servo_stop(or_pose_estimator_state *desired,
                const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

  gettimeofday(&tv, NULL);
  desired->ts.sec = tv.tv_sec;
  desired->ts.nsec = tv.tv_usec * 1000.;

  desired->pos._present = false;
  desired->vel._present = false;
  desired->acc._present = false;

  return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel nhfc_log of function log.
 *
 * Returns genom_ok.
 * Throws nhfc_e_sys.
 */
genom_event
nhfc_log(const char path[64], uint32_t decimation, nhfc_log_s **log,
         const genom_context self)
{
  int fd;

  fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
  if (fd < 0) return nhfc_e_sys_error(path, self);

  if (write(fd, nhfc_log_header_fmt "\n", sizeof(nhfc_log_header_fmt)) < 0)
    return nhfc_e_sys_error(path, self);

  if ((*log)->req.aio_fildes >= 0) {
    close((*log)->req.aio_fildes);

    if ((*log)->pending)
      while (aio_error(&(*log)->req) == EINPROGRESS)
        /* empty body */;
  }
  (*log)->req.aio_fildes = fd;
  (*log)->pending = false;
  (*log)->skipped = false;
  (*log)->decimation = decimation < 1 ? 1 : decimation;
  (*log)->missed = 0;
  (*log)->total = 0;

  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel nhfc_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
nhfc_log_stop(nhfc_log_s **log, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  if (*log && (*log)->req.aio_fildes >= 0)
    close((*log)->req.aio_fildes);
  (*log)->req.aio_fildes = -1;

  return genom_ok;
}


/* --- Function log_info ------------------------------------------------ */

/** Codel nhfc_log_info of function log_info.
 *
 * Returns genom_ok.
 */
genom_event
nhfc_log_info(const nhfc_log_s *log, uint32_t *miss, uint32_t *total,
              const genom_context self)
{
  *miss = *total = 0;
  if (log) {
    *miss = log->missed;
    *total = log->total;
  }
  return genom_ok;
}
