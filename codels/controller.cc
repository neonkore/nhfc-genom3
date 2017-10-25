/*
 * Copyright (c) 2016-2017 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Anthony Mallet on Tue Mar 22 2016
 */
#include "acnhfc.h"

#include <aio.h>
#include <err.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <codels.h>

/*
 * This file implements the controller described in:
 *
 * T. Lee, M. Leoky and N. H. McClamroch, "Geometric tracking control of a
 * quadrotor UAV on SE(3)", 49th IEEE Conference on Decision and Control
 * (CDC), Atlanta, GA, 2010, pp. 5420-5425.
 */

int
nhfc_controller(const nhfc_ids_servo_s *servo,
                const or_pose_estimator_state *state,
                const or_pose_estimator_state *desired,
                nhfc_log_s *log,
                double *thrust, double torque[3])
{
  Eigen::Matrix3d Rd;
  Eigen::Quaternion<double> qd;
  Eigen::Vector3d xd, vd, wd, ad;

  Eigen::Matrix3d R;
  Eigen::Quaternion<double> q;
  Eigen::Vector3d x, v, w;

  Eigen::Vector3d ex, ev, eR, ew;
  static Eigen::Vector3d Iex;

  Eigen::Vector3d f;
  Eigen::Map<Eigen::Matrix<double, 3, 1> > t(torque);
  Eigen::Matrix3d E;
  int i;

  static bool emerg;
  bool emerg_x, emerg_q, emerg_v, emerg_w;

  /* gains */
  const Eigen::Array3d Kp(servo->gain.Kpxy, servo->gain.Kpxy, servo->gain.Kpz);
  const Eigen::Array3d Ki(servo->gain.Kixy, servo->gain.Kixy, servo->gain.Kiz);
  const Eigen::Array3d Kv(servo->gain.Kvxy, servo->gain.Kvxy, servo->gain.Kvz);
  const Eigen::Array3d Kq(servo->gain.Kqxy, servo->gain.Kqxy, servo->gain.Kqz);
  const Eigen::Array3d Kw(servo->gain.Kwxy, servo->gain.Kwxy, servo->gain.Kwz);


  /* desired state */
  if (desired->pos._present) {
    xd <<
      desired->pos._value.x, desired->pos._value.y, desired->pos._value.z;
    qd.coeffs() <<
      desired->pos._value.qx, desired->pos._value.qy, desired->pos._value.qz,
      desired->pos._value.qw;
  } else {
    xd << 0., 0., 0.;
    qd = Eigen::Quaternion<double>::Identity();
    Iex << 0., 0., 0.;
  }

  if (desired->vel._present) {
    vd <<
      desired->vel._value.vx, desired->vel._value.vy, desired->vel._value.vz;
    wd <<
      desired->vel._value.wx, desired->vel._value.wy, desired->vel._value.wz;
  } else {
    vd << 0., 0., 0.;
    wd << 0., 0., 0.;
  }

  if (desired->acc._present) {
    ad <<
      desired->acc._value.ax,
      desired->acc._value.ay,
      desired->acc._value.az;
  } else
    ad << 0., 0., 0.;


  /* current state */
  if (state->pos._present && !std::isnan(state->pos._value.x) &&
      state->pos_cov._present &&
      state->pos_cov._value.cov[0] < servo->emerg.dx &&
      state->pos_cov._value.cov[2] < servo->emerg.dx &&
      state->pos_cov._value.cov[5] < servo->emerg.dx) {
    x << state->pos._value.x, state->pos._value.y, state->pos._value.z;
    if (!desired->pos._present)
      xd = x + Eigen::Vector3d(0, 0, -5e-2);
    emerg_x = false;
  } else {
    emerg_x = true;
    x = xd;
    Iex << 0., 0., 0.;
  }

  if (state->pos._present && !std::isnan(state->pos._value.qw) &&
      state->pos_cov._present &&
      state->pos_cov._value.cov[9] < servo->emerg.dq &&
      state->pos_cov._value.cov[14] < servo->emerg.dq &&
      state->pos_cov._value.cov[20] < servo->emerg.dq &&
      state->pos_cov._value.cov[27] < servo->emerg.dq) {
    q.coeffs() <<
      state->pos._value.qx, state->pos._value.qy, state->pos._value.qz,
      state->pos._value.qw;
    if (!desired->pos._present)
      qd = q;
    emerg_q = false;
  } else {
    emerg_q = true;
    q = qd;
  }
  R = q.matrix();

  if (state->vel._present && !std::isnan(state->vel._value.vx) &&
      state->vel_cov._present &&
      state->vel_cov._value.cov[0] < servo->emerg.dv &&
      state->vel_cov._value.cov[2] < servo->emerg.dv &&
      state->vel_cov._value.cov[5] < servo->emerg.dv) {
    v << state->vel._value.vx, state->vel._value.vy, state->vel._value.vz;
    emerg_v = false;
  } else {
    emerg_v = true;
    v = vd;
  }

  if (state->vel._present && !std::isnan(state->vel._value.wx) &&
      state->vel_cov._present &&
      state->vel_cov._value.cov[9] < servo->emerg.dw &&
      state->vel_cov._value.cov[14] < servo->emerg.dw &&
      state->vel_cov._value.cov[20] < servo->emerg.dw) {
    w << state->vel._value.wx, state->vel._value.wy, state->vel._value.wz;
    emerg_w = false;
  } else {
    emerg_w = true;
    w = wd;
  }

  if (emerg_x || emerg_v)
    ad = Eigen::Vector3d(0, 0, - servo->emerg.descent);

  if (emerg_x || emerg_q || emerg_v || emerg_w) {
    if (!emerg) {
      warnx("emergency descent due to uncertain state estimation");
      emerg = true;
    }
  } else {
    if (emerg) {
      warnx("recovered from emergency");
      emerg = false;
    }
  }


  /* position error */
  ex = x - xd;
  for(i = 0; i < 3; i++)
    if (fabs(ex(i)) > servo->sat.x) ex(i) = copysign(servo->sat.x, ex(i));

  Iex += ex * nhfc_control_period_ms/1000.;
  for(i = 0; i < 3; i++)
    if (fabs(Iex(i)) > servo->sat.ix) Iex(i) = copysign(servo->sat.ix, Iex(i));

  /* velocity error */
  ev = v - vd;
  for(i = 0; i < 3; i++)
    if (fabs(ev(i)) > servo->sat.v) ev(i) = copysign(servo->sat.v, ev(i));


  /* desired orientation matrix */
  f =
    - Kp * ex.array() - Kv * ev.array() - Ki * Iex.array()
    + servo->mass * (Eigen::Vector3d(0, 0, 9.81) + ad).array();

  Rd.col(2) = f.normalized();

  Rd.col(1) = Rd.col(2).cross(qd.matrix().col(0));
  Rd.col(1).normalize();

  Rd.col(0) = Rd.col(1).cross(Rd.col(2));


  /* orientation error */
  E = 0.5 * (Rd.transpose()*R - R.transpose()*Rd);
  eR <<
    (E(2, 1) - E(1, 2))/2.,
    (E(0, 2) - E(2, 0))/2.,
    (E(1, 0) - E(0, 1))/2.;


  /* angular velocity error */
  ew = R.transpose() * (w - wd);

  /* output */
  *thrust = f.dot(R.col(2));
  t = - Kq * eR.array() - Kw * ew.array();


  /* logging */
  if (log->req.aio_fildes >= 0) {
    log->total++;
    if (log->total % log->decimation == 0) {
      if (log->pending) {
        if (aio_error(&log->req) != EINPROGRESS) {
          log->pending = false;
          if (aio_return(&log->req) <= 0) {
            warn("log");
            close(log->req.aio_fildes);
            log->req.aio_fildes = -1;
          }
        } else {
          log->skipped = true;
          log->missed++;
        }
      }
    }

    if (log->req.aio_fildes >= 0 && !log->pending) {
      double d;
      double roll, pitch, yaw;

      d = hypot(Rd(0,0), Rd(1,0));
      if (fabs(d) > 1e-10) {
        yaw = atan2(Rd(1,0), Rd(0,0));
        roll = atan2(Rd(2,1), Rd(2,2));
      } else {
        yaw = atan2(-Rd(0,1), Rd(1,1));
        roll = 0.;
      }
      pitch = atan2(-Rd(2,0), d);

      log->req.aio_nbytes = snprintf(
        log->buffer, sizeof(log->buffer),
        "%s" nhfc_log_fmt "\n",
        log->skipped ? "\n" : "",
        state->ts.sec, state->ts.nsec, *thrust,
        f(0), f(1), f(2), t(0), t(1), t(2),
        xd(0), xd(1), xd(2), roll, pitch, yaw,
        vd(0), vd(1), vd(2), wd(0), wd(1), wd(2),
        ad(0), ad(1), ad(2),
        ex(0), ex(1), ex(2), ev(0), ev(1), ev(2),
        eR(0), eR(1), eR(2), ew(0), ew(1), ew(2));

      if (aio_write(&log->req)) {
        warn("log");
        close(log->req.aio_fildes);
        log->req.aio_fildes = -1;
      } else
        log->pending = true;

      log->skipped = false;
    }
  }

  return 0;
}
