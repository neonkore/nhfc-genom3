/*
 * Copyright (c) 2016-2018,2021 LAAS/CNRS
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

#include "Eigen/Core"
#include "Eigen/Dense"

#include "codels.h"

/*
 * --- nhfc_controller -----------------------------------------------------
 *
 * Implements the controller described in:
 *
 * T. Lee, M. Leoky and N. H. McClamroch, "Geometric tracking control of a
 * quadrotor UAV on SE(3)", 49th IEEE Conference on Decision and Control
 * (CDC), Atlanta, GA, 2010, pp. 5420-5425.
 */

int
nhfc_controller(const nhfc_ids_body_s *body,
                const nhfc_ids_servo_s *servo,
                const or_pose_estimator_state *state,
                const or_rigid_body_state *desired,
                const or_wrench_estimator_state *exwrench,
                nhfc_log_s *log,
                or_rotorcraft_rotor_control *wprop)
{
  using namespace Eigen;

  Matrix3d Rd;
  Quaternion<double> qd;
  Vector3d xd, vd, wd, ad;

  Matrix3d R;
  Quaternion<double> q;
  Vector3d x, v, w;

  Vector3d ex, ev, eR, ew;
  static Vector3d Iex;
  Matrix3d E;

  Vector3d f;
  Matrix<double, 6, 1> wrench;
  Map< Array<double, or_rotorcraft_max_rotors, 1> > wprop_(wprop->_buffer);

  int i;


  /* geometry */
  Map<
    const Matrix<double, or_rotorcraft_max_rotors, 6, RowMajor>
    > iG_(body->iG);

  /* gains */
  const Array3d Kp(servo->gain.Kpxy, servo->gain.Kpxy, servo->gain.Kpz);
  const Array3d Ki(servo->gain.Kixy, servo->gain.Kixy, servo->gain.Kiz);
  const Array3d Kv(servo->gain.Kvxy, servo->gain.Kvxy, servo->gain.Kvz);
  const Array3d Kq(servo->gain.Kqxy, servo->gain.Kqxy, servo->gain.Kqz);
  const Array3d Kw(servo->gain.Kwxy, servo->gain.Kwxy, servo->gain.Kwz);


  /* desired state */
  if (desired->pos._present) {
    xd <<
      desired->pos._value.x, desired->pos._value.y, desired->pos._value.z;
  } else {
    xd << 0., 0., 0.;
    Iex << 0., 0., 0.;
  }
  if (desired->att._present) {
    qd.coeffs() <<
      desired->att._value.qx, desired->att._value.qy, desired->att._value.qz,
      desired->att._value.qw;
  } else {
    qd = Eigen::Quaternion<double>::Identity();
  }

  if (desired->vel._present) {
    vd <<
      desired->vel._value.vx, desired->vel._value.vy, desired->vel._value.vz;
  } else {
    vd << 0., 0., 0.;
  }
  if (desired->avel._present) {
    wd <<
      desired->avel._value.wx, desired->avel._value.wy, desired->avel._value.wz;
  } else {
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
  if (state->pos._present) {
    x << state->pos._value.x, state->pos._value.y, state->pos._value.z;
    if (!desired->pos._present)
      xd = x + Eigen::Vector3d(0, 0, -5e-2);
  } else {
    x = xd;
    ad = Eigen::Vector3d(0, 0, - servo->emerg.descent);
    Iex << 0., 0., 0.;
  }

  if (state->att._present) {
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;
    if (!desired->att._present)
      qd = q;
  } else {
    q = qd;
  }
  R = q.matrix();

  if (state->vel._present) {
    v << state->vel._value.vx, state->vel._value.vy, state->vel._value.vz;
  } else {
    v = vd;
  }

  if (state->avel._present) {
    w << state->avel._value.wx, state->avel._value.wy, state->avel._value.wz;
  } else {
    w = wd;
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
    + body->mass * (Eigen::Vector3d(0, 0, 9.81) + ad).array();

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


  /* wrench - XXX assumes vertical thrust in body frame */
  wrench.block<3, 1>(0, 0) << 0., 0., f.dot(R.col(2));
  wrench.block<3, 1>(3, 0) = - Kq * eR.array() - Kw * ew.array();


  /* thrust limitation */
  if (wrench(2) < body->thrust_min[2]) wrench(2) = body->thrust_min[2];
  if (wrench(2) > body->thrust_max[2]) wrench(2) = body->thrust_max[2];

  /* output */
  wprop_ = (iG_ * wrench).array().sqrt();

  /* torque limitation */
  if ((wprop_.block(0, 0, wprop->_length, 1) < body->wmin).any() ||
      (wprop_.block(0, 0, wprop->_length, 1) > body->wmax).any()) {
    Array<double, 6, 1> k, kmin, kmax;

    kmin << 1., 1., 1., 0., 0., 0.;
    kmax << 1., 1., 1., 1., 1., 1.;

    do {
      k = (kmin + kmax)/2.;

      wprop_ = (iG_ * (wrench.array() * k).matrix()).array().sqrt();

      if ((wprop_.block(0, 0, wprop->_length, 1) < body->wmin).any() ||
          (wprop_.block(0, 0, wprop->_length, 1) > body->wmax).any())
        kmax = k;
      else
        kmin = k;
    } while(((kmax - kmin) > 1e-2).any());

    wrench = wrench.array() * k;
  }


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
        state->ts.sec, state->ts.nsec,
        wrench(0), wrench(1), wrench(2), wrench(3), wrench(4), wrench(5),
        exwrench->force._present ? exwrench->force._value.x : nan(""),
        exwrench->force._present ? exwrench->force._value.y : nan(""),
        exwrench->force._present ? exwrench->force._value.z : nan(""),
        exwrench->torque._present ? exwrench->torque._value.x : nan(""),
        exwrench->torque._present ? exwrench->torque._value.y : nan(""),
        exwrench->torque._present ? exwrench->torque._value.z : nan(""),
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


/*
 * --- nhfc_state_check ----------------------------------------------------
 *
 * Return non-zero in case of emergency
 */

int
nhfc_state_check(const struct timeval now,
                 const nhfc_ids_servo_s *servo,
                 or_pose_estimator_state *state,
                 or_rigid_body_state *desired)
{
  int e = NHFC_EOK;

  /* deal with obsolete reference */
  if (now.tv_sec + 1e-6 * now.tv_usec >
      0.5 + desired->ts.sec + 1e-9 * desired->ts.nsec) {
    desired->vel._present = false;
    desired->acc._present = false;
  }

  /* check state */
  if (now.tv_sec + 1e-6 * now.tv_usec >
      0.5 + state->ts.sec + 1e-9 * state->ts.nsec) {
    state->pos._present = false;
    state->att._present = false;
    state->vel._present = false;
    state->avel._present = false;
    return NHFC_ETS;
  }

  if (!state->pos._present
      || std::isnan(state->pos._value.x)
      || !state->pos_cov._present
      || state->pos_cov._value.cov[0] > servo->emerg.dx
      || state->pos_cov._value.cov[2] > servo->emerg.dx
      || state->pos_cov._value.cov[5] > servo->emerg.dx) {
    state->pos._present = false;
    e |= NHFC_EPOS;
  }

  if (!state->att._present
      || std::isnan(state->att._value.qw) ||
      !state->att_cov._present ||
      state->att_cov._value.cov[0] > servo->emerg.dq ||
      state->att_cov._value.cov[2] > servo->emerg.dq ||
      state->att_cov._value.cov[5] > servo->emerg.dq ||
      state->att_cov._value.cov[9] > servo->emerg.dq) {
    state->att._present = false;
    e |= NHFC_EATT;
  }

  if (!state->vel._present
      || std::isnan(state->vel._value.vx)
      || !state->vel_cov._present
      || state->vel_cov._value.cov[0] > servo->emerg.dv
      || state->vel_cov._value.cov[2] > servo->emerg.dv
      || state->vel_cov._value.cov[5] > servo->emerg.dv) {
    state->vel._present = false;
    e |= NHFC_EVEL;
  }

  if (!state->avel._present
      || std::isnan(state->avel._value.wx)
      || !state->avel_cov._present
      || state->avel_cov._value.cov[0] > servo->emerg.dw
      || state->avel_cov._value.cov[2] > servo->emerg.dw
      || state->avel_cov._value.cov[5] > servo->emerg.dw) {
    state->avel._present = false;
    e |= NHFC_EAVEL;
  }

  return e;
}


/* --- nhfc_invert_G ------------------------------------------------------- */

void
nhfc_invert_G(const double G[6 * or_rotorcraft_max_rotors],
              double iG[or_rotorcraft_max_rotors * 6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Map< Matrix<double, or_rotorcraft_max_rotors, 6, RowMajor> > iG_(iG);

  iG_ = G_.
    jacobiSvd(ComputeFullU | ComputeFullV).
    solve(Matrix<double, 6, 6>::Identity());
}


/* --- nhfc_Gw2 ------------------------------------------------------------ */

void
nhfc_Gw2(const double G[6 * or_rotorcraft_max_rotors], const double w,
         double f[6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Map< Matrix<double, 6, 1> > f_(f);

  f_ = G_ * Matrix<double, or_rotorcraft_max_rotors, 1>::Constant(w * w);
}
