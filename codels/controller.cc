/*
 * Copyright (c) 2016 LAAS/CNRS
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
#include <cmath>

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
                double *thrust, double torque[3])
{
  Eigen::Matrix3d Rd;
  Eigen::Quaternion<double> qd;
  Eigen::Vector3d xd, vd, wd, ad;

  Eigen::Matrix3d R;
  Eigen::Quaternion<double> q;
  Eigen::Vector3d x, v, w;

  Eigen::Vector3d ex, ev, eR, ew;

  Eigen::Vector3d f;
  Eigen::Map<Eigen::Matrix<double, 3, 1> > t(torque);
  Eigen::Matrix3d E;
  int i;

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
  if (state->pos._present && !std::isnan(state->pos._value.x))
    if (state->pos_cov._present && (
          state->pos_cov._value.cov[0] > 0.1 ||
          state->pos_cov._value.cov[2] > 0.1 ||
          state->pos_cov._value.cov[5] > 0.1))
      x = xd;
    else
      x << state->pos._value.x, state->pos._value.y, state->pos._value.z;
  else
    x = xd;

  if (state->pos._present && !std::isnan(state->pos._value.qw))
    if (state->pos_cov._present && (
          state->pos_cov._value.cov[9] > 0.1 ||
          state->pos_cov._value.cov[14] > 0.1 ||
          state->pos_cov._value.cov[20] > 0.1 ||
          state->pos_cov._value.cov[27] > 0.1))
      q = qd;
    else
      q.coeffs() <<
        state->pos._value.qx, state->pos._value.qy, state->pos._value.qz,
        state->pos._value.qw;
  else
    q = qd;

  R = q.matrix();

  if (state->vel._present && !std::isnan(state->vel._value.vx))
    if (state->vel_cov._present && (
          state->vel_cov._value.cov[0] > 0.1 ||
          state->vel_cov._value.cov[2] > 0.1 ||
          state->vel_cov._value.cov[5] > 0.1))
      v = vd;
    else
      v << state->vel._value.vx, state->vel._value.vy, state->vel._value.vz;
  else
    v = vd;

  if (state->vel._present && !std::isnan(state->vel._value.wx))
    if (state->vel_cov._present && (
          state->vel_cov._value.cov[9] > 0.1 ||
          state->vel_cov._value.cov[14] > 0.1 ||
          state->vel_cov._value.cov[20] > 0.1))
      w = wd;
    else
      w << state->vel._value.wx, state->vel._value.wy, state->vel._value.wz;
  else
    w = wd;


  /* position error */
  if (desired->pos._present) {
    ex = x - xd;
    for(i = 0; i < 2; i++)
      if (fabs(ex(i)) > 0.15) ex(i) = copysign(0.15, ex(i));
  } else
    ex << 0., 0., 0.;

  /* velocity error */
  if (desired->vel._present) {
    ev = v - vd;
    for(i = 0; i < 2; i++)
      if (fabs(ev(i)) > 0.15) ev(i) = copysign(0.15, ev(i));
  } else
    ev << 0., 0., 0.;


  /* desired orientation matrix */
  f =
    - servo->gain.Kx * ex - servo->gain.Kv * ev
    + servo->mass * (Eigen::Vector3d(0, 0, 9.81) + ad);
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
  t = - servo->gain.Kq * eR - servo->gain.Kw * ew;

  return 0;
}
