/*
 * Copyright (c) 2018 LAAS/CNRS
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
 *                                           Anthony Mallet on Wed Jan 24 2018
 */
#include "acnhfc.h"

#include <cmath>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "codels.h"

static Eigen::Matrix3d iJ;	/* inverse apparent inertia */


/*
 * --- nhfc_adm_filter -----------------------------------------------------
 *
 * Implements admittance filter on the desired state.
 */

int
nhfc_adm_filter(const nhfc_ids_body_s *body, const nhfc_ids_af_s *af,
                const or_pose_estimator_state *reference,
                const or_wrench_estimator_state *exwrench,
                or_pose_estimator_state *desired)
{
  using namespace Eigen;

  static const double dt = nhfc_control_period_ms / 1000.;

  Vector3d xr, vr, wr, ar;
  Quaternion<double> qr;
  Matrix3d Rr;

  Vector3d xd, vd, wd, ad, dwd;
  Quaternion<double> qd;
  Matrix3d Rd;

  Vector3d exF, exT;

  Vector3d ex, eR, ev, ew;
  Matrix3d E;

  Map< const Array<double, 6, 1> > B(af->B);
  Map< const Array<double, 6, 1> > K(af->K);
  const Vector3d force(af->force.x, af->force.y, af->force.z);
  const Vector3d torque(af->torque.x, af->torque.y, af->torque.z);


  /* reference */
  if (reference->pos._present) {
    xr <<
      reference->pos._value.x,
      reference->pos._value.y,
      reference->pos._value.z;
    qr.coeffs() <<
      reference->pos._value.qx,
      reference->pos._value.qy,
      reference->pos._value.qz,
      reference->pos._value.qw;
  } else {
    xr = Vector3d::Zero();
    qr = Quaternion<double>::Identity();
  }

  if (reference->vel._present) {
    vr <<
      reference->vel._value.vx,
      reference->vel._value.vy,
      reference->vel._value.vz;
    wr <<
      reference->vel._value.wx,
      reference->vel._value.wy,
      reference->vel._value.wz;
  } else {
    vr << Vector3d::Zero();
    wr << Vector3d::Zero();
  }


  /* desired */
  if (desired->pos._present) {
    xd <<
      desired->pos._value.x,
      desired->pos._value.y,
      desired->pos._value.z;
    qd.coeffs() <<
      desired->pos._value.qx,
      desired->pos._value.qy,
      desired->pos._value.qz,
      desired->pos._value.qw;

    if (!reference->pos._present) {
      xr = xd;
      qr = qd;
    }
  } else {
    xd = xr;
    qd = qr;
  }

  if (desired->vel._present) {
    vd <<
      desired->vel._value.vx,
      desired->vel._value.vy,
      desired->vel._value.vz;
    wd <<
      desired->vel._value.wx,
      desired->vel._value.wy,
      desired->vel._value.wz;
  } else {
    vd = vr;
    wd = wr;
  }


  /* external wrench */
  if (exwrench->force._present) {
    exF <<
      exwrench->force._value.x,
      exwrench->force._value.y,
      exwrench->force._value.z;
  } else {
    exF = Vector3d::Zero();
  }

  if (exwrench->torque._present) {
    exT <<
      exwrench->torque._value.x,
      exwrench->torque._value.y,
      exwrench->torque._value.z;
  } else {
    exT = Vector3d::Zero();
  }


  /* position error */
  ex = xr - xd;

  /* orientation error */
  Rr = qr.matrix();
  Rd = qd.matrix();

  E = 0.5 * (Rd.transpose()*Rr - Rr.transpose()*Rd);
  eR <<
    (E(2, 1) - E(1, 2))/2.,
    (E(0, 2) - E(2, 0))/2.,
    (E(1, 0) - E(0, 1))/2.;

  /* velocity error */
  ev = vr - vd;
  ew = wr - wd;


  /* desired acceleration */
  ad =
    ar +
    1/af->mass * (
      (K.block<3, 1>(0, 0) * ex.array()).matrix() +
      (B.block<3, 1>(0, 0) * ev.array()).matrix() +
      exF +
      force);

  dwd =
    iJ * Vector3d(
      (K.block<3, 1>(3, 0) * eR.array()).matrix() +
      (B.block<3, 1>(3, 0) * ew.array()).matrix() +
      exT +
      torque
      );

  /* integration */
  xd += vd * dt + ad * dt*dt/2;
  vd += ad * dt;
  wd += dwd * dt;

  Quaternion<double> dq;
  double a2 = dt * dt * wd.squaredNorm();
  if (a2 < 1e-1) {
    dq.w() = 1 - a2/8 /*std::cos(a/2)*/;
    dq.vec() = (0.5 - a2/48 /*std::sin(a/2)/a*/) * dt * wd;
  } else {
    double a = std::sqrt(a2);
    dq.w() = std::cos(a/2);
    dq.vec() = std::sin(a/2)/a * dt * wd;
  }
  qd = dq * qd;


  /* output */
  desired->ts = reference->ts;

  desired->pos._present = true;
  desired->pos._value.x = xd(0);
  desired->pos._value.y = xd(1);
  desired->pos._value.z = xd(2);
  desired->pos._value.qx = qd.vec()(0);
  desired->pos._value.qy = qd.vec()(1);
  desired->pos._value.qz = qd.vec()(2);
  desired->pos._value.qw = qd.w();

  desired->vel._present = true;
  desired->vel._value.vx = vd(0);
  desired->vel._value.vy = vd(1);
  desired->vel._value.vz = vd(2);
  desired->vel._value.wx = 0.; /* XXX wd(0) */
  desired->vel._value.wy = 0.; /* XXX wd(1) */
  desired->vel._value.wz = wd(2);

  desired->acc._present = true;
  desired->acc._value.ax = ad(0);
  desired->acc._value.ay = ad(1);
  desired->acc._value.az = ad(2);

  return 0;
}


/*
 * --- nhfc_adm_J ----------------------------------------------------------
 *
 */

void
nhfc_adm_J(const double J[3 * 3])
{
  Eigen::Map< const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > J_(J);

  iJ = J_.inverse();
}
