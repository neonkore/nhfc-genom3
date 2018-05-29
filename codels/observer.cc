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
 *                                           Anthony Mallet on Mon Jan 22 2018
 */
#include "acnhfc.h"

#include <cmath>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "codels.h"

/*
 * This file implements a wrench observer, as described in:
 *
 * T. Tomić and C. Ott and S. Haddadin, "External Wrench Estimation, Collision
 * Detection, and Reflex Reaction for Flying Robots", IEEE Transactions on
 * Robotics, vol. 33, no. 6, pp. 1467-1482, December 2017.
 */

int
nhfc_wrench_observer(const nhfc_ids_body_s *body,
                     const nhfc_ids_wo_s *wo,
                     const or_pose_estimator_state *state,
                     const double wprop[or_rotorcraft_max_rotors],
                     double xF[3], double xT[3])
{
  using namespace Eigen;

  static Vector3d exF(Vector3d::Zero()); /* estimated external force */
  static Vector3d exT(Vector3d::Zero()); /* estimated external torque */
  static Vector3d eL(Vector3d::Zero()); /* estimated angular momentum */

  Quaternion<double> q; /* orientation */
  Vector3d w; /* angular velocity (world frame) */
  Vector3d a; /* acceleration (world frame) */
  Matrix<double, or_rotorcraft_max_rotors, 1> wprop2_; /* propeller vel.^2 */

  Vector3d L; /* angular momentum (world frame) */

  Map< const Matrix<double,
                    6, or_rotorcraft_max_rotors, RowMajor> > G(body->G);
  Map< const Matrix<double, 3, 3, RowMajor> > J(body->J);
  size_t i;

  /* current state - XXX do something if state not present / uncertain */
  if (state->att._present && !std::isnan(state->att._value.qw))
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;
  else
    q = Quaternion<double>::Identity();

  if (state->avel._present && !std::isnan(state->avel._value.wx))
    w << state->avel._value.wx, state->avel._value.wy, state->avel._value.wz;
  else
    w = Vector3d::Zero();

  if (state->acc._present && !std::isnan(state->acc._value.ax))
    a << state->acc._value.ax, state->acc._value.ay, state->acc._value.az;
  else
    a = Vector3d::Zero();

  wprop2_ =
    Map< const Array<double, or_rotorcraft_max_rotors, 1> >(wprop).square();


  /* force estimation */
  exF +=
    Map< const Matrix<double, 3, 1> >(wo->K).asDiagonal() * (
      - exF
      + (
        body->mass * (a + Vector3d(0, 0, 9.81))
        - q.matrix() * G.block<3, or_rotorcraft_max_rotors>(0, 0) * wprop2_
        )
      ) * nhfc_wo_period_ms/1000.;


  /* torque estimation */
  L = q * J * (q.conjugate() * w);
  eL +=
    ( L.cross(w)
      + q * ( G.block<3, or_rotorcraft_max_rotors>(3, 0) * wprop2_ )
      + exT ) * nhfc_wo_period_ms/1000.;

  exT =
    Map< const Matrix<double, 3, 1> >(&wo->K[3]).asDiagonal() * (L - eL);


  /* output */
  for(i = 0; i < 3; i++) {
    xF[i] = exF(i) - wo->bias[i];
    xT[i] = exT(i) - wo->bias[i + 3];
  }

  return 0;
}
