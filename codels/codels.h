/*
 * Copyright (c) 2016-2018 LAAS/CNRS
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
#ifndef H_NHFC_CODELS
#define H_NHFC_CODELS

#include <aio.h>

#include "nhfc_c_types.h"

#ifdef __cplusplus
extern "C" {
#endif

  int	nhfc_controller(const nhfc_ids_body_s *body,
                        const nhfc_ids_servo_s *servo,
                        const or_pose_estimator_state *state,
                        const or_pose_estimator_state *desired,
                        const or_wrench_estimator_state *exwrench,
                        nhfc_log_s *log,
                        or_rotorcraft_rotor_control *wprop);

  void	nhfc_invert_G(const double G[6 * or_rotorcraft_max_rotors],
                      double iG[or_rotorcraft_max_rotors * 6]);
  void	nhfc_Gw2(const double G[6 * or_rotorcraft_max_rotors], const double w,
                 double f[6]);

  int	nhfc_adm_filter(const nhfc_ids_body_s *body, const nhfc_ids_af_s *af,
                        const or_pose_estimator_state *reference,
                        const or_wrench_estimator_state *exwrench,
                        or_pose_estimator_state *desired);
  void	nhfc_adm_J(const double J[3 * 3]);

  int	nhfc_wrench_observer(const nhfc_ids_body_s *body,
                             const nhfc_ids_wo_s *wo,
                             const or_pose_estimator_state *state,
                             const double wprop[or_rotorcraft_max_rotors],
                             double xF[3], double xT[3]);

#ifdef __cplusplus
}
#endif

static inline genom_event
nhfc_e_sys_error(const char *s, genom_context self)
{
  nhfc_e_sys_detail d;
  size_t l = 0;

  d.code = errno;
  if (s) {
    strncpy(d.what, s, sizeof(d.what) - 3);
    l = strlen(s);
    strcpy(d.what + l, ": ");
    l += 2;
  }
  if (strerror_r(d.code, d.what + l, sizeof(d.what) - l)) { /* ignore error*/; }
  return nhfc_e_sys(&d, self);
}

struct nhfc_log_s {
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define nhfc_logfmt	" %g "
# define nhfc_log_header_fmt                                            \
  "ts "                                                                 \
  "fx fy fz tx ty tz "                                                  \
  "exfx exfy exfz extx exty extz "                                      \
  "xd yd zd rolld pitchd yawd vxd vyd vzd wxd wyd wzd axd ayd azd "     \
  "e_x e_y e_z e_vx e_vy e_vz e_rx e_ry e_rz e_wx e_wy e_wz"
# define nhfc_log_fmt                                                   \
  "%d.%09d "                                                            \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt                                   \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt
};

#endif /* H_NHFC_CODELS */
