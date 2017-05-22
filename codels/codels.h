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
#ifndef H_NHFC_CODELS
#define H_NHFC_CODELS

#include "nhfc_c_types.h"

#ifdef __cplusplus
extern "C" {
#endif

  int	nhfc_controller(const nhfc_ids_servo_s *servo,
                        const or_pose_estimator_state *state,
                        const or_pose_estimator_state *desired,
                        const nhfc_log_s *log,
                        double *thrust, double torque[3]);

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
  FILE *f;

# define nhfc_logfmt	" %e "
# define nhfc_log_header_fmt                                            \
  "ts thrust fx fy fz tx ty tz "                                        \
  "xd yd zd rolld pitchd yawd vxd vyd vzd wxd wyd wzd axd ayd azd "     \
  "e_x e_y e_z e_vx e_vy e_vz e_rx e_ry e_rz e_wx e_wy e_wz"
# define nhfc_log_fmt                                                   \
  "%d.%09d " nhfc_logfmt                                                \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt                                   \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt \
  nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt nhfc_logfmt
};

#endif /* H_NHFC_CODELS */
