/*
 * Copyright (c) 2015 LAAS/CNRS
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


/* --- Task main -------------------------------------------------------- */


/** Codel nhfc_main_start of task main.
 *
 * Triggered by nhfc_start.
 * Yields to nhfc_ether.
 */
genom_event
nhfc_main_start(nhfc_ids_servo_s *servo, genom_context self)
{
  servo->att_pid.Kp = 2.7;	servo->att_pid.Kd = 0.35;
  servo->o_pid.Kp = 0.25;	servo->o_pid.Kd = 0.015;
  servo->xy_pid.Kp = 8;		servo->xy_pid.Kd = 6;
  servo->z_pid.Kp = 12;		servo->z_pid.Kd = 4;

  servo->mass = 0.950;
  servo->vmin = 18;		servo->vmax = 90;

  servo->xdes = servo->ydes = servo->zdes = servo->odes = 0.;
  servo->pulsedes = servo->raddes = 0.;

  return nhfc_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel nhfc_servo_start of activity servo.
 *
 * Triggered by nhfc_start.
 * Yields to nhfc_step.
 * Throws nhfc_e_input.
 */
FILE *flog;

genom_event
nhfc_servo_start(const nhfc_reference *reference, double *xdes,
                 double *ydes, double *zdes, double *odes,
                 genom_context self)
{
  const or_pose_estimator_state *ref_data = reference->data(self);
  double qw, qx, qy, qz;

  flog = fopen("log", "w");
  fprintf(flog, "# 1:ro 2:pi 3:ya 4:vro 5:vpi 6:vya 7:fz 8:tx 9:ty 10:tz\n");

  if (!ref_data) {
    *xdes = *ydes = *zdes = *odes = 0.;
    return nhfc_step;
  }

  if (reference->read(self) != genom_ok) return nhfc_e_input(self);
  if (!ref_data->pos._present) return nhfc_e_input(self);

  *xdes = ref_data->pos._value.x;
  *ydes = ref_data->pos._value.y;
  *zdes = ref_data->pos._value.z;

  qw = ref_data->pos._value.qw;
  qx = ref_data->pos._value.qx;
  qy = ref_data->pos._value.qy;
  qz = ref_data->pos._value.qz;
  *odes = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

  return nhfc_step;
}

/** Codel nhfc_servo_step of activity servo.
 *
 * Triggered by nhfc_step.
 * Yields to nhfc_pause_step.
 * Throws nhfc_e_input.
 */
genom_event
nhfc_servo_step(const nhfc_ids_servo_s *servo,
                const nhfc_reference *reference, const nhfc_imu *imu,
                const nhfc_cmd_wrench *cmd_wrench, genom_context self)
{
  const or_pose_estimator_state *refpos, *imudata;
  or_rotorcraft_ts_wrench *cmddata;

  double dt = 0.001;

  double thrust;
  double roll, pitch, yaw;
  static double vroll, vpitch, vyaw;
  double rolldes, pitchdes, sinrolldes, sinpitchdes;

  static double x, y, z;
  double qx, qy, qz, qw;
  static double lpvx, lpvy, lpvz;

  double xdes, ydes, zdes;
  double vxdes, vydes, vzdes;
  double axdes, aydes, azdes;
  static double t;

  double xerr, yerr, zerr, oerr;
  struct timeval tv;

  /* circle */
  t += dt;
  xdes = servo->xdes + servo->raddes * (cos(servo->pulsedes * t) - 1);
  ydes = servo->ydes + servo->raddes * sin(servo->pulsedes * t);
  zdes = servo->zdes + 0*servo->raddes * sin(servo->pulsedes * t);

  vxdes = - servo->raddes * servo->pulsedes * sin(servo->pulsedes * t);
  vydes = + servo->raddes * servo->pulsedes * cos(servo->pulsedes * t);
  vzdes = 0 * servo->raddes * servo->pulsedes * cos(servo->pulsedes * t);

  axdes = - servo->raddes *
    servo->pulsedes * servo->pulsedes * cos(servo->pulsedes * t);
  aydes = - servo->raddes *
    servo->pulsedes * servo->pulsedes * sin(servo->pulsedes * t);
  azdes = - 0 * servo->raddes *
    servo->pulsedes * servo->pulsedes * sin(servo->pulsedes * t);

  /* position */
  reference->read(self);
  imu->read(self);
  refpos = reference->data(self);
  imudata = imu->data(self);
  if (!imudata) return nhfc_stop;
  if (!imudata->vel._present) return nhfc_stop;

  if (refpos && refpos->pos._present) {
    x = refpos->pos._value.x;
    y = refpos->pos._value.y;
    z = refpos->pos._value.z;

    qw = refpos->pos._value.qw;
    qx = refpos->pos._value.qx;
    qy = refpos->pos._value.qy;
    qz = refpos->pos._value.qz;
    yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));
  } else
    yaw = 0.;

  qw = imudata->pos._value.qw;
  qx = imudata->pos._value.qx;
  qy = imudata->pos._value.qy;
  qz = imudata->pos._value.qz;
  roll = atan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx*qx + qy*qy));
  pitch = asin(2 * (qw*qy - qz*qx));

  /* linear velocity */
  if (refpos == NULL) {
    lpvx = vxdes;
    lpvy = vydes;
    lpvz = vzdes;
  } else {
    static or_time_ts ts;

    if (ts.sec != refpos->ts.sec || ts.nsec != refpos->ts.nsec) {
      static double px, py, pz;
      double dtref;
      double lpa = 0.5;

      dtref = (refpos->ts.sec - ts.sec) + (refpos->ts.nsec - ts.nsec)*1e-9;

      lpvx = lpa * lpvx + (1-lpa) * (x - px)/dtref;
      lpvy = lpa * lpvy + (1-lpa) * (y - py)/dtref;
      lpvz = lpa * lpvz + (1-lpa) * (z - pz)/dtref;

      px = x; py = y; pz = z;
      ts = refpos->ts;
    }
  }

  /* angular velocity */
  vroll = imudata->vel._value.wx;
  vpitch = imudata->vel._value.wy;
  vyaw = imudata->vel._value.wz;


  /* position control */
  if (refpos == NULL) {
    xerr = yerr = zerr = oerr = 0.;
  } else {
    xerr = xdes - x;
    if (fabs(xerr) > 0.15) xerr = copysign(0.15, xerr);

    yerr = ydes - y;
    if (fabs(yerr) > 0.15) yerr = copysign(0.15, yerr);

    zerr = zdes - z;
    if (fabs(zerr) > 0.15) zerr = copysign(0.15, zerr);

    oerr = servo->odes - yaw;
    if (fabs(oerr) > 15*M_PI/180.) oerr = copysign(15*M_PI/180., oerr);
  }

  thrust =
    servo->mass * (
      9.81 + azdes +
      servo->z_pid.Kp * zerr +
      servo->z_pid.Kd * (vzdes - lpvz)
      ) / cos(roll)/cos(pitch);
  if (fabs(thrust) > 20) thrust = copysign(20, thrust);

  sinrolldes = servo->mass/thrust *
    (sin(yaw) * (axdes +
                 servo->xy_pid.Kp * xerr +
                 servo->xy_pid.Kd * (vxdes - lpvx)) +
     -cos(yaw) * (aydes +
                 servo->xy_pid.Kp * yerr +
                 servo->xy_pid.Kd * (vydes - lpvy)));
  if (fabs(sinrolldes) > 0.95) sinrolldes = copysign(0.95, sinrolldes);
  rolldes = asin(sinrolldes);

  sinpitchdes = servo->mass/thrust/cos(roll) *
    (cos(yaw) * (axdes +
                 servo->xy_pid.Kp * xerr +
                 servo->xy_pid.Kd * (vxdes - lpvx)) +
     sin(yaw) * (aydes +
                 servo->xy_pid.Kp * yerr +
                 servo->xy_pid.Kd * (vydes - lpvy)));
  if (fabs(sinpitchdes) > 0.95) sinpitchdes = copysign(0.95, sinpitchdes);
  pitchdes = asin(sinpitchdes);

  /* wrench */
  cmddata = cmd_wrench->data(self);

  gettimeofday(&tv, NULL);
  cmddata->ts.sec = tv.tv_sec;
  cmddata->ts.nsec = tv.tv_usec * 1000;

  cmddata->w.f.x = 0.;
  cmddata->w.f.y = 0.;
  cmddata->w.f.z = thrust;

  cmddata->w.t.x =
    servo->att_pid.Kp * (rolldes - roll) +
    - servo->att_pid.Kd * vroll;
  cmddata->w.t.y =
    servo->att_pid.Kp * (pitchdes - pitch) +
    - servo->att_pid.Kd * vpitch;
  cmddata->w.t.z =
    servo->o_pid.Kp * oerr +
    - servo->o_pid.Kd * vyaw;

  cmd_wrench->write(self);

  printf("ro: %2.3f pi: %2.3f ya: %2.3f\n"
         "fz: %2.3f tx: %2.3f ty: %2.3f tz: %2.3f\n",
         roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI,
         cmddata->w.f.z, cmddata->w.t.x, cmddata->w.t.y, cmddata->w.t.z);
  fprintf(flog, "%f %f %f %f %f %f %f %f %f %f\n",
         roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI,
         vroll*180/M_PI, vpitch*180/M_PI, vyaw*180/M_PI,
         cmddata->w.f.z, cmddata->w.t.x, cmddata->w.t.y, cmddata->w.t.z);
  return nhfc_pause_step;
}

/** Codel mk_servo_stop of activity servo.
 *
 * Triggered by nhfc_stop.
 * Yields to nhfc_ether.
 * Throws nhfc_e_input.
 */
genom_event
mk_servo_stop(const nhfc_cmd_wrench *cmd_wrench, genom_context self)
{
  fclose(flog);
  return nhfc_ether;
}
