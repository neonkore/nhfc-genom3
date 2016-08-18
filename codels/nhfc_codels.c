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
