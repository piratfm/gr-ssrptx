/* -*- c++ -*- */
/*
 * Copyright 2003 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "time_stuff.h"

#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

static double
timeval_to_secs (struct timeval *tv)
{
  return (double) tv->tv_sec + (double) tv->tv_usec * 1e-6;
}

double
get_cpu_usage (void)
{
  struct rusage	ru;

  if (getrusage (RUSAGE_SELF, &ru) != 0)
    return 0;

  return timeval_to_secs (&ru.ru_utime) + timeval_to_secs (&ru.ru_stime);
}

/*
 * return elapsed time (wall time) in seconds
 */
double
get_elapsed_time (void)
{
  struct timeval	tv;
  if (gettimeofday (&tv, 0) != 0)
    return 0;

  return timeval_to_secs (&tv);
}

