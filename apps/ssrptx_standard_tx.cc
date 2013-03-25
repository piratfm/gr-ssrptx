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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <usb.h>			/* needed for usb functions */
#include <getopt.h>
#include <assert.h>
#include <math.h>
#include "time_stuff.h"
#include "sdrtx_standard.h"
#include "sdrtx_bytesex.h"

char *prog_name;

static bool test_output  (sdrtx_standard_tx *stx, bool float_input, FILE *fp);

static void
set_progname (char *path)
{
  char *p = strrchr (path, '/');
  if (p != 0)
    prog_name = p+1;
  else
    prog_name = path;
}

static void
usage ()
{
  fprintf (stderr, "usage: %s [-v] [-f frequency] [-a amplitude] [-i input_file]\n", prog_name);
  fprintf (stderr, "  [-f frequency] - set output freq in mhz\n");
  fprintf (stderr, "  [-a amplitude] - set output amplitude in [0.0...2.0]\n");
  fprintf (stderr, "  [-f] loop forever\n");
  fprintf (stderr, "  [-v] verbose\n");
  fprintf (stderr, "  [-F] float input\n");
  exit (1);
}

static void
die (const char *msg)
{
  fprintf (stderr, "die: %s: %s\n", prog_name, msg);
  exit (1);
}

int
main (int argc, char **argv)
{
  bool 	verbose_p = false;
  bool 	float_input = false;
  char	*input_filename = 0;
  int	which_board = 0;
  int   ch;
  float frequency = 0.0;
  float amplitude = 1.0;
  int ret;

  set_progname (argv[0]);

  while ((ch = getopt (argc, argv, "vFi:f:a:")) != EOF){
    switch (ch){
    case 'v':
      verbose_p = true;
      break;

    case 'i':
      input_filename = optarg;
      break;

    case 'f':
      frequency = atof(optarg);
      break;

    case 'a':
      amplitude = atof(optarg);
      break;
    case 'F':
      float_input = true;
      break;


    default:
      usage ();
    }
  }

  
  FILE *fp = 0;

  if (input_filename){
    fp = fopen (input_filename, "r");
    if (fp == 0)
      perror (input_filename);
  } else {
	fp = stdin;
  }
     

  sdrtx_standard_tx *stx = sdrtx_standard_tx::make (which_board, "txS_1024.ihx");
  if (stx == 0)
    die ("sdrtx_standard_tx::make");

  printf("Got Interface\n");
  frequency = frequency * 1000000;
  ret = stx->set_frequency_amplitude ((unsigned long) frequency, amplitude);
  if (ret){
	  fprintf (stderr, "test_output: freq/ampli failed, ret = %d\n", ret);
  }

  test_output (stx, float_input, fp);

  if (fp)
    fclose (fp);

  delete stx;

  return 0;
}


static bool
test_output  (sdrtx_standard_tx *stx, bool float_input, FILE *fp)
{
  int		   fd = -1;
  static const int BUFSIZE = 16 * 1024;
  //static int N = BUFSIZE/sizeof (short);
  static const int N = BUFSIZE/sizeof (short);
  short   buf[N];
  float   buf_fl[N];
  int		   nbytes = 0;
  int		   max_bytes = 128 * (1L << 20);
  //int		   max_bytes = 16384;

  if(float_input) {
	  printf("float input sizeof(float)=%d\n", sizeof(float));
  }


  double	   start_wall_time = get_elapsed_time ();
  double	   start_cpu_time  = get_cpu_usage ();

  if (fp)
    fd = fileno (fp);
  
  bool underrun;
  int nunderruns = 0;

  printf("sizeof (buf) %d\n", sizeof (buf));

#if 0
  for (unsigned int i = 0; i < sizeof (buf) / sizeof (short); i+=2)
  {
      buf[i] = (short)(100*sin( (6.28*i)/512 ) + 128)*256 + (100*sin( (6.28*i)/512 ) + 128);
      //printf ("%d: %d\n", i, buf[i]);
  }
#endif

  for (nbytes = 0; 1; nbytes += BUFSIZE){
#if 1
      if (fd != -1){

    if(float_input) {
    	  if (read (fd, buf_fl, sizeof(buf_fl)) != sizeof(buf_fl)){
    	      perror ("file read error\n");
    	      //fd = -1;
    	  }
  	  for (unsigned int i = 0; i < N; i++) {
  		  //if(buf_fl[i] > 32767.0f || buf_fl[i] < -32768.0f) {
  			//fprintf(stderr, "bad float: %.2f\n", buf_fl[i]);
  		  //}

  	  	  //−32,768	+32,767
  		  if(buf_fl[i] > 32767.0f)
  			buf_fl[i] = 32767.0f;
  		  if(buf_fl[i] < -32768.0f)
  			buf_fl[i] = -32768.0f;

  		  buf[i] = buf_fl[i];
  	  }
    } else {
  	  if (read (fd, buf, sizeof (buf)) != sizeof (buf)){
  	      perror ("file read error\n");
  	      //fd = -1;
  	  }
    }
#if 0
	  for (unsigned int i = 0; i < sizeof (buf) / sizeof (short); i++)
	      buf[i] = host_to_sdrtx_short (buf[i]);
#endif
      }
#endif
      int ret = stx->write (buf, sizeof (buf), &underrun);
      //int ret = stx->write (buf, max_bytes, &underrun);
      
      //printf("wrote: %d\n", nbytes);

      if (ret != sizeof (buf)){
	  fprintf (stderr, "test_output: error, ret = %d\n", ret);
      }

      if (underrun){
	  printf ("tx_underrun\n");
	  nunderruns++;
      }
    
  }

  if(float_input)
  	  free(buf_fl);

  double stop_wall_time = get_elapsed_time ();
  double stop_cpu_time  = get_cpu_usage ();

  double delta_wall = stop_wall_time - start_wall_time;
  double delta_cpu  = stop_cpu_time  - start_cpu_time;

  printf ("xfered %.3g bytes in %.3g seconds.  %.4g bytes/sec.  cpu time = %.4g\n",
	  (double) max_bytes, delta_wall, max_bytes / delta_wall, delta_cpu);
  printf ("nunderruns = %d\n", nunderruns);

  return true;
}
