/* -*- c++ -*- */
/*
 * Copyright 2003,2004,2005 Free Software Foundation, Inc.
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

#include "sdrtx_basic.h"
#include "sdrtx_prims.h"
#include "sdrtx_interfaces.h"
#include "fusb.h"
#include <usb.h>
#include <stdio.h>
#include <stdexcept>
#include <assert.h>
#include <math.h>

#define NELEM(x) (sizeof (x) / sizeof (x[0]))

// These set the buffer size used for each end point using the fast
// usb interface.  The kernel ends up locking down this much memory.

static const int FUSB_BUFFER_SIZE = 8 * (1L << 20);	// 8 MB
static const int FUSB_BLOCK_SIZE = 16 * (1L << 10);	// 16KB is hard limit
static const int FUSB_NBLOCKS    = FUSB_BUFFER_SIZE / FUSB_BLOCK_SIZE;


static const double POLLING_INTERVAL = 0.1;	// seconds

////////////////////////////////////////////////////////////////

static struct usb_dev_handle *
open_tx_interface (struct usb_device *dev)
{
  struct usb_dev_handle *udh = sdrtx_open_tx_interface (dev);
  if (udh == 0){
    fprintf (stderr, "sdrtx_basic_tx: can't open tx interface\n");
    usb_strerror ();
  }
  return udh;
}


//////////////////////////////////////////////////////////////////
//
//			sdrtx_basic
//
////////////////////////////////////////////////////////////////


sdrtx_basic::sdrtx_basic (int which_board,
			char* firmware_filename,
			struct usb_dev_handle *
			open_interface (struct usb_device *dev))
  : d_udh (0)
{
    sdrtx_one_time_init ();

  //FIXME disabled for now
  if (!sdrtx_load_standard_bits (which_board, firmware_filename, false))
    throw std::runtime_error ("sdrtx_basic/sdrtx_load_standard_bits");

  //FIXME added true arguement to detect generic FX2
  struct usb_device *dev = sdrtx_find_device (which_board,true);
  if (dev == 0){
    fprintf (stderr, "sdrtx_basic: can't find sdrtx[%d]\n", which_board);
    throw std::runtime_error ("sdrtx_basic/sdrtx_find_device");
  }

  if ((d_udh = open_interface (dev)) == 0)
    throw std::runtime_error ("sdrtx_basic/open_interface");
}

sdrtx_basic::~sdrtx_basic ()
{
  if (d_udh)
    usb_close (d_udh);
}

bool
sdrtx_basic::initialize ()
{
  return true;		// nop
}


////////////////////////////////////////////////////////////////
//
//			   sdrtx_basic_tx
//
////////////////////////////////////////////////////////////////

sdrtx_basic_tx::sdrtx_basic_tx (int which_board, char* firmware_filename)
  : sdrtx_basic (which_board, firmware_filename, open_tx_interface),
    d_devhandle (0), d_ephandle (0),
    d_bytes_seen (0), d_first_write (true),
    d_tx_enable (false)
{
  d_devhandle = fusb_sysconfig::make_devhandle (d_udh);
  d_ephandle = d_devhandle->make_ephandle (SDRTX_TX_ENDPOINT, false,
					   FUSB_BLOCK_SIZE, FUSB_NBLOCKS);
}

sdrtx_basic_tx::~sdrtx_basic_tx ()
{
  d_ephandle->stop ();
  delete d_ephandle;
  delete d_devhandle;

}

bool
sdrtx_basic_tx::initialize ()
{
  if (!sdrtx_basic::initialize ())
    return false;
  
  if (!d_ephandle->start ()){
    fprintf (stderr, "sdrtx_basic_tx: failed to start end point streaming");
    usb_strerror ();
    return false;
  }

  return true;
}

sdrtx_basic_tx *
sdrtx_basic_tx::make (int which_board, char* firmware_filename)
{
  sdrtx_basic_tx *s = 0;
  
  try {
    s = new sdrtx_basic_tx (which_board, firmware_filename);
    if (!s->initialize ()){
      fprintf (stderr, "sdrtx_basic_tx::make failed to initialize\n");
      throw std::runtime_error ("sdrtx_basic_tx::make");
    }
    return s;
  }
  catch (...){
    delete s;
    return 0;
  }

  return s;
}

/*!
 * \brief Write data to the D/A's via the FX2.
 *
 * \p len must be a multiple of 1024 bytes.
 * \returns number of bytes written or -1 on error.
 *
 * if \p underrun is non-NULL, it will be set to true iff
 * a transmit underrun condition is detected.
 */
int
sdrtx_basic_tx::write (const void *buf, int len, bool *underrun)
{
  int	r;
  
  if (underrun)
    *underrun = false;
  
  if (len < 0 || (len % 1024) != 0){
    fprintf (stderr, "sdrtx_basic_tx::write: invalid length = %d\n", len);
    return -1;
  }

  r = d_ephandle->write (buf, len);
  if (r > 0)
    d_bytes_seen += r;

  return r;
}

void
sdrtx_basic_tx::wait_for_completion ()
{
  d_ephandle->wait_for_completion ();
}


