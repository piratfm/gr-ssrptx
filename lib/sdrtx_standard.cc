/* -*- c++ -*- */
/*
 * Copyright 2004 Free Software Foundation, Inc.
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

#include <sdrtx_standard.h>

#include "sdrtx_prims.h"
#include <stdexcept>
#include <stdio.h>
#include <assert.h>
#include <math.h>

#define NELEM(x) (sizeof (x) / sizeof (x[0]))

//////////////////////////////////////////////////////////////////

sdrtx_standard_tx::sdrtx_standard_tx (int which_board, char* firmware_filename)
  : sdrtx_basic_tx (which_board, firmware_filename)
{
}

sdrtx_standard_tx::~sdrtx_standard_tx ()
{
  // nop
}

bool
sdrtx_standard_tx::initialize ()
{
  if (!sdrtx_basic_tx::initialize ())
    return false;

  // add our code here

  usb_dev_handle *dh = sdrtx_open_nth_cmd_interface(0);
  sdrtx_spi_write (dh, 0x00, 0x00600000, 4); //CFR1
  //sdrtx_spi_write (dh, 0x01, 0x01000E60, 4); //CFR2 - keep last val
  sdrtx_spi_write (dh, 0x01, 0x01000E20, 4); //CFR2 - last val set to zero
  sdrtx_spi_write (dh, 0x02, 0x1118c12a, 4); //CFR3
  sdrtx_spi_write (dh, 0x03, 0x000000ff, 4); //Auxiliary DAC Control Register

  //sdrtx_spi_write (dh, 0x09, 0x000022D4, 8); //Amplitude Scale Factor (ASF) Register

//7D => 194.064
// round(2^32 ⋅ (194064000/(24576000⋅21))) = 0x60430C31
//
//./spiwrite 0x0e 1 8 0xFC80000060430C31 # Profile 0 Register—Single Tone|QDUC 2048sr
//  sdrtx_spi_write (s->d_devhandle, int adr, unsigned long long int dat, int len);
//  sdrtx_spi_write (dh, 0x0e, 0xFC80000060430C31, 8);
  sdrtx_close_interface (dh);



  return true;
}

sdrtx_standard_tx *
sdrtx_standard_tx::make (int which_board, char* firmware_filename)
{
  sdrtx_standard_tx *s = 0;
  
  try {
    s = new sdrtx_standard_tx (which_board, firmware_filename);
    if (!s->initialize ()){
      fprintf (stderr, "sdrtx_standard_tx::make failed to initialize\n");
      throw std::runtime_error ("sdrtx_standard_tx::make");
    }
    return s;
  }
  catch (...){
    delete s;
    return 0;
  }

  return s;
}

int sdrtx_standard_tx::set_frequency_amplitude (unsigned long frequency, float amplitude)
{
	double frq1 = round(pow(2,32)/(24576000 * 21) * frequency);
	unsigned long long int freq_word = (unsigned long long int) frq1;
	float amplitude_data = 128.0f * amplitude;

	fprintf (stderr, "sdrtx_standard_tx::frq1:  %.2f\n",frq1);
	fprintf (stderr, "sdrtx_standard_tx::frequency:  %ld => 0x%016llx, amplitude %.1f => %.2f\n", frequency, freq_word, amplitude, amplitude_data);

	unsigned long long int amplitude_data2 = (unsigned char) amplitude_data;
	freq_word |= 0xFC00000000000000 | amplitude_data2 << (6*8);
	fprintf (stderr, "sdrtx_standard_tx::FTW: 0x%016llx\n", freq_word);
	usb_dev_handle *dh = sdrtx_open_nth_cmd_interface(0);
	sdrtx_spi_write (dh, 0x0e, freq_word, 8);
	sdrtx_close_interface (dh);

	return 0;
}
