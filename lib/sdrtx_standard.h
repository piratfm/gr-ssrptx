/* -*- c++ -*- */
/*
 * Copyright 2004,2005 Free Software Foundation, Inc.
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

#ifndef INCLUDED_SDRTX_STANDARD_H
#define INCLUDED_SDRTX_STANDARD_H

#include <sdrtx_basic.h>

// ----------------------------------------------------------------

/*!
 * \brief standard sdrtx TX class.
 *
 */
class sdrtx_standard_tx : public sdrtx_basic_tx
{

 protected:
  sdrtx_standard_tx (int which_board, char* firmware_filename);	// throws if trouble

  // called after construction in base class to derived class order
  bool initialize ();

 public:
  ~sdrtx_standard_tx ();

  /*!
   * \brief invokes constructor, returns instance or 0 if trouble
   */
  static sdrtx_standard_tx *make (int which_board, char* firmware_filename);
  int set_frequency_amplitude (unsigned long frequency, float amplitude);

};

#endif /* INCLUDED_SDRTX_STANDARD_H */
