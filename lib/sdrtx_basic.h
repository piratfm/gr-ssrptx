/* -*- c++ -*- */
/*
 * Copyright 2003,2004 Free Software Foundation, Inc.
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

/*
 * ----------------------------------------------------------------------
 * Mid level interface to the Simple Software Radio Peripheral
 *
 * These classes implement the basic functionality for talking to the
 * SDRTX.  They try to be as independent of the signal processing code
 * in FPGA as possible.  They implement access to the low level
 * peripherals on the board, provide a common way for reading and
 * writing registers in the FPGA, and provide the high speed interface
 * to streaming data across the USB.
 *
 * It is expected that subclasses will be derived that provide
 * access to the functionality to a particular FPGA configuration.
 * ----------------------------------------------------------------------
 */

#ifndef INCLUDED_SDRTX_BASIC_H
#define INCLUDED_SDRTX_BASIC_H


struct usb_dev_handle;
class  fusb_devhandle;
class  fusb_ephandle;

/*!
 * \brief base class for sdrtx operations
 */
class sdrtx_basic {
private:
  // NOT IMPLEMENTED
  sdrtx_basic (const sdrtx_basic &rhs);			// no copy constructor
  sdrtx_basic &operator= (const sdrtx_basic &rhs);	// no assignment operator

  
protected:
  struct usb_dev_handle	*d_udh;

  sdrtx_basic (int which_board,
	      char* firmware_filename,
	      struct usb_dev_handle *open_interface (struct usb_device *dev));

  /*!
   * \brief called after construction in base class to derived class order
   */
  bool initialize ();

public:
  virtual ~sdrtx_basic ();

};

/*!
 * \brief class for accessing the transmit side of the SDRTX
 */
class sdrtx_basic_tx : public sdrtx_basic {
private:
  fusb_devhandle	*d_devhandle;
  fusb_ephandle		*d_ephandle;
  int			 d_bytes_seen;		// how many bytes we've seen
  bool			 d_first_write;
  bool			 d_tx_enable;

 protected:
  sdrtx_basic_tx (int which_board, char* firmware_filename);	// throws if trouble

  // called after construction in base class to derived class order
  bool initialize ();

public:

  ~sdrtx_basic_tx ();

  /*!
   * \brief invokes constructor, returns instance or 0 if trouble
   */
  static sdrtx_basic_tx *make (int which_board, char* firmware_filename);

  /*!
   * \brief Write data to the D/A's
   *
   * \p len must be a multiple of 512 bytes.
   * \returns number of bytes written or -1 on error.
   *
   * if \p underrun is non-NULL, it will be set to true iff
   * a transmit underrun condition is detected.
   */
  int write (const void *buf, int len, bool *underrun);

  /*
   * Block until all outstanding writes have completed.
   * This is typically used to assist with benchmarking
   */
  void wait_for_completion ();

};

#endif
