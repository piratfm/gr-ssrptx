/* -*- c++ -*- */
/*
 * Copyright 2003,2004,2005 Free Software Foundation, Inc. 
 *
 * This file is part of the SDRTX project
 * based on code by the GNU Radio project
 * 
 * The SDRTX project is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 * 
 * The SDRTX Project is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with the SDRTX project; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
 * Low level primitives for directly messing with SDRTX hardware.
 *
 * If you're trying to use the SDRTX, you'll probably want to take a look
 * at the sdrtx_tx classes.  They hide a bunch of low level details
 * and provide high performance streaming i/o.
 *
 * This interface is built on top of libusb, which allegedly works under
 * Linux, *BSD and Mac OS/X.  http://libusb.sourceforge.net
 */

#ifndef _SDRTX_PRIMS_H_
#define _SDRTX_PRIMS_H_

static const int SDRTX_HASH_SIZE = 16;  //FIXME

enum sdrtx_load_status_t { ULS_ERROR = 0, ULS_OK, ULS_ALREADY_LOADED };

struct usb_dev_handle;
struct usb_device;

/*!
 * \brief initialize libusb; probe busses and devices.
 * Safe to call more than once.
 */
void sdrtx_one_time_init ();

/*
 * force a rescan of the buses and devices
 */
void sdrtx_rescan ();

/*!
 * \brief locate Nth (zero based) SDRTX device in system.
 * Return pointer or 0 if not found.
 *
 * The following kinds of devices are considered SDRTXs:
 *
 *   unconfigured SDRTX (no firwmare loaded)
 *   configured SDRTX (firmware loaded)
 *   unconfigured Cypress FX2 (only if fx2_ok_p is true)
 */
struct usb_device *sdrtx_find_device (int nth, bool fx2_ok_p = false);

bool sdrtx_sdrtx_p (struct usb_device *q);		//< is this a SDRTX
bool sdrtx_fx2_p (struct usb_device *q);			//< is this an unconfigured Cypress FX2

bool sdrtx_unconfigured_sdrtx_p (struct usb_device *q);	//< some kind of unconfigured SDRTX
bool sdrtx_configured_sdrtx_p (struct usb_device *q);	//< some kind of configured SDRTX

struct usb_dev_handle *sdrtx_open_nth_cmd_interface (int nth);

/*!
 * \brief given a usb_device return an instance of the appropriate usb_dev_handle
 *
 * These routines claim the specified interface and select the
 * correct alternate interface.  (USB nomenclature is totally screwed!)
 *
 * If interface can't be opened, or is already claimed by some other
 * process, 0 is returned.
 */
struct usb_dev_handle *sdrtx_open_tx_interface (struct usb_device *dev);

/*!
 * \brief close interface.
 */
bool sdrtx_close_interface (struct usb_dev_handle *udh);

/*!
 * \brief load intel hex format file into SDRTX/Cypress FX2 (8051).
 *
 * The filename extension is typically *.ihx
 *
 * Note that loading firmware may cause the device to renumerate.  I.e.,
 * change its configuration, invalidating the current device handle.
 */

sdrtx_load_status_t 
sdrtx_load_firmware (struct usb_dev_handle *udh, const char *filename, bool force);

/*!
 * \brief load intel hex format file into SDRTX FX2 (8051).
 *
 * The filename extension is typically *.ihx
 *
 * Note that loading firmware may cause the device to renumerate.  I.e.,
 * change its configuration, invalidating the current device handle.
 * If the result is ULS_OK, sdrtx_load_firmware_nth delays 1 second
 * then rescans the busses and devices.
 */
sdrtx_load_status_t
sdrtx_load_firmware_nth (int nth, const char *filename, bool force);

/*!
 * \brief load the regular firmware in the Nth SDRTX.
 *
 * This is the normal starting point...
 */
bool sdrtx_load_standard_bits (int nth, char* firmware_filename, bool force);

/*!
 * \brief copy the given \p hash into the SDRTX hash slot \p which.
 * The sdrtx implements two hash slots, 0 and 1.
 */
bool sdrtx_set_hash (struct usb_dev_handle *udh, int which,
		    const unsigned char hash[SDRTX_HASH_SIZE]);

/*!
 * \brief retrieve the \p hash from the SDRTX hash slot \p which.
 * The sdrtx implements two hash slots, 0 and 1.
 */
bool sdrtx_get_hash (struct usb_dev_handle *udh, int which,
		    unsigned char hash[SDRTX_HASH_SIZE]);

bool sdrtx_set_sleep_bits (struct usb_dev_handle *udh, int bits, int mask);

// i2c_read and i2c_write are limited to a maximum len of 64 bytes.

bool sdrtx_i2c_write (struct usb_dev_handle *udh, int i2c_addr,
		     const void *buf, int len);

bool sdrtx_i2c_read (struct usb_dev_handle *udh, int i2c_addr,
		    void *buf, int len);

// spi_read and spi_write are limited to a maximum of 64 bytes
// See spi_defs.h for more info
bool
sdrtx_spi_read (struct usb_dev_handle *udh,
	       int adr, unsigned long long int *dat_out, int len);

bool
sdrtx_spi_write (struct usb_dev_handle *udh,
		int adr, unsigned long long int dat, int len);

// Write 24LC024 / 24LC025 EEPROM on motherboard or daughterboard.
// Which EEPROM is determined by i2c_addr.  See i2c_addr.h
//FIXME - compatible?
bool sdrtx_eeprom_write (struct usb_dev_handle *udh, int i2c_addr,
			int eeprom_offset, const void *buf, int len);


// Read 24LC024 / 24LC025 EEPROM on motherboard or daughterboard.
// Which EEPROM is determined by i2c_addr.  See i2c_addr.h
//FIXME - compatible?
bool sdrtx_eeprom_read (struct usb_dev_handle *udh, int i2c_addr,
		       int eeprom_offset, void *buf, int len);


#endif /* _SDRTX_PRIMS_H_ */
