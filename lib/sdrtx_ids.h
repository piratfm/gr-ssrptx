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

/*
 * USB Vendor and Product IDs that we use
 *
 * (keep in sync with usb_descriptors.a51)
 */

#ifndef _SDRTX_IDS_H_
#define _SDRTX_IDS_H_


#define		USB_VID_CYPRESS			0x04b4
#define		USB_PID_CYPRESS_FX2		0x8613
//#define	USB_VID_CYPRESS			0x0925
//#define	USB_PID_CYPRESS_FX2		0x3881


//#define	USB_VID_FSF			0x04b4	  // Free Software Folks
//#define	USB_VID_FSF			0x0925	  // Free Software Folks
#define	USB_VID_FSF				0xfffe	  // Free Software Folks
#define	USB_PID_FSF_EXP_0		0x0000	  // Experimental 0
#define	USB_PID_FSF_EXP_1		0x0001	  // Experimental 1

#define	USB_PID_FSF_SDRTX		0x0002	  // Universal Software Radio Peripheral
//#define	USB_PID_FSF_SDRTX		0x3881	  // Universal Software Radio Peripheral

#define	USB_DID_SDRTX_0			0x8613	  // unconfigured rev 0 SDRTX
#define	USB_DID_SDRTX_1			0x0001	  // unconfigured rev 1 SDRTX

#endif /* _SDRTX_IDS_H_ */
