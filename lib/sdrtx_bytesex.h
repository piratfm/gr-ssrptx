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
#ifndef INCLUDED_SDRTX_BYTESEX_H
#define INCLUDED_SDRTX_BYTESEX_H

/*!
 * \brief routines for convertering between host and sdrtx byte order
 *
 * Prior to including this file, the user must include "config.h"
 * which will or won't define WORDS_BIGENDIAN based on the
 * result of the AC_C_BIGENDIAN autoconf test.
 */

#ifdef HAVE_BYTESWAP_H
#include <byteswap.h>
#else
static inline unsigned short int
bswap_16 (unsigned short int x)
{
  return ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8));
}
#endif


#ifdef WORDS_BIGENDIAN

static inline short int
host_to_sdrtx_short (short int x)
{
  return bswap_16 (x);
}

static inline short int
sdrtx_to_host_short (short int x)
{
  return bswap_16 (x);
}

#else

static inline short int
host_to_sdrtx_short (short int x)
{
  return x;
}

static inline short int
sdrtx_to_host_short (unsigned short int x)
{
  return x;
}

#endif

#endif /* INCLUDED_SDRTX_BYTESEX_H */
