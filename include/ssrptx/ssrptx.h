/* -*- c++ -*- */
/* 
 * Copyright 2013 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_SSRPTX_SSRPTX_H
#define INCLUDED_SSRPTX_SSRPTX_H

#include <ssrptx/api.h>
#include <gr_sync_block.h>

namespace gr {
  namespace ssrptx {

    /*!
     * \brief SSRPTX API
     * \ingroup ssrptx
     *
     */
    class SSRPTX_API ssrptx : virtual public gr_sync_block
    {
     public:
      typedef boost::shared_ptr<ssrptx> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of ssrptx::ssrptx.
       *
       * To avoid accidental use of raw pointers, ssrptx::ssrptx's
       * constructor is in a private implementation
       * class. ssrptx::ssrptx::make is the public interface for
       * creating new instances.
       */
      static sptr make(const int device_id=0, const int frequency=145500000, const float gain = 1.0);
    };

  } // namespace ssrptx
} // namespace gr

#endif /* INCLUDED_SSRPTX_SSRPTX_H */

