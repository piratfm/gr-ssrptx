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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


#include <stdio.h>
#include <gr_io_signature.h>
#include "ssrptx_impl.h"

#include "sdrtx_standard.h"

namespace gr {
  namespace ssrptx {

    ssrptx::sptr
    ssrptx::make(const int device_id, const int frequency, const float gain)
    {
      return gnuradio::get_initial_sptr (new ssrptx_impl(device_id, frequency, gain));
    }

    /*
     * The private constructor
     */
    ssrptx_impl::ssrptx_impl(const int device_id, const int frequency, const float gain)
      : gr_sync_block("ssrptx",
		      gr_make_io_signature(1, 1, sizeof (short) * 2),
		      gr_make_io_signature(0, 0, 0))
    {
        fprintf(stderr, "device_id=%d, frequency=%d, gain=%.1f\n", device_id, frequency, gain);
        sdrtx_standard_tx *stx = sdrtx_standard_tx::make (device_id, "txS_1024.ihx");
      	stx->set_frequency_amplitude (frequency, gain);
      	stx_ptr = (void *) stx;


    	set_output_multiple (4 * 1024 / sizeof (short));
    }

    /*
     * Our virtual destructor.
     */
    ssrptx_impl::~ssrptx_impl()
    {
    	sdrtx_standard_tx *stx = (sdrtx_standard_tx *) stx_ptr;
    	stx->~sdrtx_standard_tx ();
    }

    int
    ssrptx_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const short *in = (const short *) input_items[0];
        float *out = (float *) output_items[0];
        bool underrun;
        sdrtx_standard_tx *stx = (sdrtx_standard_tx *) stx_ptr;

        int ret = stx->write (in, noutput_items * sizeof(short)*2, &underrun);
        //int ret = stx->write (buf, max_bytes, &underrun);

        //printf("wrote: %d\n", nbytes);

        if (ret != noutput_items * sizeof(short) * 2){
        	fprintf (stderr, "output: error, ret = %d\n", ret);
        }

        //fprintf(stderr, "data[%d]: %d %d, %d %d\n", noutput_items, in[0], in[1], in[2], in[3]);
        // Do <+signal processing+>

        // Tell runtime system how many output items we produced.
        return noutput_items;
    }

  } /* namespace ssrptx */
} /* namespace gr */

