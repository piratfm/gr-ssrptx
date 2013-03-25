/* -*- c++ -*- */

#define SSRPTX_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "ssrptx_swig_doc.i"

%{
#include "ssrptx/ssrptx.h"
%}


%include "ssrptx/ssrptx.h"
GR_SWIG_BLOCK_MAGIC2(ssrptx, ssrptx);
