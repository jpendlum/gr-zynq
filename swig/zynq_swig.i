/* -*- c++ -*- */

#define ZYNQ_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "zynq_swig_doc.i"

%{
#include "zynq/fir_filter_ii.h"
%}

%include "zynq/fir_filter_ii.h"
GR_SWIG_BLOCK_MAGIC2(zynq, fir_filter_ii);

