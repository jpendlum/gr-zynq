#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2013 <+YOU OR YOUR COMPANY+>.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

from gnuradio import gr, gr_unittest
import zynq_swig as zynq

class qa_fir_filter_ii (gr_unittest.TestCase):

    def setUp (self):
        self.tb = gr.top_block ()

    def tearDown (self):
        self.tb = None

    def test_001_t (self):
        # Check impulse response
        # Impulse is 2^15 due to the fixed point format of the coefficients. With a large enough impulse,
        # the filter's impulse response will simply be the filter coefficients.
        src_data = (2**15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        expected_result = (-51,-662,-190,510,12,-719,238,945,-671,-1161,1438,1341,-3060,-1460,10287,17886,10287,-1460,-3060,1341,1438,-1161,-671,945,238,-719,12,510,-190,-662,-51)
        filter_taps = (-51,-662,-190,510,12,-719,238,945,-671,-1161,1438,1341,-3060,-1460,10287,17886)
        # Filter coefficients are fx1.31, so scale them up to allow us to retreive the filter coefficients from the impulse response.
        filter_taps = [x * 2**16 for x in filter_taps]
        src = gr.vector_source_i(src_data)
        # FIR filter taps are set to expected_result
        fir_filter = zynq.fir_filter_ii(filter_taps)
        dst = gr.vector_sink_i()
        self.tb.connect(src, fir_filter)
        self.tb.connect(fir_filter, dst)
        self.tb.run()
        result_data = dst.data()
        self.assertEqual(result_data,expected_result)
        self.tb.disconnect(src, fir_filter)
        self.tb.disconnect(fir_filter, dst)

if __name__ == '__main__':
    gr_unittest.run(qa_fir_filter_ii)
