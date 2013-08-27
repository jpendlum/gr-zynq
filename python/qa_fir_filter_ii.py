#!/usr/bin/env python
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

    def test_fir_filter_001 (self):
        # Check impulse response
        # Impulse is 2^5 due to truncation in the FPGA
        src_data = (32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        expected_result = (6, 0, -4, -3,  5, 6, -6, -13, 7, 44, 64, 44, 7, -13, -6, 6, 5, -3, -4, 0, 6)
        filter_taps = (6, 0, -4, -3,  5, 6, -6, -13, 7, 44, 64)
        src = gr.vector_source_i(src_data)
        # FIR filter taps are set to expected_result
        fir_filter = zynq.fir_filter_ii(filter_taps)
        dst = gr.vector_sink_i()
        self.tb.connect(src, fir_filter)
        self.tb.connect(fir_filter, dst)
        self.tb.run()
        result_data = dst.data()
        self.assertEqual(result_data,expected_result)

if __name__ == '__main__':
    gr_unittest.run(qa_fir_filter_ii)