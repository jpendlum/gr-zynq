/* -*- c++ -*- */
/****************************************************************************
 *
 * Copyright 2013 Jonathon Pendlum
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *
 * Author:      Jonathon Pendlum (jon.pendlum@gmail.com)
 * Description: FPGA accelerated 31 tap, fixed point (fx1.31), symmetric
 *              FIR filter with reloadable coefficients.
 *
 ***************************************************************************/

#ifndef INCLUDED_ZYNQ_FIR_FILTER_II_IMPL_H
#define INCLUDED_ZYNQ_FIR_FILTER_II_IMPL_H

#include <zynq/fir_filter_ii.h>

namespace gr {
  namespace zynq {

    class fir_filter_ii_impl : public fir_filter_ii
    {
     private:
      int d_fd;
      unsigned long int d_buffer_length;
      unsigned long int d_control_length;
      unsigned long int d_phys_addr;
      unsigned int *d_control_regs;
      long long int *d_buff;

      int open_driver();
      int close_driver();
      int read_fpga_status();
      int get_params_from_sysfs();

     public:
      fir_filter_ii_impl(const std::vector<int> &taps);
      ~fir_filter_ii_impl();

      void set_taps(const std::vector<int> &taps);

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };

  } // namespace zynq
} // namespace gr

#endif /* INCLUDED_ZYNQ_FIR_FILTER_II_IMPL_H */
