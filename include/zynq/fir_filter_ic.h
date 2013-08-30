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


#ifndef INCLUDED_ZYNQ_FIR_FILTER_IC_H
#define INCLUDED_ZYNQ_FIR_FILTER_IC_H

#include <zynq/api.h>
#include <gr_sync_block.h>

namespace gr {
  namespace zynq {

    /*!
     * \brief FPGA accelerated FIR filter
     * \ingroup FPGA
     *
     */
    class ZYNQ_API fir_filter_ic : virtual public gr_sync_block
    {
     public:
      typedef boost::shared_ptr<fir_filter_ic> sptr;

      /*!
       * \brief FPGA accelerated FIR filter
       *
       * 21 point, symmetric FIR filter implemented in FPGA fabric.
       *
       */
      static sptr make(const std::vector<int> &taps);
    };

  } // namespace zynq
} // namespace gr

#endif /* INCLUDED_ZYNQ_FIR_FILTER_IC_H */

