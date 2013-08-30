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
 * Description: FPGA accelerated 31 tap, fixed point (fx17.15), symmetric
 *              FIR filter with reloadable coefficients.
 *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gr_io_signature.h>
#include "fir_filter_ii_impl.h"
#include <zynq/api.h>
#include <gr_sync_block.h>
#include <stdio.h>
#include <stdlib.h>
// #include <time.h>
#include <libudev.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <stdexcept>
#include <iostream>
#include <fstream>

// The kernel driver allocates 2^17 bytes (2^15 addressable) for control registers
// and 2^10*4096 bytes for data.
//
// The control registers are broken into three banks based on Bits 13:12 of the memory
// address:
// 00: host to slave commands (reads)
// 01: slave to host commands (writes)
// 10: global settings
// 11: Unused
#define OFFSET_H2S          (0)
#define OFFSET_S2H          (1 << 10)
#define OFFSET_GLOBAL       (1 << 11)
#define OFFSET_FIR_SAMP_WR  (0*8) + OFFSET_H2S // AXI Stream 0 (see FPGA code)
#define OFFSET_FIR_SAMP_RD  (0*8) + OFFSET_S2H // AXI Stream 0
#define OFFSET_FIR_COEFF_WR (1*8) + OFFSET_H2S // AXI Stream 1
#define OFFSET_FIR_COEFF_RD (1*8) + OFFSET_S2H // AXI Stream 1
#define OFFSET_STREAM2_WR   (2*8) + OFFSET_H2S // AXI Stream 2 Unused
#define OFFSET_STREAM2_RD   (2*8) + OFFSET_S2H // AXI Stream 2 Unused
#define OFFSET_STREAM3_WR   (3*8) + OFFSET_H2S // AXI Stream 3 Unused
#define OFFSET_STREAM3_RD   (3*8) + OFFSET_S2H // AXI Stream 3 Unused


#define FIFO_WR_CLEAR       0
#define FIFO_WR_ADDR        1
#define FIFO_WR_SIZE        2
#define FIFO_WR_STS_RDY     3
#define FIFO_WR_STS         4
#define FIFO_RD_SIG         0
#define FIFO_RD_STATUS      1
#define FIFO_RD_STATUS_CNT  2
#define FIFO_RD_ADDR_CNT    3
#define FIFO_RD_SIZE_CNT    4

// Transfers to the FPGA fabric are costly in terms of processing time, so sending
// too few samples is wasteful.
#define MIN_NUM_SAMPLES     1
// The AXI ACP bus read / write transfer rates are not symmetric, so a FIFO exists in the
// FPGA fabric to account for the rate difference. This prevents sending too many samples that
// might cause the FIFO to overflow if a severe rate difference
#define MAX_NUM_SAMPLES     8196
// Number of unique taps (FIR filter is symmetric)
#define NUM_TAPS            16

namespace gr {
  namespace zynq {
    // FIXME: Hack to ensure only one FPGA accelerator instance occurs
    static int g_init = 0;

    fir_filter_ii::sptr
    fir_filter_ii::make(const std::vector<int> &taps)
    {
      return gnuradio::get_initial_sptr(new fir_filter_ii_impl(taps));
    }

    /*
     * The private constructor
     */
    fir_filter_ii_impl::fir_filter_ii_impl(const std::vector<int> &taps)
      : gr_sync_block("fpga_fir_filter_ii",
          gr_make_io_signature(1, 1, sizeof(int)),
          gr_make_io_signature(1, 1, sizeof(int)))
    {
      if (g_init == 1)
      {
        throw std::runtime_error("Only one accelerator instance supported (fix in future version!)\n");
      }

      if (this->read_fpga_status() != 0)
      {
        throw std::runtime_error("FPGA not programmed!\n");
      }
      if (this->open_driver() != 0)
      {
        throw std::runtime_error("Failed to properly open driver!\n");
      };
      g_init = 1;
      // Reset everything so we start in a known state
      d_control_regs[OFFSET_GLOBAL] = 0;                // Soft reset command (resets everything)
      // Useful debugging information
      // printf("\n");
      // printf("Control Registers Address: \t%p\n",d_control_regs);
      // printf("Control Registers Length: \t%lx\n",d_control_length);
      // printf("Sample Buffer Address: \t\t%p\n",d_buff);
      // printf("Sample Buffer Length: \t\t%lx\n",d_buffer_length);
      // printf("Read signature: \t\t%x\n",d_control_regs[FIFO_RD_SIG]);
      // printf("Status FIFO output: \t\t%x\n",d_control_regs[FIFO_RD_STATUS]);
      // printf("Status FIFO Word Count: \t%d\n",d_control_regs[FIFO_RD_STATUS_CNT]);
      // printf("Address FIFO Word Count: \t%d\n",d_control_regs[FIFO_RD_ADDR_CNT]);
      // printf("Size FIFO Word Count: \t\t%d\n",d_control_regs[FIFO_RD_SIZE_CNT]);
      this->set_taps(taps);
    }

    /*
     * Our virtual destructor.
     */
    fir_filter_ii_impl::~fir_filter_ii_impl()
    {
      this->close_driver();
      g_init = 0;
    }

    /*
     * The FIR filter coefficients are loaded by writing them to the kernel buffer and
     * loading those coefficients on stream 1 (see FPGA code). Once loaded, the new coefficients
     * replace the old automatically.
     * Coefficeints are fixed point fx17.15, i.e. 17 integer bits & 15 fraction bits.
     */
    void fir_filter_ii_impl::set_taps(const std::vector<int> &taps)
    {
      // Make sure the correct number of taps are provided
      if (taps.size() != NUM_TAPS)
      {
        throw std::runtime_error("Incorrect number of filter coefficients!\n");
      }
      // Write taps into kernel buffer
      for (int i = 0; i < taps.size(); i++)
      {
        d_buff[i] = (long long int)taps[i];
        //printf("d_buff[%2d](%p)=%lld\n",i,&d_buff[i],d_buff[i]);
      }
      // Load new coefficients by setting the stream to 2 (see OFFSET_FIR_RELOAD) and writing
      // the physical address + size to the control register FIFOs.
      d_control_regs[OFFSET_FIR_RELOAD+FIFO_WR_ADDR] = d_phys_addr;
      d_control_regs[OFFSET_FIR_RELOAD+FIFO_WR_SIZE] = taps.size() * sizeof(long long int);
      read(d_fd,0,0);
    }

    /*
     * Work function copies samples to the kernel buffer, sends memory read / write commands
     * to the FPGA via the AXI slave bus, and copies filtered samples back into the output
     * buffer.
     */
    int fir_filter_ii_impl::work(int noutput_items,
                                 gr_vector_const_void_star &input_items,
                                 gr_vector_void_star &output_items)
    {
      const int *in = (const int *) input_items[0];
      int *out = (int *) output_items[0];
      int i = 0;
      int val = 0;
      int num_samples = 0;

      if (noutput_items < MIN_NUM_SAMPLES)
      {
        return(0);
      }
      else if (noutput_items > MAX_NUM_SAMPLES)
      {
        num_samples = MAX_NUM_SAMPLES;
      }
      else
      {
        num_samples = noutput_items;
      }

      // Fill kernel buffer with samples from input buffer
      // Cast to 64 bit as AXI bus is 64 bits wide
      for(i = 0; i < num_samples; i++)
      {
        d_buff[i] = (long long int)in[i];
        //printf("d_buff[%2d](%p)=%lld\n",i,&d_buff[i],d_buff[i]);
      }

      // Setup control registers to read samples from kernel buffer and write
      // samples back to kernel buffer
      int num_bytes = num_samples * sizeof(long long int); //noutput_items * sizeof(long long int);
      // Set the write address
      d_control_regs[OFFSET_FIR_SAMP_WR+FIFO_WR_ADDR] = d_phys_addr + num_bytes;
      // Set the number of bytes to write
      d_control_regs[OFFSET_FIR_SAMP_WR+FIFO_WR_SIZE] = num_bytes;
      // Set the read address
      d_control_regs[OFFSET_FIR_SAMP_RD+FIFO_WR_ADDR] = d_phys_addr;
      // Set the number of bytes to read
      d_control_regs[OFFSET_FIR_SAMP_RD+FIFO_WR_SIZE] = num_bytes;
      val = read(d_fd,0,0);
      // Read all status FIFOs. If not, the status FIFOs will fill and eventually cause
      // the AXI datamover in the FPGA to ignore further requests.
      d_control_regs[OFFSET_FIR_SAMP_RD+FIFO_WR_STS_RDY] = 0;
      d_control_regs[OFFSET_FIR_SAMP_WR+FIFO_WR_STS_RDY] = 0;
      d_control_regs[OFFSET_FIR_RELOAD+FIFO_WR_STS_RDY] = 0;

      // Copy result from kernel buffer to output buffer
      for(i = 0; i < num_samples; i++)
      {
        out[i] = (int)d_buff[i+num_samples];
        //printf("d_buff[%2d](%p)=%lld\n",i+num_samples,&d_buff[i+num_samples],d_buff[i+num_samples]);
      }

      // Tell runtime system how many output items we produced.
      return num_samples;
    }

    int fir_filter_ii_impl::open_driver()
    {
      // open the file descriptor to our kernel module
      const char* dev = "/dev/user_peripheral";

      // Open with read / write access and block until write has completed
      d_fd = open(dev, O_RDWR|O_SYNC);
      if (d_fd == 0)
      {
        printf("ERROR: Failed to open %s\n",dev);
        perror("");
        return(-1);
      }

      // Get user peripheral parameters
      if (this->get_params_from_sysfs() != 0)
      {
        printf("ERROR: Failed to get sysfs parameters\n");
        close(d_fd);
        return(-1);
      }

      // mmap the control and data regions into virtual space
      d_control_regs = (unsigned int*)mmap(NULL, d_control_length, PROT_READ|PROT_WRITE, MAP_SHARED, d_fd, 0x1000);
      d_control_length = d_control_length/sizeof(unsigned int);
      if (d_control_regs == MAP_FAILED)
      {
        perror("ERROR: Failed mapping control registers");
        close(d_fd);
        return(-1);
      }

      d_buff = (long long int*)mmap(NULL, d_buffer_length, PROT_READ|PROT_WRITE, MAP_SHARED, d_fd, 0x2000);
      d_buffer_length = d_buffer_length/sizeof(long long int);
      if (d_buff == MAP_FAILED)
      {
        perror("ERROR: Failed mapping kernel buffer");
        close(d_fd);
        return(-1);
      }

      // zero out the data region
      // memset((void *)d_buff, 0, (unsigned int)(d_buffer_length));
      return(0);
    }

    int fir_filter_ii_impl::close_driver()
    {
      munmap(d_control_regs,d_control_length);
      munmap(d_buff,d_buffer_length);
      close(d_fd);
      d_control_regs = 0;
      d_buff = 0;
      d_fd = 0;
      return(0);
    }

    int fir_filter_ii_impl::read_fpga_status()
    {
      struct udev *udev;
      struct udev_enumerate *enumerate;
      struct udev_list_entry *device;
      struct udev_device *dev;
      const char *path;
      long prog_done = 0;

      udev = udev_new();
      if (!udev)
      {
        printf("ERROR: Failed udev_new()\n");
        return -1;
      }

      // Enumerate devcfg
      enumerate = udev_enumerate_new(udev);
      udev_enumerate_add_match_sysname(enumerate, "f8007000.devcfg");
      udev_enumerate_scan_devices(enumerate);
      device = udev_enumerate_get_list_entry(enumerate);

      // List should have only one entry
      if (udev_list_entry_get_next(device) != NULL)
      {
        printf("ERROR: Found more than one devcfg device!\n");
        return(-1);
      }

      // Create udev device
      path = udev_list_entry_get_name(device);
      dev = udev_device_new_from_syspath(udev, path);

      prog_done = atol(udev_device_get_sysattr_value(dev, "prog_done"));

      //printf("%s/prog_done = %ld\n", udev_device_get_syspath(dev),prog_done);

      udev_enumerate_unref(enumerate);
      udev_unref(udev);

      if (prog_done != 1) return(-1);
      return(0);
    }

    int fir_filter_ii_impl::get_params_from_sysfs()
    {
      struct udev *udev;
      struct udev_enumerate *enumerate;
      struct udev_list_entry *devices, *dev_list_entry;
      struct udev_device *dev;

      udev = udev_new();
      if (!udev)
      {
        printf("ERROR: udev failed\n");
        return(-1);
      }

      enumerate = udev_enumerate_new(udev);
      udev_enumerate_add_match_sysname(enumerate, "40000000.user_peripheral");
      udev_enumerate_scan_devices(enumerate);
      devices = udev_enumerate_get_list_entry(enumerate);

      udev_list_entry_foreach(dev_list_entry, devices)
      {
        const char *path;

        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);

        d_buffer_length = atol(udev_device_get_sysattr_value(dev, "xx_buf_len"));
        d_control_length = atol(udev_device_get_sysattr_value(dev, "regs_len"));
        d_phys_addr = atol(udev_device_get_sysattr_value(dev, "xx_phys_addr"));
      }

      udev_enumerate_unref(enumerate);
      udev_unref(udev);

      return(0);
      }

  } /* namespace zynq */
} /* namespace gr */

