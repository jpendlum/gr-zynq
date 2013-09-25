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
 * Description: Program to test the performance of FPGA filter filter
 *
 ***************************************************************************/
#include <vector>
#include "fir_filter_ccc_impl.h"
#include "gnuradio/types.h"
#include <cstdio>
#include <cstdlib>
#include <ctime>

#define NUM_TAPS 31
#define NUM_SAMPLES 50000000

double get_wall_time();

int main() {
  int i = 0;
  int consumed = 0;
  int total_consumed = 0;
  int total_left = NUM_SAMPLES/2;

  std::vector<gr_complex> taps;
  for (i = 0; i < NUM_TAPS; i++) {
    taps.push_back(std::complex<float>(rand(),rand()));
  }

  // Place on heap as stack does not have enough space
  float* input_buffer = new float[NUM_SAMPLES];
  for (i = 0; i < NUM_SAMPLES; i++) {
    input_buffer[i] = (float)rand();
  }
  std::vector<const void*> input_items;
  input_items.push_back((const void *) input_buffer);

  // Place on heap as stack does not have enough space
  float* output_buffer = new float[NUM_SAMPLES];
  for (i = 1; i < NUM_SAMPLES; i++) {
    output_buffer[i] = 0.0;
  }
  std::vector<void*> output_items;
  output_items.push_back((void *) output_buffer);

  gr::filter::fir_filter_ccc_impl *filter = new gr::filter::fir_filter_ccc_impl(1, taps);

  double t0, t1, tcal;
  double total_time = 0.0;

  // Calibrate
  t0 = get_wall_time();
  tcal = get_wall_time();
  t1 = get_wall_time();
  tcal = t1 - t0;

  // Measure filtering time
  while(total_left > 0) {
    t0 = get_wall_time();
    consumed = filter->work(total_left,(gr_vector_const_void_star&) input_items,(gr_vector_void_star&) output_items);
    t1 = get_wall_time();
    total_time = total_time + (t1 - t0) - 2*tcal;
    total_consumed = total_consumed + consumed;
    total_left = total_left - consumed;
    if (total_left > 0) {
      //printf("Total left: %d \t Total consumed: %d \n",total_left,total_consumed);
      input_items.pop_back();
      input_items.push_back((const void *) &input_buffer[total_consumed]);
      output_items.pop_back();
      output_items.push_back((void *) &output_buffer[total_consumed]);
    }
  }

  printf("Time elasped (secs): %f\n Samples/sec: %f\n",total_time,NUM_SAMPLES/total_time);

  /*for (i = 0; i < NUM_TAPS; i++) {
    printf("input[%d]: %d \t\t output[%d]: %d\n",i,input_buffer[i],i,output_buffer[i]);
  } */

  delete input_buffer;
  delete output_buffer;

  return 0;
}

double get_wall_time() {
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        //  Handle error
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}