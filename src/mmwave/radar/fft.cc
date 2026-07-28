module;
#include <fftw3.h>
module mmwave.radar;

import :types;
import :fft;

namespace mmwave::radar {
static fftwf_complex *to_fftwf(FFTComplex *ptr) {
  return reinterpret_cast<fftwf_complex *>(ptr);
}

FFTWPlan::FFTWPlan(fftwf_plan plan) noexcept : plan_(plan) {}
FFTWPlan::FFTWPlan(FFTWPlan &&other) noexcept : plan_(other.plan_) {
  other.plan_ = nullptr;
}

FFTWPlan::~FFTWPlan() {
  if (plan_)
    fftwf_destroy_plan(plan_);
}

void FFTWPlan::reset() noexcept {
  if (plan_) {
    fftwf_destroy_plan(plan_);
    plan_ = nullptr;
  }
}

FFTWPlan &FFTWPlan::operator=(FFTWPlan &&other) noexcept {
  if (this != &other) {
    reset();

    plan_ = other.plan_;

    other.plan_ = nullptr;
  }

  return *this;
}

void FFTWPlan::execute() noexcept { fftwf_execute(plan_); }

RadarFFT::RadarFFT(ComplexView input, ComplexView output) {
  create_range_plan(input, output);
  create_doppler_plan(output);
}

void RadarFFT::create_range_plan(ComplexView input, ComplexView output) {

  fftwf_iodim dim;

  dim.n = static_cast<int>(input.extent(3));

  dim.is = 1;
  dim.os = 1;

  fftwf_iodim howmany;

  howmany.n =
      static_cast<int>(input.extent(0) * input.extent(1) * input.extent(2));

  howmany.is = static_cast<int>(input.extent(3));

  howmany.os = static_cast<int>(output.extent(3));

  auto plan = fftwf_plan_guru_dft(
      1, &dim,

      1, &howmany,

      reinterpret_cast<fftwf_complex *>(input.data_handle()),

      reinterpret_cast<fftwf_complex *>(output.data_handle()),

      FFTW_FORWARD, FFTW_ESTIMATE);

  plans_[0] = FFTWPlan(plan);
}

void RadarFFT::create_doppler_plan(ComplexView range) {

  fftwf_iodim dim;

  // chirp axis
  dim.n = static_cast<int>(range.extent(1));
  dim.is = static_cast<int>(range.stride(1));
  dim.os = dim.is;

  fftwf_iodim howmany[3];

  // range
  howmany[0].n = static_cast<int>(range.extent(3));
  howmany[0].is = static_cast<int>(range.stride(3));
  howmany[0].os = howmany[0].is;

  // rx
  howmany[1].n = static_cast<int>(range.extent(2));
  howmany[1].is = static_cast<int>(range.stride(2));
  howmany[1].os = howmany[1].is;

  // frame
  howmany[2].n = static_cast<int>(range.extent(0));
  howmany[2].is = static_cast<int>(range.stride(0));
  howmany[2].os = howmany[2].is;

  auto plan = fftwf_plan_guru_dft(
      1, &dim,

      3, howmany,

      reinterpret_cast<fftwf_complex *>(range.data_handle()),

      reinterpret_cast<fftwf_complex *>(range.data_handle()),

      FFTW_FORWARD, FFTW_ESTIMATE);

  plans_[1] = FFTWPlan(plan);
}

void RadarFFT::range_fft() { plans_[0].execute(); }

void RadarFFT::doppler_fft() { plans_[1].execute(); }

} // namespace mmwave::radar
