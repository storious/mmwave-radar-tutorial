module;
#include <cassert>
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

void FFTWPlan::execute() noexcept {
  assert(plan_);
  fftwf_execute(plan_);
}

RadarFFT::RadarFFT(ComplexView input, ComplexView output) {
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
  assert(plans_[0]);
}

void RadarFFT::range_fft() { plans_[0].execute(); }

void RadarFFT::doppler_fft(ComplexView input, ComplexView output) {}
} // namespace mmwave::radar
