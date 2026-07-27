module mmwave.radar;

import :types;
import :fft;

namespace mmwave::radar {
static fftw_complex *to_fftw(FFTComplex *ptr) {
  return reinterpret_cast<fftw_complex *>(ptr);
}

FFTWPlan::FFTWPlan(fftw_plan plan) noexcept : plan_(plan) {}

FFTWPlan::FFTWPlan(FFTWPlan &&other) noexcept : plan_(other.plan_) {
  other.plan_ = nullptr;
}

void FFTWPlan::execute() noexcept {
  if (plan_)
    fftw_execute(plan_);
}

void FFTWPlan::reset() noexcept {
  if (plan_) {
    fftw_destroy_plan(plan_);
    plan_ = nullptr;
  }
}

void RadarFFT::create_range_plan(ComplexView input, ComplexView output) {
  int n[1] = {static_cast<int>(input.extent(3))};

  int howmany = input.extent(0) * input.extent(1) * input.extent(2);

  plans_[0] =
      fftw_plan_many_dft(1, n, howmany,

                         reinterpret_cast<fftw_complex *>(input.data_handle()),

                         nullptr, 1, input.extent(3),

                         reinterpret_cast<fftw_complex *>(output.data_handle()),

                         nullptr, 1, output.extent(3),

                         FORWARD, ESTIMATE);
}

void RadarFFT::create_doppler_plan(ComplexView input, ComplexView output) {
  int n[1] = {static_cast<int>(input.extent(1))};

  int howmany = input.extent(0) * input.extent(2) * input.extent(3);

  int stride = input.extent(2) * input.extent(3);

  plans_[1] =
      fftw_plan_many_dft(1, n, howmany,

                         reinterpret_cast<fftw_complex *>(input.data_handle()),

                         nullptr, stride, 1,

                         reinterpret_cast<fftw_complex *>(output.data_handle()),

                         nullptr, stride, 1,

                         FORWARD, ESTIMATE);
}

void RadarFFT::range_fft() noexcept {
  // FFTW / custom FFT implementation
  plans_[0].execute();
}

void RadarFFT::doppler_fft() noexcept {
  // Doppler FFT
  plans_[1].execute();
}

} // namespace mmwave::radar
