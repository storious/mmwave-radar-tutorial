module mmwave.radar;

import :types;
import :fft;

namespace mmwave::radar {

FFTWPlan::FFTWPlan(fftw_plan plan) noexcept : plan_(plan) {}

FFTWPlan RadarFFT::create_1d_plan(int n, fftw_complex *in, fftw_complex *out) {
  auto plan = fftw_plan_dft_1d(n, in, out, FORWARD, ESTIMATE);

  return FFTWPlan(plan);
}

FrameView RadarFFT::run_1d(ConstFrameView frame) noexcept {
  // FFTW / custom FFT implementation
  FrameView outview;
  return outview;
}

FrameView RadarFFT::run_2d(ConstFrameView frame) noexcept {
  // Doppler FFT
  FrameView outview;
  return outview;
}

} // namespace mmwave::radar
