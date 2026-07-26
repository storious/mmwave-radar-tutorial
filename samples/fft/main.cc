
extern "C" {
#include "fftw3.h"
}

#include <print>
import mmwave.radar;

using namespace mmwave::radar;

int main() {
  constexpr int N = 8;

  auto *input =
      static_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * N));

  auto *output =
      static_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * N));

  if (!input || !output) {
    std::println("alloc failed");
    return 1;
  }

  FFTWMemPool pool;

  RadarFFT fft(&pool);

  auto plan = fft.create_1d_plan(N, input, output);

  if (!plan) {
    std::println("create fft plan failed");
    return 1;
  }

  plan.execute();

  fftw_free(input);
  fftw_free(output);
  std::println("FFT OK");
}
