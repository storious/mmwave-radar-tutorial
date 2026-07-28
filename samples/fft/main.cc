#include <print>
#include <vector>

import mmwave.radar;

using namespace mmwave::radar;

auto main() -> int {
  std::string filepath = "adc_data.bin";
  FFTWMemPool pool;

  FrameReader<> reader(&pool);
  reader.add_files({filepath});

  if (!reader.open()) {
    std::println("open failed");
    return 1;
  }

  auto res = reader.read_complex_frames();
  if (!res) {
    std::println("read failed");
    return 1;
  }

  auto &[cube, count] = *res;

  std::println("frames count = {}, cube size = {}", count, cube.size());

  auto in_view = FrameProcessor::make_complex_view(cube, count);
  auto shape = in_view.extents();

  std::println("cube shape: {} {} {} {}", shape.extent(0), shape.extent(1),
               shape.extent(2), shape.extent(3));

  FFTFrame out(&pool);
  out.resize(cube.size());
  std::println("out size = {}", out.size());
  auto out_view = ComplexView(out.data(), shape);
  std::println("in view size = {}, out view size = {}", in_view.size(),
               out_view.size());
  RadarFFT fft(in_view, out_view);
  fft.range_fft();

  std::println("FFT OK");
}
