#include <print>
#include <vector>

import mmwave.radar;

using namespace mmwave::radar;

auto main() -> int {
  std::string filepath = "adc_data.bin";

  FrameReader<> reader;
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

  auto [cube, count] = *res;

  std::println("frames size = {}", count);

  auto in_view = FrameProcessor::make_complex_view(cube, count);
  auto shape = in_view.extents();

  std::println("{} {} {} {}", shape.extent(0), shape.extent(1), shape.extent(2),
               shape.extent(3));

  std::vector<FFTComplex> out(cube.size());

  auto out_view = ComplexView(out.data(), shape.extent(0), shape.extent(1),
                              shape.extent(2), shape.extent(3));
  RadarFFT fft;
  fft.create_range_plan(in_view, out_view);

  std::println("FFT OK");
}
