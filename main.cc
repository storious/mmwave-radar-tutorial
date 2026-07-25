import adc;
import config;
import std;

using namespace mmwave;

auto main() -> int {

  FrameReader reader;

  reader.add_file(config::BinPath);

  auto open_result = reader.open();

  if (!open_result) {
    std::println("Failed to open file");
    return 1;
  }

  auto frame_result = reader.read_frame();

  if (not frame_result) {
    std::println("Failed to read frame");
    return 1;
  }

  ADCFrame frame = std::move(*frame_result);

  auto view_result = FrameProcessor::make_view(frame);

  if (not view_result) {
    std::println("Invalid frame size");
    return 1;
  }

  auto view = *view_result;

  FrameProcessor::print_info(view);

  std::println("Sample data (Chirp 0, Rx 0, Samples 0-4):");

  for (std::size_t i : std::views::iota(0, 5)) {
    auto value = view[0, 0, i];

    std::println("sample {}: {}+{}j", i, value.real(), value.imag());
  }

  return 0;
}
