#include <print>
import std;
import mmwave.radar.frame_processor;
import mmwave.radar.types;

using namespace mmwave::radar;

auto main(int argc, char **argv) -> int {
  std::string filepath = "adc_data.bin";
  if (argc > 1) {
    std::println("use specific path {}", argv[1]);
    filepath = argv[1];
  }

  FrameReader<> reader;
  reader.add_files({filepath});

  if (!reader.open()) {
    std::println("open failed");
    return 1;
  }

  auto frame = reader.read_frame();
  if (!frame) {
    std::println("read failed");
    return 1;
  }

  auto view = FrameProcessor::make_const_view(*frame);
  if (!view) {
    std::println("view failed");
    return 1;
  }

  FrameProcessor::print_info(*view);
  std::println("done.");
  return 0;
}
