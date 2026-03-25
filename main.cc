#include "adc.cc"

#include <fstream>
#include <iostream>

auto main(int argc, char *argv[]) -> int {
  // 1. open adc file (*.bin)
  std::ifstream file(BinPath, std::ios::binary | std::ios::ate);
  if (!file) {
    std::cerr << "Error: can't open file" << BinPath << std::endl;
    return 1;
  }

  // 2. get file size
  // TODO: add spatial read file
  std::streamsize total_bytes = file.tellg();
  file.seekg(0, std::ios::beg);

  size_t bytes_per_frame = BytesPerFrame;

  // 3. compute number of frams
  size_t num_frames =
      total_bytes / static_cast<std::streamsize>(bytes_per_frame);

  if (num_frames == 0) {
    std::cerr << "Error: File size (" << total_bytes << " bytes)，"
              << "not enough data (" << bytes_per_frame << " bytes)"
              << std::endl;
    return 1;
  }

  std::cout << "Frames" << num_frames << std::endl;

  std::cout << "Read and process first frame" << std::endl;

  // bytes_per_frame / 2 = number of int16s
  std::vector<int16_t> raw_frame_int16(bytes_per_frame / 2);

  if (!file.read(reinterpret_cast<char *>(raw_frame_int16.data()),
                 bytes_per_frame)) {
    std::cerr << "Error: read first frame" << std::endl;
    return 1;
  }

  ADCData adc_data = organize_adc(raw_frame_int16);
  LVDS frames(adc_data.data());

  std::cout << "data dimension: [Chirps=" << ChirpsPerFrame
            << ", Rx=" << NumRx << ", Samples=" << NumADCSamples << "]"
            << std::endl;

  std::cout << "\nSample data (Chirp 0, Rx 0, Samples 0-4):" << std::endl;
  for (int i = 0; i < 5 && i < NumADCSamples; ++i) {
    ComplexI16 val = frames[0, 0, i];
    std::cout << "  Sample " << i << ": " << val << std::endl;
  }
  return 0;
}
