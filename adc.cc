#include <complex>
#include <cstdint>
#include <vector>

#include <mdspan> // for reshape raw adc data to LVDS style

#include "config.hpp"
// ==========================================
// Type defination
// ==========================================

using ComplexI16 = std::complex<int16_t>;
using ADCData = std::vector<ComplexI16>;
using LVDS =
    std::mdspan<ComplexI16, std::extents<std::size_t, ChirpsPerFrame,
                                         NumRx, NumADCSamples>>;

// ==========================================
// Core logical implementation
// ==========================================

/**
 * @brief transform raw data to adc data
 *
 * TI mmWave Studio default LVDS format (complex)。
 *
 * @param raw_data int16
 * @return ADCData IQ complex signal
 */
ADCData organize_adc(const std::vector<int16_t> &raw_data) {
  size_t data_idx = 0;
  std::vector<ComplexI16> raw_data_frame(raw_data.size() / 2);

  for (int i = 0; i + 3 < raw_data.size(); i += 4) {
    raw_data_frame[data_idx++] = ComplexI16(raw_data[i], raw_data[i + 2]);
    raw_data_frame[data_idx++] = ComplexI16(raw_data[i + 1], raw_data[i + 3]);
  }

  return raw_data_frame;
}

class ADCReader {
public:
  struct Config {
    std::size_t frame_size_bytes_;
    std::size_t buffer_capacity_;
    int timeout_ms_;

    Config()
        : frame_size_bytes_(BytesPerFrame), buffer_capacity_(FPS),
          timeout_ms_(100) {}
  };

private:
};
