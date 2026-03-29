// ==============================
//  mmwave-radar-tutorial: ch1 Range-Doppler map (RDMap)
// ===============================
#include "../../mmwave.h"
#include <stdlib.h>

void read_file(const char *filename, RadarConfig *config);

int main(void) {
  // 1. read raw adc data and organize to frames
  RadarConfig config = {
      .num_chirps = 2, .num_rx = 4, .num_samples = 128, .num_tx = 1};
  read_file("adc_data.bin", &config);
  return EXIT_SUCCESS;
}

void read_file(const char *filename, RadarConfig *config) {}
