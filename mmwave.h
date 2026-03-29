#ifndef MMWAVE_H_
#define MMWAVE_H_
#include <fftw3.h>
#include <stdint.h>

typedef fftwf_complex Complex;

typedef struct {
  int8_t num_tx;
  int8_t num_rx;
  int16_t num_samples;
  int16_t num_chirps;
} RadarConfig;

int get_total_frames(FILE *in, const RadarConfig *config);

int read_adc_to_frames(FILE *in, Complex *out, const RadarConfig *cfg);

#endif // MMWAVE_H_
