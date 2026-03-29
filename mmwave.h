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
  int16_t num_frames; // set by `read_adc_to_frames`
} RadarConfig;

int get_total_frames(FILE *in, const RadarConfig *config);

int read_adc_to_frames(FILE *in, Complex *out, RadarConfig *cfg);

void range_fft(Complex *data, const RadarConfig *config);

void doppler_fft(Complex *data, const RadarConfig *config);

void print_magnitude(const Complex *data, int count, const char *title);

#endif // MMWAVE_H_
