#include "mmwave.h"
#include <math.h>
#include <stdlib.h>

#ifndef NDEBUG
#define LOG_WARN(fmt, ...) fprintf(stderr, "[WARN] " fmt, ##__VA_ARGS__)
#else
#define LOG_WARN(fmt, ...) ((void)0)
#endif

int get_total_frames(FILE *in, const RadarConfig *config) {
  long current_pos = ftell(in);
  fseek(in, 0, SEEK_END);
  long file_size = ftell(in);
  fseek(in, current_pos, SEEK_SET);

  if (file_size <= 0)
    return 0;

  size_t frame_size = (size_t)config->num_chirps * config->num_samples *
                      config->num_rx * 2 * sizeof(int16_t);

  return (int)(file_size / frame_size);
}

/**
 * @brief Read ADC transfomr to fftwf_complex
 *
 * @param in file pointer
 * @param out buffer (fftwf_complex type)
 * @param config radar Config
 * @return size_t frames
 */

int read_adc_to_frames(FILE *in, Complex *out, RadarConfig *config) {
  if (!in || !out || !config)
    return -1;

  const int num_rx = config->num_rx;
  const int num_samples = config->num_samples;
  const int num_chirps = config->num_chirps;

  // I1 I2 Q1 Q2
  const size_t raw_samples_per_frame =
      (size_t)num_chirps * num_samples * num_rx * 2;

  int16_t *raw_buffer =
      (int16_t *)malloc(raw_samples_per_frame * sizeof(int16_t));
  if (!raw_buffer) {
    perror("Failed to allocate raw buffer");
    return -1;
  }

  // [Frame][Chirp][Rx][Sample]
  const size_t stride_rx = num_samples;
  const size_t stride_chirp = num_rx * stride_rx;
  const size_t stride_frame = num_chirps * stride_chirp;

  size_t frames_read = 0;

  while (true) {
    size_t read_count =
        fread(raw_buffer, sizeof(int16_t), raw_samples_per_frame, in);

    if (read_count < raw_samples_per_frame) {
      if (feof(in)) {
        if (read_count > 0) {
          LOG_WARN("Incomplete frame at EOF. Expected %zu samples, got %zu. "
                   "Data discarded.\n",
                   raw_samples_per_frame, read_count);
        }
      } else {
        perror("File read error");
      }
      break;
    }

    Complex *frame_ptr = out + (frames_read * stride_frame);
    size_t samples_idx = 0;

    for (int i = 0; i + 3 < raw_samples_per_frame; i += 4) {

      // float i1 = (float)raw_buffer[i], i2 = (float)raw_buffer[i + 1];
      // float q1 = (float)raw_buffer[i + 2], q2 = (float)raw_buffer[i + 3];

      frame_ptr[samples_idx][0] = raw_buffer[i];       // I1
      frame_ptr[samples_idx++][1] = raw_buffer[i + 2]; // Q1

      frame_ptr[samples_idx][0] = raw_buffer[i + 1];   // I2
      frame_ptr[samples_idx++][1] = raw_buffer[i + 3]; // Q2
    }

    frames_read++;
  }

  config->num_frames = frames_read;
  free(raw_buffer);
  return frames_read;
}

/**
 * @brief Range FFT With FFTW
 *
 * @param data file pointer
 * @param config radar Config
 */

void range_fft(Complex *data, const RadarConfig *config) {
  int n = config->num_samples;
  int howmany = config->num_frames * config->num_chirps * config->num_rx;

  // FFTW Advance Interface
  // 1, &n      : 1D, data length is n
  // howmany    : batch process rounds
  // in, NULL, 1, n : input data ptr。Sample is continues in [frames, chirps,
  // rx, samples], so stride=1, out, NULL, 1, n :  (In-place，same as input)
  fftwf_plan plan =
      fftwf_plan_many_dft(1, &n, howmany, data, nullptr, 1, n, data, nullptr, 1,
                          n, FFTW_FORWARD, FFTW_ESTIMATE);
  if (plan) {
    fftwf_execute(plan);
    fftwf_destroy_plan(plan);
    printf("Range FFT completed.\n");
  } else {
    LOG_WARN("Failed to create Range FFT plan.\n");
  }
}

void doppler_fft(Complex *data, const RadarConfig *config) {
  int n = config->num_samples;
  int howmany = config->num_frames * config->num_chirps * config->num_rx;

  // memory layout: [Frame][Chirp][Rx][Sample]
  // index: idx = f*(C*R*S) + c*(R*S) + r*(S) + s
  // along the 'c' (Chirp) dimension。

  // Stride:
  // while c + 1，index + (num_rx * num_samples)
  int stride = config->num_rx * config->num_samples;

  // Dist:
  // Distance of the next FFT sequence
  // fixed f, r, s, range c
  // next is s+1
  //
  int dist = 1;

  fftwf_plan plan =
      fftwf_plan_many_dft(1, &n, howmany, data, nullptr, stride, dist, data,
                          nullptr, stride, dist, FFTW_FORWARD, FFTW_ESTIMATE);
  if (plan) {
    fftwf_execute(plan);
    fftwf_destroy_plan(plan);
    printf("Doppler FFT completed.\n");
  } else {
    LOG_WARN("Failed to create Doppler FFT plan.\n");
  }
}

void print_magnitude(const Complex *data, int count, const char *title) {
  printf("\n--- %s ---\n", title);
  for (int i = 0; i < count; ++i) {
    float real = data[i][0];
    float imag = data[i][1];
    float mag = sqrt(real * real + imag * imag);
    printf("Index[%d]: Mag = %.4f\n", i, mag);
  }
  printf("-------------------------------\n");
}
