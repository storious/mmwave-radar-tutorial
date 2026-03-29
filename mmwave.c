#include "mmwave.h"
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

int read_adc_to_frames(FILE *in, Complex *out, const RadarConfig *config) {
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

      float i1 = (float)raw_buffer[i], i2 = (float)raw_buffer[i + 1];
      float q1 = (float)raw_buffer[i + 2], q2 = (float)raw_buffer[i + 3];

      frame_ptr[samples_idx][0] = i1;
      frame_ptr[samples_idx++][1] = q1;

      frame_ptr[samples_idx][0] = i2;
      frame_ptr[samples_idx++][1] = q2;
    }

    frames_read++;
  }

  free(raw_buffer);
  return frames_read;
}
