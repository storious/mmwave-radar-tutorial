#include "mmwave.h"
#include <stdio.h> // 需要包含以使用 printf 和 fopen_s
#include <stdlib.h>

int main() {
  FILE *fp = NULL;

  errno_t err = fopen_s(&fp, "adc_data.bin", "rb");

  if (err != 0 || fp == NULL) {
    printf("Failed to open file. Error code: %d\n", err);
    return -1;
  }

  RadarConfig config = {.num_rx = 4, .num_samples = 128, .num_chirps = 2};

  // 1. 预估帧数以分配内存
  int max_frames = get_total_frames(fp, &config);
  printf("Estimated frames: %d\n", max_frames);

  // 2. 分配 FFTW 内存
  size_t total_samples = (size_t)max_frames * config.num_chirps *
                         config.num_rx * config.num_samples;
  fftwf_complex *data =
      (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * total_samples);

  // 3. 读取数据
  int actual_frames = read_adc_to_frames(fp, data, &config);
  printf("Successfully read %d frames.\n", actual_frames);

  // 4. 打印第一帧的前五个采样点
  // 数据布局: [Frame][Chirp][Rx][Sample]
  // 我们打印 Frame 0, Chirp 0, Rx 0 的前 5 个采样点
  if (actual_frames > 0) {
    printf("\n--- First Frame Data (First 5 Samples of Chirp 0, Rx 0) ---\n");
    for (int i = 0; i < 5; ++i) {
      // 索引计算：
      // Frame=0, Chirp=0, Rx=0, Sample=i
      // 此时数据在内存中是连续的，直接使用 data[i] 即可
      float real = data[i][0];
      float imag = data[i][1];
      printf("Sample[%d]: %.4f + %.4fi\n", i, real, imag);
    }
    printf("-------------------------------------------------------------\n");
  }

  // 5. 后续处理 (如 Range FFT)
  // ... process(data, actual_frames) ...

  fclose(fp);
  fftwf_free(data);
  return 0;
}
