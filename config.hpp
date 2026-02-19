#ifndef CONFIG_H_
#define CONFIG_H_
#include <string>

// ==========================================
// Config
// ==========================================
const std::string BinPath = "adc_data.bin";
const int NumADCSamples = 128;
const int NumTxAntennas = 1;
const int NumRxAntennas = 4;
const int NumChirpsLoop = 2;
const double periodicity = 0.01; // seconds

constexpr int ChirpsPerFrame = NumChirpsLoop * NumTxAntennas;
constexpr std::size_t FPS = 10;

// Chirps * Rx * Samples * 2(IQ) * 2(bytes/int16)
constexpr std::size_t BytesPerFrame =
    static_cast<size_t>(ChirpsPerFrame) * NumRxAntennas * NumADCSamples * 4;

#endif // CONFIG_H_
