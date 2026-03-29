# [中文](./README.md) | English 
# mmWave Radar Learning Repository
Since I have never been exposed to mmWave radar before and am still learning, I found that relevant open-source materials are too scattered, and most of the code is outdated, making many papers difficult to reproduce. Therefore, I created this repository to learn mmWave radar data processing and related algorithms from scratch.

If you find any errors in this repository or have suggestions, feel free to raise them in the `issues`.

For more details, see the `tutorial` directory.
# Experimental Conditions

Data Acquisition: TI IWR6843ISK + DCA1000EVM (ADC data format is LVDS Lane 2)
> TI xWR18xx, xWR16xx, xWR68xx series all follow the same format, so they can all be used. Refer to [Mmwave Radar Device ADC Raw Data Capture](https://www.ti.com.cn/lit/an/swra581b/swra581b.pdf?ts=1774154882296&ref_url=https%253A%252F%252Fcn.bing.com%252F)

Runtime Environment:
 - [clang](https://clang.llvm.org/): A compiler supporting the C23 standard is required. Do not use MSVC.
 - [fftw](https://www.fftw.org/download.html): High-performance FFT computation library.
 - [make](https://www.gnu.org/software/make/)

# Quick Start
1. Please ensure you have installed a C compiler supporting the C23 standard and correctly configured `fftw`.
2. Modify the variables `CC`, `FFTW_INCLUDE`, and `FFTW_LIBRARY` in the Makefile to point to the correct compiler and `fftw` paths.
3. Execute in the project root directory:

```shell
make run
```
If you see output similar to the following, it indicates the environment is configured correctly:

```text
Estimated frames: 5000
Successfully read 5000 frames.
--- First Frame Data (First 5 Samples of Chirp 0, Rx 0) ---
Sample[0]: -213.0000 + -31.0000i
Sample[1]: -23.0000 + -31.0000i
Sample[2]: -167.0000 + -116.0000i
Sample[3]: 14.0000 + -4.0000i
Sample[4]: -121.0000 + -156.0000i
-------------------------------------------------------------
```
# Project Layout
[cmini](https://github.com/storious/cmini): This comes from another C utility library I created for fun.

gtrack: (To be implemented) TI Gtrack target tracking algorithm.

tutorial: Learning notes documentation.

# References
- [OpenRadar](https://github.com/PreSenseRadar/OpenRadar.git)

- [TI doc](https://dev.ti.com/tirex/explore/node?isTheia=false&node=A__Af9Z649Xr4Fm41j.zJ.nZg__radar_toolbox__1AslXXD__LATEST)
