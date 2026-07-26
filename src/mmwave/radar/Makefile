CXX=clang++
CXXFLAGS=-std=c++23 -stdlib=libc++

BUILD_DIR=../../../build/mmwave
MODULE_DIR=$(BUILD_DIR)/modules
OBJ_DIR=$(BUILD_DIR)/obj

MODULE_PATH=-fprebuilt-module-path=$(MODULE_DIR)

MODULE_PREFIX=mmwave.radar


TYPES_PCM=$(MODULE_DIR)/$(MODULE_PREFIX)-types.pcm
FRAME_PCM=$(MODULE_DIR)/$(MODULE_PREFIX)-frame.pcm
ADC_PCM=$(MODULE_DIR)/$(MODULE_PREFIX)-adc.pcm
FFT_PCM=$(MODULE_DIR)/$(MODULE_PREFIX)-fft.pcm
RADAR_PCM=$(MODULE_DIR)/$(MODULE_PREFIX).pcm


TYPES_OBJ=$(OBJ_DIR)/$(MODULE_PREFIX)-types.o
FRAME_OBJ=$(OBJ_DIR)/$(MODULE_PREFIX)-frame.o
ADC_OBJ=$(OBJ_DIR)/$(MODULE_PREFIX)-adc.o
FFT_OBJ=$(OBJ_DIR)/$(MODULE_PREFIX)-fft.o
FFT_INTERFACE_OBJ=$(OBJ_DIR)/mmwave.radar-fft-interface.o
RADAR_OBJ=$(OBJ_DIR)/$(MODULE_PREFIX).o


.PHONY: all clean

all: \
	$(RADAR_PCM) \
	$(RADAR_OBJ) \
	$(FRAME_OBJ) \
	$(ADC_OBJ) \
	$(FFT_OBJ) \
	$(FFT_INTERFACE_OBJ)



# ==========================
# Primary module
# ==========================

$(RADAR_PCM): mod.mpp \
	$(TYPES_PCM) \
	$(FRAME_PCM) \
	$(ADC_PCM) \
	$(FFT_PCM)

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@


$(RADAR_OBJ): mod.mpp $(RADAR_PCM)

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		-c $< \
		-o $@



# ==========================
# Partition interface
# ==========================

$(TYPES_PCM): types.mpp

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@


$(FRAME_PCM): frame.ccm \
	$(TYPES_PCM) \
	$(ADC_PCM)

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@


$(ADC_PCM): adc.ccm \
	$(FFT_PCM)

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@


$(FFT_PCM): fft.mpp \
	$(TYPES_PCM)

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@



# ==========================
# Partition implementation
# ==========================

$(FRAME_OBJ): frame.ccm $(FRAME_PCM)

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		-c $< \
		-o $@


$(ADC_OBJ): adc.ccm $(ADC_PCM)

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		-c $< \
		-o $@


$(FFT_OBJ): fft.cc $(FFT_PCM)

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-c $< \
		-o $@

$(FFT_INTERFACE_OBJ): \
    fft.mpp \
    $(MODULE_DIR)/${FFT_PCM}

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		-c fft.mpp \
		-o $@

clean:
	rm -rf $(BUILD_DIR)
