CXX=clang++
CXXFLAGS=-std=c++23 -stdlib=libc++

BUILD_DIR=../build/mmwave
MODULE_DIR=$(BUILD_DIR)/modules
OBJ_DIR=$(BUILD_DIR)/obj
BIN_DIR=$(BUILD_DIR)/bin

MODULE_PATH=-fprebuilt-module-path=$(MODULE_DIR)


RADAR_OBJS= \
	$(OBJ_DIR)/mmwave.radar.o \
	$(OBJ_DIR)/mmwave.radar-frame.o \
	$(OBJ_DIR)/mmwave.radar-adc.o \
	$(OBJ_DIR)/mmwave.radar-fft.o \
	$(OBJ_DIR)/mmwave.radar-fft-interface.o


.PHONY: all

all: $(BIN_DIR)/demo.out $(BIN_DIR)/fft.out


$(BIN_DIR)/%.out: %/main.o
	@mkdir -p $(BIN_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		$(RADAR_OBJS) \
		$< \
		-lfftw3f \
		-o $@


%/main.o: %/main.cc

	@mkdir -p $(@D)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-c $< \
		-o $@
