CXX=clang++
CXXFLAGS=-std=c++23 -stdlib=libc++

BUILD_DIR=../../../build/mmwave
MODULE_DIR=$(BUILD_DIR)/modules
OBJ_DIR=$(BUILD_DIR)/obj

MODULE_PATH=-fprebuilt-module-path=$(MODULE_DIR)

# NOTE
MODULE_PREFIX=mmwave.radar

.PHONY: all

all: $(MODULE_DIR)/${MODULE_PREFIX}.pcm \
			$(OBJ_DIR)/${MODULE_PREFIX}-frame.o \
    	$(OBJ_DIR)/${MODULE_PREFIX}-adc.o \
			$(OBJ_DIR)/${MODULE_PREFIX}-fft.o



$(MODULE_DIR)/${MODULE_PREFIX}.pcm: mod.mpp \
    $(MODULE_DIR)/${MODULE_PREFIX}-frame.pcm \

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile mod.mpp \
		-o $@


$(MODULE_DIR)/${MODULE_PREFIX}-frame.pcm: frame.ccm \
    $(MODULE_DIR)/${MODULE_PREFIX}-types.pcm \
    $(MODULE_DIR)/${MODULE_PREFIX}-adc.pcm \

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile frame.ccm \
		-o $@


$(MODULE_DIR)/${MODULE_PREFIX}-adc.pcm: adc.ccm \
		$(MODULE_DIR)/${MODULE_PREFIX}-fft.pcm \

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile adc.ccm \
		-o $@


$(OBJ_DIR)/${MODULE_PREFIX}-%.o: %.cc $(MODULE_DIR)/${MODULE_PREFIX}-%.pcm

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-c $< \
		-o $@


$(OBJ_DIR)/${MODULE_PREFIX}-%.o: %.ccm $(MODULE_DIR)/${MODULE_PREFIX}-%.pcm

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-c $< \
		-o $@


$(MODULE_DIR)/${MODULE_PREFIX}-%.pcm: %.mpp

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@


$(MODULE_DIR)/${MODULE_PREFIX}-%.pcm: %.ccm

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@

