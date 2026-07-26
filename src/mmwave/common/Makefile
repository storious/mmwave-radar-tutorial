CXX=clang++
CXXFLAGS=-std=c++23 -stdlib=libc++

BUILD_DIR=../../../build/mmwave
MODULE_DIR=$(BUILD_DIR)/modules
OBJ_DIR=$(BUILD_DIR)/obj

MODULE_PATH=-fprebuilt-module-path=$(MODULE_DIR)

# NOTE
MODULE=mmwave.common

.PHONY: all

all: $(MODULE_DIR)/${MODULE}.pcm


$(MODULE_DIR)/${MODULE}.pcm: mod.mpp \
    $(MODULE_DIR)/${MODULE}-error.pcm

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile mod.mpp \
		-o $@


$(OBJ_DIR)/${MODULE}-%.o: %.cc $(MODULE_DIR)/${MODULE}-%.pcm

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-c $< \
		-o $@


$(OBJ_DIR)/${MODULE}-%.o: %.ccm $(MODULE_DIR)/${MODULE}-%.pcm

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-c $< \
		-o $@


$(MODULE_DIR)/${MODULE}-%.pcm: %.mpp

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@


$(MODULE_DIR)/${MODULE}-%.pcm: %.ccm

	@mkdir -p $(MODULE_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@


