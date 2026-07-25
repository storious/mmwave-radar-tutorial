CXX=clang++
CXXFLAGS=-std=c++23 -stdlib=libc++

BUILD_DIR=../build/mmwave
MODULE_DIR=$(BUILD_DIR)/modules
OBJ_DIR=$(BUILD_DIR)/obj
BIN_DIR=$(BUILD_DIR)/bin

MODULE_PATH=-fprebuilt-module-path=$(MODULE_DIR)

# NOTE

.PHONY: all

all: ${BIN_DIR}/demo.out


${BIN_DIR}/demo.out: $(OBJ_DIR)/demo.o

	@mkdir -p $(BIN_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		$(OBJ_DIR)/*.o \
		-o $@


$(OBJ_DIR)/%.o: %.cc

	@mkdir -p $(OBJ_DIR)

	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-c $< \
		-o $@

