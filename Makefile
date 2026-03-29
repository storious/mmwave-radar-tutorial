FFTW_INCLUDE_DIR="D:/Shared/FFTW/"
FFTW_LIBRARY="D:/Shared/FFTW/libfftw3f-3.lib" 
CC = clang
CFLAGS = -g -std=c23 -Wall -I$(FFTW_INCLUDE_DIR)
LDFLAGS = $(FFTW_LIBRARY)


GTRACK_OBJS = \
	gtrack_preprocess.o \
	gtrack_cluster.o \
	gtrack_predict.o \
	gtrack_associate.o \
	gtrack_update.o \
	gtrack_processFrame.o

%.o: %.c utest.h
	$(CC) $(CFLAGS) -c $< -o $@

# ==================== tests ====================
test_gtrack: gtrack_test.o $(GTRACK_OBJS)
	$(CC) $^ $(LDFLAGS) -o test_gtrack
	./test_gtrack

%_test: %_test.o
	$(CC) $< -o $@
	@./$@

# ==================== main ====================
run: main.o mmwave.o
	$(CC) $^ $(LDFLAGS) -o main
	./main

# ==================== clean up ====================
clean:
	rm -f *.o test_gtrack test_ptr test_utest test_arena main
