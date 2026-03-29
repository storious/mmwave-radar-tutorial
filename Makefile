# !!!Import: replace the below FFTW path with your path

FFTW_INCLUDE="D:/Shared/FFTW/"
FFTW_LIBRARY="D:/Shared/FFTW/libfftw3f-3.lib" 

CC = clang
CFLAGS = -g -std=c23 -Wall -I$(FFTW_INCLUDE)
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

ch0: tutorial/ch0/main.o mmwave.o
	$(CC) $^ $(CFLAGS) $(LDFLAGS) -o main.out
	@./main.out

test_gtrack: gtrack_test.o $(GTRACK_OBJS)
	$(CC) $^ $(LDFLAGS) -o test_gtrack
	./test_gtrack

%_test: %_test.o
	$(CC) $< -o $@
	@./$@

# ==================== main ====================
run: main.o mmwave.o
	$(CC) $^ $(CFLAGS) $(LDFLAGS) -o main.out
	@./main.out

# ==================== clean up ====================
clean:
	rm -f *.o *.out *.pdb
