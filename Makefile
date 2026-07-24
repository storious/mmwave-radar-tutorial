CXX=clang++
CXXFLAGS=-std=c++23 -stdlib=libc++
MODULE_PATH=-fprebuilt-module-path=.


MODULES=$(patsubst %.mpp,%.pcm,$(wildcard *.mpp)) \
        $(patsubst %.ccm,%.pcm,$(wildcard *.ccm))


OBJECTS=$(patsubst %.ccm,%.o,$(wildcard *.ccm))

.PHONY: build
build:
	${MAKE} -C ./src/ 

run: std.pcm $(MODULES) $(OBJECTS)
	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		main.cc \
		$(OBJECTS) \
		-o adc.out


%.pcm: %.ccm
	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@


%.pcm: %.mpp
	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-x c++-module \
		--precompile $< \
		-o $@


%.o: %.ccm %.pcm
	$(CXX) $(CXXFLAGS) \
		$(MODULE_PATH) \
		-c $< \
		-o $@


std.pcm:
	$(CXX) $(CXXFLAGS) \
		-x c++-module \
		--precompile \
		/usr/share/libc++/v1/std.cppm \
		-o std.pcm


clean:
	rm -f *.out *.pcm *.o
