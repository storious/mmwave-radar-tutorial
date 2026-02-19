build:
	clang++-20 -std=c++23 -stdlib=libc++ -fmodule-file=std=std.pcm -o adc main.cc std.pcm

clean:
	rm -f *.out
