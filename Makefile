
run:adc.cc
	@clang++ -std=c++23 -stdlib=libc++ -fprebuilt-module-path=. main.cc -o adc.out

init:std.pcm
	@clang++ -std=c++23 -stdlib=libc++ /usr/share/libc++/v1/std.cppm --precompile -o std.pcm

%.out:%.cc
	@clang++ -std=c++23 -stdlib=libc++ $< -o $@ -fprebuilt-module-path=.  

%.pcm:%.ccm
	@clang++ -std=c++23 -stdlib=libc++ -fprebuilt-module-path=. --precompile $< -o $@ 

clean:
	@rm -f *.out
