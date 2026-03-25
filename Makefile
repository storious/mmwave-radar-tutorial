
run:adc.cc
	@clang++ -std=c++23 -stdlib=libc++ -fprebuilt-module-path=. main.cc -o adc.out

test:
	clang++ -v -o build/linux/x86_64/release/demo build/.objs/demo/linux/x86_64/release/samples/demo.cc.o build/.objs/mmwave/linux/x86_64/release/usr/share/libc++/v1/std.cppm.o -m64 -stdlib=libc++ -static-libstdc++ -Lbuild/linux/x86_64/release -s -lmmwave

init:std.pcm
	@clang++ -std=c++23 -stdlib=libc++ /usr/share/libc++/v1/std.cppm --precompile -o std.pcm

%.out:%.cc
	@clang++ -std=c++23 -stdlib=libc++ $< -o $@ -fprebuilt-module-path=.  

%.pcm:%.ccm
	@clang++ -std=c++23 -stdlib=libc++ -fprebuilt-module-path=. --precompile $< -o $@ 

clean:
	@rm -f *.out
