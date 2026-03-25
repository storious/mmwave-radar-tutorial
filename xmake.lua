add_rules("mode.debug", "mode.release")
set_languages("c++23")
set_toolchains("clang")

add_cxxflags("-std=c++23", "-stdlib=libc++")
add_ldflags("-std=c++23", "-stdlib=libc++", "-lc++abi")

set_runtimes("c++_static")
add_defines("ROOT")

includes("src", "samples")
