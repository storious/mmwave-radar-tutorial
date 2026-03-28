target("mmwave")
  set_kind("static")
  add_files("mmwave/common/*.ccm", "mmwave/radar/*.ccm", { public = true })
  add_cxxflags("--precompile")

