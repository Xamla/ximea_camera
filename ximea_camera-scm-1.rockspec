package = "ximea_camera"
version = "scm-1"

source = {
  url = "git://github.com/Xamla/ximea_camera"
}

description = {
  summary = "Ximea Camera Driver",
  detailed = [[
  ]],
  homepage = "http://www.xamla.com/egomo/",
  license = "GPLv3"
}

dependencies = {
"torch >= 7.0"
}

build = {
type = "command",
  build_command = [[
  cmake -E make_directory build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$(PREFIX)" && $(MAKE) -j4
  ]],
    install_command = "cd build && $(MAKE) install"
}
