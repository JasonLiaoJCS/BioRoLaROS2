#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
out=${1:?Usage: build.sh output-directory}
mkdir -p "$out"
# All three production translation units are compiled. NI and gRPC are mocked.
# vendor curses headers were read from sbRIO; use local libcurses runtime.
g++ -std=c++14 -O1 -g -pthread \
    -I"$root/test/fake" -I"$root/include" -I"$root/test/vendor" -I/usr/include/yaml-cpp \
    -DCONFIG_PATH="\"$root/test/config.yaml\"" \
    -DFPGA_CONSOLE_LOG="\"$out/console.log\"" \
    "$root/src/console.cpp" "$root/src/fpga_server.cpp" "$root/src/fpga_handler.cpp" \
    "$root/test/fake_ni.cpp" -Wl,--wrap=wgetch -Wl,--wrap=poll -l:libncurses.so.6 -l:libtinfo.so.6 -lyaml-cpp -o "$out/fake_driver"
