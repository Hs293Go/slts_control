#!/bin/sh

SCRIPTPATH=$(dirname "$0")

if command -v clang-format >/dev/null 2>&1; then
  find . -path './build' -prune -o -type f -regex '.*\.\(c\|cpp\|h\|hpp\)' -exec \
    clang-format -i -style=file {} \;
else
  echo 'clang-format is not installed. Will not format C/C++ source files' >&2
fi

if command -v cmake-format >/dev/null 2>&1; then
  find . -path './build' -prune -o -type f -name 'CMakeLists.txt' -exec \
    cmake-format -i -c "$SCRIPTPATH/.cmake-format" {} \;
else
  echo 'cmake-format is not installed. Will not format CMake listfiles' >&2
fi
