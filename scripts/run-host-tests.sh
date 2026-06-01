#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/.."
mkdir -p build

if command -v g++ >/dev/null 2>&1; then
  CXX="$(command -v g++)"
elif command -v clang++ >/dev/null 2>&1; then
  CXX="$(command -v clang++)"
else
  echo "No supported C++ compiler found. Install g++ or clang++."
  echo "Then run: sh scripts/run-host-tests.sh"
  exit 2
fi

echo "Using $CXX"
"$CXX" -std=c++11 -Wall -Wextra -Werror -Isrc test/AutoRunnerTests.cpp src/AutoRunner.cpp -o build/AutoRunnerTests
./build/AutoRunnerTests
