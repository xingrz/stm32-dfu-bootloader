#!/bin/bash
if [[ -n "$1" ]]; then
  export TARGET=$1
fi
cmake -B build -G"Ninja"
cmake --build build --config Release
