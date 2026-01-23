#!/usr/bin/env bash
set -e  # stop on first error

echo "Configuring and building release binary..."
cmake -S . -B build/debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/debug -j

echo "[INFO] RUN AS: ./build/debug/bin/Estimator" 

