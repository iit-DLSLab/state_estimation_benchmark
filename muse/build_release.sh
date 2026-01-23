#!/usr/bin/env bash
set -e  # stop on first error

echo "Configuring and building release binary..."
cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release
cmake --build build/release -j

echo "[INFO] RUN AS: ./build/release/bin/Estimator" 

