#!/usr/bin/env bash

TREXTON_PATH=~/Documents/Internship/treXton/

rm -rf build/
mkdir build
cd build/
cmake ..
make
cp relocalize.so $TREXTON_PATH
