#!/bin/bash

ROOTDIR=$(pwd)

bash proto-gen.sh
bash get_models.sh

mkdir build
cd build
cmake ..
make -j8

cd $ROOTDIR
