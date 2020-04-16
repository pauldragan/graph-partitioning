#!/bin/bash

ROOTDIR=$(pwd)

mkdir test-models
cd test-models
mkdir onnx
cd onnx

wget https://s3.amazonaws.com/download.onnx/models/opset_8/resnet50.tar.gz
tar -xf resnet50.tar.gz
rm resnet50.tar.gz

python $ROOTDIR/onnx_shape_inference.py -i resnet50/model.onnx

cd $ROOTDIR
