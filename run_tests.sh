#!/bin/bash

# resnet50_onnx test
./build/graph_partitioning tests/resnet50_onnx.toml
dot -Tpdf out_resnet50_onnx/resnet50_onnx_orig.dot -o out_resnet50_onnx/resnet50_onnx_orig.pdf
dot -Tpdf out_resnet50_onnx/resnet50_onnx.dot -o out_resnet50_onnx/resnet50_onnx.pdf
dot -Tpdf out_resnet50_onnx/resnet50_onnx_summary.dot -o out_resnet50_onnx/resnet50_onnx_summary.pdf
python tests/test_onnx.py tests/resnet50_onnx.toml
