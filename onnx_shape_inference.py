import onnx
from onnx import helper, shape_inference
from onnx import TensorProto
import argparse
import os

parser = argparse.ArgumentParser("Add shape information to ONNX models.")
parser.add_argument("-i", type=str, required=True, help="Input model path.")
# parser.add_argument("-o", type=str, help="Output model path.")

args = parser.parse_args()

original_model = onnx.load(args.i)

# Check the model and print Y's shape information
onnx.checker.check_model(original_model)


# Apply shape inference on the model
inferred_model = shape_inference.infer_shapes(original_model)

# Check the model and print Y's shape information
onnx.checker.check_model(inferred_model)

orig_name = os.path.basename(args.i)
orig_dir = os.path.dirname(args.i)

new_name = orig_name.split('.')[0] + "_shapes.onnx"
output_path = os.path.join(orig_dir, new_name)

onnx.save(inferred_model, output_path)
