import onnx
from onnx import helper, shape_inference
from onnx import TensorProto
import pdb


original_model = onnx.load("onnx_models/resnet50/model.onnx")

# Check the model and print Y's shape information
onnx.checker.check_model(original_model)


# Apply shape inference on the model
inferred_model = shape_inference.infer_shapes(original_model)

# Check the model and print Y's shape information
onnx.checker.check_model(inferred_model)

onnx.save(inferred_model, "onnx_models/resnet50/model_shapes.onnx")
