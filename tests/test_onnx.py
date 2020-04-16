import numpy as np
import onnxruntime
import onnx
from onnx import numpy_helper
import networkx as nx
from os import path
import sys
import toml

assert len(sys.argv) == 2, "[ERROR] Please pass path to the .toml configuration file."

CONFIG = toml.load(sys.argv[1])
print("Running test with config:")
print(CONFIG)

model = onnx.load(CONFIG["model_path"])

inp = None
exp_out = None

input_tensor = onnx.TensorProto()
exp_output_tensor = onnx.TensorProto()
with open(CONFIG["test_data_input"], "rb") as fp:
    input_tensor.ParseFromString(fp.read())
with open(CONFIG["test_data_output"], "rb") as fp:
    exp_output_tensor.ParseFromString(fp.read())

inp = numpy_helper.to_array(input_tensor)
exp_out = numpy_helper.to_array(exp_output_tensor)

session = onnxruntime.InferenceSession(CONFIG["model_path"], None)
input_name = session.get_inputs()[0].name
final_output_name = session.get_outputs()[0].name

output = session.run([], {input_name: inp})[0]

np.testing.assert_almost_equal(exp_out, output)

G = nx.read_graphml(path.join(CONFIG["output_dir"], CONFIG["model_name"] + "_summary.xml"))
onnx_models = {}

skip = []

for n in G.nodes.data("subgraph"):
    subgraph_name = "subgraph_" + str(n[1]) + ".onnx"
    subgraph_path = CONFIG["output_dir"] + "/" + subgraph_name
    print(subgraph_path)

    proto = onnx.load(subgraph_path)
    if (len(proto.graph.output) == 0):
        skip.append(n[1])
        continue

    sess = onnxruntime.InferenceSession(subgraph_path, None)
    onnx_models.update({n[1]: sess})
    inputs = [v.name for v in sess.get_inputs()]
    outputs = [v.name for v in sess.get_outputs()]
    print("Inputs: ", inputs)
    print("Outputs: ", outputs)

print("Skipping subgraphs: ", skip)

inputs_dict = {input_name: inp}
topo = nx.topological_sort(G)
for n in topo:
    subg_idx = G.nodes[n]["subgraph"]
    if subg_idx in skip:
        continue

    sess = onnx_models[subg_idx]
    feed_dict = {}
    for iname in [v.name for v in sess.get_inputs()]:
        val = inputs_dict.get(iname, None)
        if val is None:
            raise RuntimeError("Input {} not found.".format(iname))
        feed_dict.update({iname: val})
    outputs = sess.run([], feed_dict)
    outputs_name = [v.name for v in sess.get_outputs()]
    for i in range(len(outputs)):
        oname = outputs_name[i]
        val = outputs[i]
        dict_entry = {oname: val}

        inputs_dict.update(dict_entry)


assert np.argmax(inputs_dict[final_output_name]) == np.argmax(exp_out), "Test failed."
