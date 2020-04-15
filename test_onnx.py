import numpy as np
import onnxruntime
import onnx
from onnx import numpy_helper
import networkx as nx
import pdb

model = onnx.load("onnx_models/resnet50/model_shapes.onnx")

inp = None
exp_out = None

input_tensor = onnx.TensorProto()
exp_output_tensor = onnx.TensorProto()
with open("onnx_models/resnet50/test_data_set_0/input_0.pb", "rb") as fp:
    input_tensor.ParseFromString(fp.read())
with open("onnx_models/resnet50/test_data_set_0/output_0.pb", "rb") as fp:
    exp_output_tensor.ParseFromString(fp.read())

inp = numpy_helper.to_array(input_tensor)
exp_out = numpy_helper.to_array(exp_output_tensor)

session = onnxruntime.InferenceSession("onnx_models/resnet50/model_shapes.onnx", None)
input_name = session.get_inputs()[0].name
final_output_name = session.get_outputs()[0].name

output = session.run([], {input_name: inp})[0]

np.testing.assert_almost_equal(exp_out, output)

print(np.argmax(exp_out), np.argmax(output))

subgraph_dir = "build/out_pbs"
G = nx.read_graphml("build/out_graph_summary.xml")
onnx_models = {}

skip = []

for n in G.nodes.data("subgraph"):
    print(n)
    path = subgraph_dir + "/subgraph_" + str(n[1]) + ".onnx"

    proto = onnx.load(path)
    if (len(proto.graph.output) == 0):
        skip.append(n[1])
        continue

    sess = onnxruntime.InferenceSession(path, None)
    onnx_models.update({n[1]: sess})
    inputs = [v.name for v in sess.get_inputs()]
    outputs = [v.name for v in sess.get_outputs()]
    print("Inputs: ", inputs)
    print("Outputs: ", outputs)

print("Skipping subgraphs: ", skip)

inputs_dict = {input_name: inp}
topo = nx.topological_sort(G)
for n in topo:
    print(G.nodes[n]["subgraph"])
    subg_idx = G.nodes[n]["subgraph"]
    if subg_idx in skip:
        continue

    sess = onnx_models[subg_idx]
    feed_dict = {}
    for iname in [v.name for v in sess.get_inputs()]:
        # print(iname, inputs_dict.keys())
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
        # print(dict_entry)

        inputs_dict.update(dict_entry)

print(np.argmax(inputs_dict[final_output_name]))
print(np.argmax(exp_out), np.argmax(output))
