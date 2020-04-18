## Automatic partitioning of computational graphs.
The purpose of this code repository is to explore different algorithmic partitioning schemes
of computational graphs (DAGs, often present in machine learning applications).

Partitioning might represent a necessity in the final stages of the deployment when one
wishes to take full advantage of the available hardware acceleration capabilities, but
not the entire used operator set is supported by the inference software provided by the
hardware manufacturer. In this case, a solution would be to execute the unsupported parts
using a more generic inference engine.

## Tool input
The tool expects as input a computational DAG, which can contain constant data, e.g. weights,
together with a configuration file that is specific to the partitioning algorithm.

Currently, ONNX files are supported and the configuration is given as TOML files.

## Tool output
The tool generates as output all the sub-graphs resulting from the partitioning algorithm,
using the same format as the input (only ONNX supported currently), together with a GraphML
description of how the resulting pieces should be connected. The graph nodes and constant data
values are copied into the generated sub-graphs from the original DAG.

After all the connections are made (externally), the new computational graph should be
functionally identical to the original one.

## Current state
### Partitioning based on supported / unsupported operator sets
The current implementation aims to create maximal clusters (sub-graphs) based on
an operator blacklist, intending to represent the set of operators that are not
supported by a certain inference engine.

For example, the following configuration file:

```
model_name = "resnet50_onnx"
output_dir = "./out_resnet50_onnx"
model_path = "test-models/onnx/resnet50/model_shapes.onnx"
model_type = "onnx"
unsupported_ops = ["Sum", "BatchNormalization"]
data_ops = ["Input_Placeholder", "Output_Placeholder"]
identity_ops = []
test_data_input = "test-models/onnx/resnet50/test_data_set_0/input_0.pb"
test_data_output = "test-models/onnx/resnet50/test_data_set_0/output_0.pb"
```

specifies that the *Sum* and *BatchNormalization* operators are considered *unsupported*.
The list of data (constants, inputs, outputs) operators and identity operators must be also provided
to the tool for better partitioning.

Given the above and a ResNet50 model, the partitioning from Figure 1. and Figure 2. (see below) will result
and associated ONNX models (one for each sub-graph) will be generated.

## Build and run examples
This project depends on the Boost library version 1.72 and on the Python packages listed in
the requirements.txt file.

When cloning this repository, make sure to initialize and update the sub-modules.

Then, from the root directory execute:

```
bash build.sh
```

to build the software and then:

```
bash run_tests.sh
```

to run the examples.

The output directories will contain the graphviz visualizations of the original graph,
the partitioned version, and a summary graph showing the connections between sub-graphs.


## Figures
**Figure 1.** Resulting partitioning of ResNet50.
![ResNet50 unsupported ops partitioning](https://user-images.githubusercontent.com/6622780/79663627-3830b000-81b5-11ea-91cd-1e5e8569c836.png)

**Figure 2.** Resulting partitioning of ResNet50 - connections between sub-graphs.
![ResNet50 unsupported ops partitioning summary](https://user-images.githubusercontent.com/6622780/79663627-3830b000-81b5-11ea-91cd-1e5e8569c836.png)