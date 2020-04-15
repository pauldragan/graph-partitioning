#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/topological_sort.hpp>

#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>

#include <onnx/onnx-ml.proto3.pb.h>

#include <graph-partitioning/graph.hpp>
#include <graph-partitioning/onnx_writer.hpp>

using namespace partition;
using namespace partition::writers;

void ONNXWriter::to_bin(std::string orig_path, std::string out_dir,
                        graph::Graph &g_partitioned,
                        std::map<int, graph::Subgraph> &subgraphs) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  std::fstream modelpb(orig_path, std::ios::in | std::ios::binary);

  onnx::ModelProto model;
  if (!model.ParseFromIstream(&modelpb)) {
    std::cerr << "[ERROR] ONNXWriter: failed to parse from stream!"
              << std::endl;
    return;
  }
  const onnx::GraphProto &orig_graph = model.graph();

  graph::VertexNameProperty vnames =
      boost::get(graph::vertex_name, g_partitioned);
  graph::VertexSubgraphIndexProperty vsubgraphs =
      boost::get(graph::vertex_subgraph_index, g_partitioned);
  graph::EdgeNameProperty enames = boost::get(graph::edge_name, g_partitioned);

  std::map<std::string, onnx::TensorProto> named_initializers;
  std::map<std::string, onnx::ValueInfoProto> named_inputs;
  std::map<std::string, onnx::ValueInfoProto> named_outputs;
  std::map<std::string, onnx::ValueInfoProto> named_values;
  for (auto it = orig_graph.value_info().begin();
       it != orig_graph.value_info().end(); it++) {
    named_values[it->name()] = *it;
  }
  for (auto it = orig_graph.input().begin(); it != orig_graph.input().end();
       it++) {
    named_inputs[it->name()] = *it;
    named_values[it->name()] = *it;
  }
  for (auto it = orig_graph.output().begin(); it != orig_graph.output().end();
       it++) {
    named_outputs[it->name()] = *it;
    named_values[it->name()] = *it;
  }
  for (auto it = orig_graph.initializer().begin();
       it != orig_graph.initializer().end(); it++) {
    named_initializers[it->name()] = *it;
  }

  std::map<std::string, onnx::NodeProto> named_nodes;
  std::map<std::string, std::vector<std::string>> node_inputs;
  std::map<std::string, std::vector<std::string>> node_outputs;
  int i = 0;
  for (auto it = orig_graph.node().begin(); it < orig_graph.node().end();
       it++) {
    std::string node_name("node_" + std::to_string(i));

    named_nodes[node_name] = *it;

    for (auto it2 = it->input().begin(); it2 < it->input().end(); it2++) {
      node_inputs[node_name].push_back(*it2);
    }
    for (auto it2 = it->output().begin(); it2 < it->output().end(); it2++) {
      node_outputs[node_name].push_back(*it2);
    }
    i++;
  }

  std::vector<graph::Vertex> vsorted;
  boost::topological_sort(g_partitioned, std::back_inserter(vsorted));
  std::reverse(vsorted.begin(), vsorted.end());

  std::map<int, std::vector<std::string>> subgraph_nodes; // Needs to be ordered
  std::map<int, std::unordered_set<std::string>> subgraph_initializers;
  std::map<int, std::unordered_set<std::string>> subgraph_inputs;
  std::map<int, std::unordered_set<std::string>> subgraph_outputs;
  std::map<int, std::unordered_set<std::string>> subgraph_values;

  for (auto it = vsorted.begin(); it != vsorted.end(); it++) {
    graph::Vertex u = *it;
    std::string n = vnames[u];
    int subgraph_index = vsubgraphs[u];
    if (named_nodes.find(n) != named_nodes.end()) {
      subgraph_nodes[subgraph_index].push_back(n);
    }
  }

  for (auto it = subgraphs.begin(); it != subgraphs.end(); it++) {
    for (auto it2 = it->second.output_edges.begin();
         it2 != it->second.output_edges.end(); it2++) {
      graph::Edge e = *it2;
      std::string en = enames[e];
      subgraph_outputs[it->first].insert(en);
    }

    for (auto it2 = it->second.input_edges.begin();
         it2 != it->second.input_edges.end(); it2++) {
      graph::Edge e = *it2;
      std::string en = enames[e];
      subgraph_inputs[it->first].insert(en);
    }

    for (auto it2 = subgraph_inputs[it->first].begin();
         it2 != subgraph_inputs[it->first].end(); it2++) {
      if (named_initializers.find(*it2) != named_initializers.end()) {
        subgraph_initializers[it->first].insert(*it2);
      }
    }
  }

  std::map<int, onnx::ModelProto> onnx_models;
  for (auto it = subgraphs.begin(); it != subgraphs.end(); it++) {
    onnx::ModelProto subgraph_model;
    subgraph_model.set_ir_version(model.ir_version());
    subgraph_model.set_producer_version(model.producer_version());
    subgraph_model.set_domain(model.domain());
    subgraph_model.set_model_version(model.model_version());
    subgraph_model.set_doc_string(model.doc_string());
    *subgraph_model.mutable_opset_import() = {
        model.mutable_opset_import()->begin(),
        model.mutable_opset_import()->end()};
    *subgraph_model.mutable_metadata_props() = {
        model.mutable_metadata_props()->begin(),
        model.mutable_metadata_props()->end()};
    *subgraph_model.mutable_training_info() = {
        model.mutable_training_info()->begin(),
        model.mutable_training_info()->end()};

    onnx::GraphProto *subgraph_graph = subgraph_model.mutable_graph();
    subgraph_graph->set_name("subgraph_" + std::to_string(it->first));
    subgraph_graph->set_doc_string(orig_graph.doc_string());

    for (auto it2 = subgraph_nodes[it->first].begin();
         it2 != subgraph_nodes[it->first].end(); it2++) {
      if (named_nodes.find(*it2) != named_nodes.end()) {
        onnx::NodeProto *subgraph_node = subgraph_graph->add_node();
        subgraph_node->CopyFrom(named_nodes[*it2]);
      }
    }

    for (auto it2 = subgraph_outputs[it->first].begin();
         it2 != subgraph_outputs[it->first].end(); it2++) {
      onnx::ValueInfoProto *out = subgraph_graph->add_output();
      if (named_values.find(*it2) != named_outputs.end()) {
        out->CopyFrom(named_values[*it2]);
      } else {
        onnx::ValueInfoProto new_out;
        new_out.set_name(*it2);
        new_out.mutable_type()->mutable_tensor_type()->set_elem_type(1);
        out->CopyFrom(new_out);
      }
    }

    for (auto it2 = subgraph_inputs[it->first].begin();
         it2 != subgraph_inputs[it->first].end(); it2++) {
      onnx::ValueInfoProto *inp = subgraph_graph->add_input();
      if (named_values.find(*it2) != named_values.end()) {
        inp->CopyFrom(named_values[*it2]);
      } else {
        onnx::ValueInfoProto new_inp;
        new_inp.set_name(*it2);
        new_inp.mutable_type()->mutable_tensor_type()->set_elem_type(1);
        inp->CopyFrom(new_inp);
      }
    }

    for (auto it2 = subgraph_initializers[it->first].begin();
         it2 != subgraph_initializers[it->first].end(); it2++) {
      onnx::TensorProto *ini = subgraph_graph->add_initializer();
      ini->CopyFrom(named_initializers[*it2]);
    }

    // std::string json;
    // // google::protobuf::util::MessageToJsonString(subgraph_model, &json);
    // google::protobuf::TextFormat::PrintToString(subgraph_model, &json);
    // std::cout << subgraph_graph->name() << ":" << std::endl;
    // std::cout << json << std::endl;

    onnx_models[it->first] = subgraph_model;
  }

  for (auto it = onnx_models.begin(); it != onnx_models.end(); it++) {
    std::stringstream file_name;
    file_name << out_dir << "/"
              << "subgraph_" << it->first << ".onnx";
    std::fstream subgraphpb(file_name.str(), std::ios::out | std::ios::binary);

    it->second.SerializeToOstream(&subgraphpb);
  }
}
