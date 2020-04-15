#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>

#include <onnx/onnx-ml.proto3.pb.h>

#include <graph-partitioning/graph.hpp>
#include <graph-partitioning/onnx_reader.hpp>

using namespace partition;
using namespace partition::readers;

graph::Graph ONNXReader::from_bin(std::string path) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  typedef std::pair<int, int> Edge;

  std::fstream modelpb (path,
			std::ios::in |
			std::ios::binary);

  onnx::ModelProto model;
  if (!model.ParseFromIstream(&modelpb)) {
    std::cerr << "[ERROR] ONNXReader: failed to parse from stream!" << std::endl;
    return graph::Graph();
  }

  onnx::GraphProto graph = model.graph();
  int V = graph.node().size();
  std::cout << "ONNX model has " << V << " nodes." << std::endl;
  graph::Graph g(V);

  graph::VertexNameProperty name = boost::get(graph::vertex_name, g);
  graph::VertexTypeProperty ntype = boost::get(graph::vertex_node_type, g);
  graph::VertexCompatProperty compat = boost::get(graph::vertex_compat, g);
  graph::VertexSubgraphIndexProperty subgraph = boost::get(graph::vertex_subgraph_index, g);
  graph::EdgeTypeProperty etype = boost::get(graph::edge_type, g);
  graph::EdgeIndexProperty eindex = boost::get(graph::edge_index, g);
  graph::EdgeNameProperty ename = boost::get(graph::edge_name, g);

  std::map<std::string, int> output_tensors;
  std::map<std::string, int> model_output_tensors;  

  int i = 0;
  for (auto it = graph.node().begin(); it < graph.node().end(); it++) {
    for (auto it2 = it->output().begin(); it2 < it->output().end(); it2++) {
      output_tensors[*it2] = i;
    }
    i++;
  }

  std::vector<int> input_nodes;
  std::vector<int> output_nodes;  
  for (auto it = graph.input().begin(); it != graph.input().end(); it++) {
    output_tensors[it->name()] = i;
    input_nodes.push_back(i);
    i++;
  }
  for (auto it = graph.output().begin(); it != graph.output().end(); it++) {
    model_output_tensors[it->name()] = i;
    output_nodes.push_back(i);
    i++;
  }  

  std::vector<Edge> edges;
  std::map<int, std::string> names;
  std::map<int, std::string> ops;
  std::map<Edge, std::string> edge_names;

  i = 0;
  for (auto it = graph.node().begin(); it < graph.node().end(); it++) {
    std::string node_name("node_" + std::to_string(i));
    names[i] = node_name;
    ops[i] = it->op_type();
    std::cout << node_name << " [" << i << "] " << ": " << it->op_type();

    for (auto it2 = it->input().begin(); it2 < it->input().end(); it2++) {
      if (output_tensors.find(*it2) == output_tensors.end()) {
	continue;
      }
      Edge e(output_tensors[*it2], i);
      edges.push_back(e);
      edge_names[e] = *it2;
      std::cout << "    " << e.first << " -> " << e.second;
    }
    for (auto it2 = it->output().begin(); it2 < it->output().end(); it2++) {
      if (model_output_tensors.find(*it2) == model_output_tensors.end()) {
	continue;
      }
      std::cout << "found output!!!" << std::endl;
      Edge e(i, model_output_tensors[*it2]);
      edges.push_back(e);
      edge_names[e] = *it2;
      std::cout << "    " << e.first << " -> " << e.second;
    }    
    i++;
    std::cout << std::endl;
  }

  for (auto it = input_nodes.begin(); it != input_nodes.end(); it++) {
    std::string node_name("placeholder_" + std::to_string(i));
    names[i] = node_name;
    ops[i] = "Input_Placeholder";
    i++;
  }

  for (auto it = output_nodes.begin(); it != output_nodes.end(); it++) {
    std::string node_name("placeholder_" + std::to_string(i));
    names[i] = node_name;
    ops[i] = "Output_Placeholder";
    i++;
  }  

  for (auto it = edges.begin(); it < edges.end(); it++) {
    // Find first
    auto it_first = names.find(it->first);
    if (it_first == names.end()) {
      continue;
    }
    // Find second
    auto it_second = names.find(it->second);
    if (it_second == names.end()) {
      continue;
    }
    int n1 = it->first;
    int n2 = it->second;
    auto e_out = boost::add_edge(n1, n2, g);
    auto e = e_out.first;

    std::cout << "(" << it->first << ", " << it->second << ") "
	      << e_out.first << " " << e_out.second << std::endl;

    boost::put(name, n1, it_first->second);
    boost::put(name, n2, it_second->second);

    boost::put(ntype, n1, ops[it->first]);
    boost::put(ntype, n2, ops[it->second]);

    boost::put(compat, n1, graph::Compat::UNKNOWN);
    boost::put(compat, n2, graph::Compat::UNKNOWN);

    boost::put(subgraph, n1, -1);
    boost::put(subgraph, n2, -1);

    boost::put(etype, e, graph::ETypes::UNKNOWN);
    boost::put(ename, e, edge_names[*it]);
    boost::put(eindex, e, -1);

    std::cout << it->first << " : " << ntype[n1] << " -> " << it->second << " : " << ntype[n2] << std::endl;
  }

  return g;
}
