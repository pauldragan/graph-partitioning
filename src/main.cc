#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/graph/graphml.hpp>
#include <boost/graph/graphviz.hpp>

#include <graph-partitioning/algorithms.hpp>
#include <graph-partitioning/graph.hpp>
#include <graph-partitioning/onnx_reader.hpp>
#include <graph-partitioning/onnx_writer.hpp>
#include <graph-partitioning/utils.hpp>

using namespace partition;
using namespace partition::utils;

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cout << "[ERROR] Please pass the path to the .toml configuration file." << std::endl;
    return -1;
  }
  std::string config_path(argv[1]);

  Config config = parse_config_file(config_path);

  auto g = readers::ONNXReader::from_bin(
      config.model_path);
  graph::VertexTypeProperty ntype = boost::get(graph::vertex_node_type, g);
  // graph::VertexNameProperty name = boost::get(graph::vertex_name, g);

  std::fstream orig_out_fp("./out_graph_orig.dot", std::ios::out);
  boost::write_graphviz(orig_out_fp, g, boost::make_label_writer(ntype));

  if (algorithms::has_cycles(g)) {
    std::cout << "[ERROR] Graph has cycles!" << std::endl;
    return -1;
  }

  std::unordered_set<std::string> rm_ops{config.unsupported_ops.begin(), config.unsupported_ops.end()};
  std::unordered_set<std::string> data_ops{config.data_ops.begin(), config.data_ops.end()};
  std::unordered_set<std::string> id_ops{config.identity_ops.begin(), config.identity_ops.end()};
  auto deleter = algorithms::RemoveUnsupportedNodes(rm_ops, data_ops, id_ops);
  graph::Graph g_mod(g);
  std::map<int, graph::Subgraph> g_mod_subgraphs;
  deleter(g, g_mod, g_mod_subgraphs);
  graph::VertexTypeProperty ntype_mod =
      boost::get(graph::vertex_node_type, g_mod);
  graph::VertexNameProperty name_mod = boost::get(graph::vertex_name, g_mod);
  graph::VertexCompatProperty compat_mod =
      boost::get(graph::vertex_compat, g_mod);
  graph::VertexSubgraphIndexProperty subgraph_mod =
      boost::get(graph::vertex_subgraph_index, g_mod);
  graph::EdgeTypeProperty etype_mod = boost::get(graph::edge_type, g_mod);
  graph::EdgeNameProperty ename_mod = boost::get(graph::edge_name, g_mod);

  auto subwriter = subgraph_writer(ntype_mod, compat_mod, subgraph_mod);

  std::fstream out_fp(config.output_dir + "/" + config.model_name + ".dot", std::ios::out);
  subwriter(out_fp, g_mod, g_mod_subgraphs,
            label_compat_writer(ntype_mod, compat_mod, subgraph_mod),
            edge_type_writer(etype_mod, ename_mod));

  graph::Graph g_summary;
  std::map<graph::Edge, graph::Edge> edge_map = algorithms::summarize_subgraphs(
      g_mod, g_summary, g_mod_subgraphs, etype_mod, ename_mod, compat_mod,
      subgraph_mod);
  graph::VertexTypeProperty ntype_summary =
      boost::get(graph::vertex_node_type, g_summary);
  graph::VertexCompatProperty compat_summary =
      boost::get(graph::vertex_compat, g_summary);
  graph::EdgeTypeProperty etype_summary =
      boost::get(graph::edge_type, g_summary);
  graph::EdgeNameProperty ename_summary =
      boost::get(graph::edge_name, g_summary);
  graph::VertexSubgraphIndexProperty subgraph_summary =
      boost::get(graph::vertex_subgraph_index, g_summary);
  std::fstream out_fp_summary(config.output_dir + "/" + config.model_name + "_summary.dot", std::ios::out);
  boost::write_graphviz(
      out_fp_summary, g_summary,
      label_compat_writer(ntype_summary, compat_summary, subgraph_summary),
      edge_type_writer(etype_summary, ename_summary));

  writers::ONNXWriter::to_bin(config.model_path,
                              config.output_dir, g_mod, g_mod_subgraphs);

  std::map<graph::Edge, std::string> inputs_map;
  std::map<graph::Edge, std::string> outputs_map;
  for (auto it = edge_map.begin(); it != edge_map.end(); it++) {
    graph::Edge orig_e = it->second;
    graph::Edge e = it->first;

    graph::Vertex u = boost::source(orig_e, g_mod);
    graph::Vertex v = boost::target(orig_e, g_mod);

    inputs_map[e] = name_mod[u];
    outputs_map[e] = name_mod[v];
  }

  boost::associative_property_map<std::map<graph::Edge, std::string>>
      inputs_prop_map(inputs_map);
  boost::associative_property_map<std::map<graph::Edge, std::string>>
      outputs_prop_map(outputs_map);

  std::fstream out_summary_graphml(config.output_dir + "/" + config.model_name + "_summary.xml", std::ios::out);
  boost::dynamic_properties dp;
  dp.property("subgraph", subgraph_summary);
  dp.property("input", inputs_prop_map);
  dp.property("output", outputs_prop_map);
  dp.property("tensor", ename_summary);
  boost::write_graphml(out_summary_graphml, g_summary, dp, true);
}
