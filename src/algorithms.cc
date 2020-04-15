#include <iostream>
#include <vector>
#include <utility>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/vf2_sub_graph_iso.hpp>

#include <graph-partitioning/graph.hpp>
#include <graph-partitioning/algorithms.hpp>

#include "algo_unsupported_detail.hpp"

namespace partition {
namespace algorithms {

  void RemoveUnsupportedNodes::operator()(const graph::Graph& g, graph::Graph& g_out, std::map<int, graph::Subgraph>& subgraphs_out) {
    using namespace partition::algorithms::detail;

    // graph::Graph G(g);
    graph::Graph& G = g_out;
    std::map<int, graph::Subgraph>& subgraph_map = subgraphs_out;

    graph::VertexTypeProperty ntype = boost::get(graph::vertex_node_type, G);
    graph::VertexCompatProperty compat = boost::get(graph::vertex_compat, G);
    graph::VertexSubgraphIndexProperty subgraph = boost::get(graph::vertex_subgraph_index, G);
    graph::EdgeTypeProperty etype = boost::get(graph::edge_type, G);
    graph::EdgeNameProperty ename = boost::get(graph::edge_name, G);

    std::vector<graph::Vertex> compat_nodes;
    std::vector<graph::Vertex> incompat_nodes;

    std::vector<graph::Edge> i2c_edges;
    std::vector<graph::Edge> i2i_edges;
    std::vector<graph::Edge> c2c_edges;
    std::vector<graph::Edge> c2i_edges;

    auto v_compat = compat_visitor(ops_,
				   ntype,
				   compat,
				   compat_nodes,
				   incompat_nodes
				   );

    auto e_compat = edge_compat_visitor(compat,
					etype,
					c2c_edges,
					c2i_edges,
					i2c_edges,
					i2i_edges
					);


    boost::depth_first_search(G,
			      boost::visitor(boost::make_dfs_visitor(v_compat)));

    std::vector<graph::Vertex> data_nodes;
    auto data_initialize = data_visitor_initialize(data_ops_, ntype, compat, data_nodes);
    boost::depth_first_search(G,
			      boost::visitor(boost::make_dfs_visitor(data_initialize)));

    std::vector<graph::Vertex> id_nodes;
    auto classify_identity = classify_identity_visitor(id_ops_, ntype, compat, id_nodes);
    boost::depth_first_search(G,
			      boost::visitor(boost::make_dfs_visitor(classify_identity)));

    auto relabel_visitor = relabel_data_visitor(ntype, compat);
    boost::depth_first_search(G,
			      boost::visitor(boost::make_dfs_visitor(relabel_visitor)));

    boost::depth_first_search(G,
			      boost::visitor(boost::make_dfs_visitor(e_compat)));

    std::map<int, std::vector<graph::Vertex>> subgraph_nodes;
    auto subgraph_initialize = subgraph_visitor_initialize(subgraph_nodes, compat, subgraph);
    auto subgraph_edges = subgraph_visitor_edges(subgraph_nodes, compat, subgraph);
    boost::depth_first_search(G,
			      boost::visitor(boost::make_dfs_visitor(boost::make_list(subgraph_initialize, subgraph_edges))));

    // remove empty subgraphs
    std::vector<int> subgraph_rm_list;
    for (auto it = subgraph_nodes.begin(); it != subgraph_nodes.end(); it++) {
      if (it->second.empty()) {
	subgraph_rm_list.push_back(it->first);
      }
      else {
	subgraph_map[it->first] = graph::Subgraph();
      }
    }
    for (auto it = subgraph_rm_list.begin(); it != subgraph_rm_list.end(); it++) {
      subgraph_nodes.erase(*it);
    }

    // Create subgraph structs
    for (auto it = subgraph_map.begin(); it != subgraph_map.end(); it++) {
      it->second.nodes = std::move(subgraph_nodes[it->first]);
    }

    assign_edges_subgraph_visitor assign_edges(subgraph_map, compat, subgraph);
    boost::depth_first_search(G,
			      boost::visitor(boost::make_dfs_visitor(assign_edges)));


    // Find 0 input subgraphs and merge with output subgraph
    subgraph_rm_list.clear();
    for (auto it = subgraph_map.begin(); it != subgraph_map.end(); it++) {
      graph::Subgraph& subg = it->second;
      if (subg.input_edges.size() > 0) {
    	continue;
      }
      graph::Compat subg_compat = compat[*subg.nodes.begin()];
      std::unordered_set<int> output_subgs;
      for (auto it2 = subg.output_edges.begin(); it2 != subg.output_edges.end(); it2++) {
    	// graph::Vertex v = boost::target(it2->get(), G);
    	graph::Vertex v = boost::target(*it2, G);

	if (compat[v] == subg_compat) {
	  output_subgs.insert(subgraph[v]);
	}
      }
      if (output_subgs.size() > 1 || output_subgs.size() == 0) {
    	continue;
      }
      int target_idx = *output_subgs.begin();
      graph::Subgraph& target_subg = subgraph_map[target_idx];
      merge_subgraphs(it->first, subg, target_idx, target_subg, subgraph);

      subgraph_rm_list.push_back(it->first);
    }
    for (auto it = subgraph_rm_list.begin(); it != subgraph_rm_list.end(); it++) {
      subgraph_map.erase(*it);
    }


    bool modified;
    do {
      modified = false;

      graph::Graph G_summary;
      std::map<graph::Edge, graph::Edge> edge_map = algorithms::summarize_subgraphs(G,
										    G_summary,
										    subgraph_map,
										    etype,
										    ename,
										    compat,
										    subgraph);
      graph::VertexSubgraphIndexProperty sum_subgraph = boost::get(graph::vertex_subgraph_index, G_summary);
      std::vector<graph::Edge> cycles;
      record_cycles_visitor cycles_visitor(cycles, sum_subgraph);
      boost::depth_first_search(G_summary,
				boost::visitor(boost::make_dfs_visitor(cycles_visitor)));

      if (cycles.size() > 0) {
	std::cout << "Cycles found: " << std::endl;
	auto it = cycles.begin();

	graph::Edge sum_e = *it;
	graph::Edge e = edge_map[*it];

	graph::Vertex u = boost::source(e, G);
	graph::Vertex v = boost::target(e, G);

	std::cout << sum_e << " (orig:" << e << " " << ntype[u] << "->" << ntype[v] << ")"<< " " << std::endl;

	int next_subgraph_idx = subgraph_map.rbegin()->first + 1;
	int old_subgraph_idx = subgraph[v];

	boost::put(subgraph, v, next_subgraph_idx);

	subgraph_map[next_subgraph_idx] = graph::Subgraph();
	subgraph_map[next_subgraph_idx].nodes.push_back(v);

	auto& old_subgraph = subgraph_map[old_subgraph_idx];
	auto old_location = std::find(old_subgraph.nodes.begin(), old_subgraph.nodes.end(), v);
	if (old_location != old_subgraph.nodes.end()) {
	  old_subgraph.nodes.erase(old_location);
	}

	std::cout << "Removing old edges" << std::endl;
	for (auto it = subgraph_map.begin(); it != subgraph_map.end(); it++) {
	  it->second.interior_edges.clear();
	  it->second.output_edges.clear();
	  it->second.input_edges.clear();
	}

	boost::depth_first_search(G,
				  boost::visitor(boost::make_dfs_visitor(assign_edges)));

	modified = true;
      }
    } while (modified);

  }

  bool has_cycles(const graph::Graph& g) {
    using namespace detail;
    bool out;
    cycles_visitor visitor(&out);
    boost::depth_first_search(g,
			      boost::visitor(boost::make_dfs_visitor(visitor)));
    return out;
  }


} // algorithms
} // partition
