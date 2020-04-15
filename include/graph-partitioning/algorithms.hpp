#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include <unordered_set>
#include <string>
#include <utility>

#include <graph-partitioning/graph.hpp>

namespace partition {
namespace algorithms {

class RemoveUnsupportedNodes {
public:
  RemoveUnsupportedNodes(const std::unordered_set<std::string>& ops,
			 const std::unordered_set<std::string>& data_ops,
			 const std::unordered_set<std::string>& id_ops) :
    ops_{ops},
    data_ops_{data_ops},
    id_ops_{id_ops}
  {};

  void operator()(const graph::Graph& g, graph::Graph& g_out, std::map<int, graph::Subgraph>& subgraphs_out);
private:
  const std::unordered_set<std::string>& ops_;
  const std::unordered_set<std::string>& data_ops_;
  const std::unordered_set<std::string>& id_ops_;
};

  template <typename EType, typename EName, typename VCompat, typename VSubgraph>
  std::map<graph::Edge, graph::Edge> summarize_subgraphs(const graph::Graph& g_orig,
							 graph::Graph& g,
							 std::map<int, graph::Subgraph>& subgraphs,
							 EType& orig_etype,
							 EName& orig_ename,
							 VCompat& orig_vcompat,
							 VSubgraph& orig_vsubgraph) {

    graph::VertexCompatProperty compat = boost::get(graph::vertex_compat, g);
    graph::EdgeTypeProperty etype = boost::get(graph::edge_type, g);
    graph::EdgeNameProperty ename = boost::get(graph::edge_name, g);
    graph::VertexTypeProperty ntype = boost::get(graph::vertex_node_type, g);
    graph::VertexSubgraphIndexProperty subgraph = boost::get(graph::vertex_subgraph_index, g);

    std::map<int, int> index_map;
    int i = 0;
    for (auto it = subgraphs.begin(); it != subgraphs.end(); it++) {
      graph::Vertex u = boost::add_vertex(g);
      index_map[it->first] = u;
      boost::put(compat, u, orig_vcompat[it->second.nodes[0]]);
      boost::put(ntype, u, "subgraph");
      boost::put(subgraph, u, it->first);
      i++;
    }

    std::map<graph::Edge, graph::Edge> edge_map;

    for (auto it = subgraphs.begin(); it != subgraphs.end(); it++) {
      for (auto it2 = it->second.output_edges.begin(); it2 != it->second.output_edges.end(); it2++) {
	graph::Edge e = *it2;
	graph::Vertex u = boost::source(e, g_orig);
	graph::Vertex v = boost::target(e, g_orig);

	int u_idx = index_map[orig_vsubgraph[u]];
	int v_idx = index_map[orig_vsubgraph[v]];

	graph::Edge new_e = boost::add_edge(u_idx, v_idx, g).first;

	boost::put(etype, new_e, orig_etype[e]);
	boost::put(ename, new_e, orig_ename[e]);

	if (edge_map.find(new_e) != edge_map.end()) {
	  std::cout << "[ERROR] " << new_e << " already mapped!";
	}
	edge_map[new_e] = e;
      }
    }

    return edge_map;
  };

  bool has_cycles(const graph::Graph& g);

} // algorithms
} // partition

#endif // ALGORITHMS_HPP
