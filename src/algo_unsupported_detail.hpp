#ifndef ALGO_UNSUPPORTED_DETAIL_HPP
#define ALGO_UNSUPPORTED_DETAIL_HPP

#include <algorithm>
#include <iostream>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>

#include <graph-partitioning/algorithms.hpp>
#include <graph-partitioning/graph.hpp>

namespace partition::algorithms::detail {

template <typename CompatProp, typename SubgraphProp>
struct assign_edges_subgraph_visitor
    : public boost::base_visitor<
          assign_edges_subgraph_visitor<CompatProp, SubgraphProp>> {
  assign_edges_subgraph_visitor(std::map<int, graph::Subgraph> &subgraph_map,
                                const CompatProp &compat_prop,
                                SubgraphProp &subgraph_prop)
      : subgraph_map_{subgraph_map}, compat_prop_{compat_prop},
        subgraph_prop_{subgraph_prop} {};

  typedef boost::on_examine_edge event_filter;
  void operator()(graph::Edge e, const graph::Graph &g) {
    graph::Vertex u = boost::source(e, g);
    graph::Vertex v = boost::target(e, g);

    int u_sub_idx = subgraph_prop_[u];
    int v_sub_idx = subgraph_prop_[v];

    if (u_sub_idx == v_sub_idx) {
      subgraph_map_[u_sub_idx].interior_edges.push_back(e);
    } else {
      subgraph_map_[u_sub_idx].output_edges.push_back(e);
      subgraph_map_[v_sub_idx].input_edges.push_back(e);
    }
  }

  std::map<int, graph::Subgraph> &subgraph_map_;
  const CompatProp &compat_prop_;
  SubgraphProp &subgraph_prop_;
};

template <typename TypeProp, typename CompatProp>
struct relabel_data_visitor
    : public boost::base_visitor<relabel_data_visitor<TypeProp, CompatProp>> {
  relabel_data_visitor(const TypeProp &type_prop, CompatProp &compat_prop)
      : type_prop_{type_prop}, compat_prop_{compat_prop} {}

  typedef boost::on_finish_vertex event_filter;

  void operator()(graph::Vertex u, const graph::Graph &g) {
    if (compat_prop_[u] != graph::Compat::DATA) {
      return;
    }

    std::vector<graph::Compat> target_types;
    std::pair<graph::OutEdgeIterator, graph::OutEdgeIterator> ep;
    for (ep = boost::out_edges(u, g); ep.first < ep.second; ep.first++) {
      graph::Vertex v = boost::target(*ep.first, g);
      target_types.push_back(compat_prop_[v]);
    }

    if (target_types.empty()) {
      return;
    }

    graph::Compat first_type = *target_types.begin();

    bool same_type = std::all_of(
        target_types.begin(), target_types.end(),
        [first_type](graph::Compat type) { return type == first_type; });

    if (same_type) {
      boost::put(compat_prop_, u, first_type);
    }
  }

  const TypeProp &type_prop_;
  CompatProp &compat_prop_;
};

template <typename TypeProp, typename CompatProp>
struct classify_identity_visitor
    : public boost::base_visitor<
          classify_identity_visitor<TypeProp, CompatProp>> {
  classify_identity_visitor(const std::unordered_set<std::string> &ops,
                            const TypeProp &type_prop, CompatProp &compat_prop,
                            std::vector<graph::Vertex> &identity_nodes)
      : ops_{ops}, type_prop_{type_prop}, compat_prop_{compat_prop},
        identity_nodes_{identity_nodes} {}

  typedef boost::on_tree_edge event_filter;

  void operator()(graph::Edge e, const graph::Graph &g) {
    graph::Vertex u = boost::source(e, g);
    graph::Vertex v = boost::target(e, g);

    if (ops_.find(type_prop_[v]) != ops_.end()) {
      boost::put(compat_prop_, v, compat_prop_[u]);
      identity_nodes_.push_back(v);
    }
  }

  const std::unordered_set<std::string> &ops_;
  const TypeProp &type_prop_;
  CompatProp &compat_prop_;
  std::vector<graph::Vertex> &identity_nodes_;
};

template <typename TypeProp, typename CompatProp>
struct data_visitor_initialize
    : public boost::base_visitor<
          data_visitor_initialize<TypeProp, CompatProp>> {
  data_visitor_initialize(const std::unordered_set<std::string> &ops,
                          const TypeProp &type_prop, CompatProp &compat_prop,
                          std::vector<graph::Vertex> &data_nodes)
      : ops_{ops}, type_prop_{type_prop}, compat_prop_{compat_prop},
        data_nodes_{data_nodes} {}

  typedef boost::on_initialize_vertex event_filter;

  void operator()(graph::Vertex u, const graph::Graph &g) {
    if (ops_.find(type_prop_[u]) != ops_.end()) {
      boost::put(compat_prop_, u, graph::Compat::DATA);
      data_nodes_.push_back(u);
    }
  }

  const std::unordered_set<std::string> &ops_;
  const TypeProp &type_prop_;
  CompatProp &compat_prop_;
  std::vector<graph::Vertex> &data_nodes_;
};

template <typename TypeProp, typename CompatProp>
struct compat_visitor
    : public boost::base_visitor<compat_visitor<TypeProp, CompatProp>> {

  compat_visitor(const std::unordered_set<std::string> &ops,
                 const TypeProp &type_prop, CompatProp &compat_prop,
                 std::vector<graph::Vertex> &compat_nodes,
                 std::vector<graph::Vertex> &incompat_nodes)
      : ops_{ops}, type_prop_{type_prop}, compat_prop_{compat_prop},
        compat_nodes_{compat_nodes}, incompat_nodes_{incompat_nodes} {}

  typedef boost::on_initialize_vertex event_filter;
  void operator()(graph::Vertex u, const graph::Graph &g) {
    if (compat_prop_[u] != graph::Compat::UNKNOWN) {
      return;
    }

    if (ops_.find(type_prop_[u]) != ops_.end()) {
      boost::put(compat_prop_, u, graph::Compat::INCOMPATIBLE);
      incompat_nodes_.push_back(u);
    } else {
      boost::put(compat_prop_, u, graph::Compat::COMPATIBLE);
      compat_nodes_.push_back(u);
    }
  }

  const std::unordered_set<std::string> &ops_;
  const TypeProp &type_prop_;
  CompatProp &compat_prop_;
  std::vector<graph::Vertex> &compat_nodes_;
  std::vector<graph::Vertex> &incompat_nodes_;
};

template <typename VCompatProp, typename ECompatProp>
struct edge_compat_visitor
    : public boost::base_visitor<
          edge_compat_visitor<VCompatProp, ECompatProp>> {

  edge_compat_visitor(VCompatProp &v_compat_prop, ECompatProp &e_compat_prop,
                      std::vector<graph::Edge> &c2c_edges,
                      std::vector<graph::Edge> &c2i_edges,
                      std::vector<graph::Edge> &i2c_edges,
                      std::vector<graph::Edge> &i2i_edges)
      : v_compat_prop_{v_compat_prop}, e_compat_prop_{e_compat_prop},
        c2c_edges_{c2c_edges}, c2i_edges_{c2i_edges}, i2c_edges_{i2c_edges},
        i2i_edges_{i2i_edges} {}

  typedef boost::on_examine_edge event_filter;
  void operator()(graph::Edge e, const graph::Graph &g) {
    graph::Vertex u = boost::source(e, g);
    graph::Vertex v = boost::target(e, g);

    if (v_compat_prop_[u] == graph::Compat::COMPATIBLE &&
        v_compat_prop_[v] == graph::Compat::INCOMPATIBLE) {
      boost::put(e_compat_prop_, e, graph::ETypes::C2I);
      c2i_edges_.push_back(e);
    } else if (v_compat_prop_[u] == graph::Compat::INCOMPATIBLE &&
               v_compat_prop_[v] == graph::Compat::COMPATIBLE) {
      boost::put(e_compat_prop_, e, graph::ETypes::I2C);
      i2c_edges_.push_back(e);
    } else if (v_compat_prop_[u] == graph::Compat::COMPATIBLE &&
               v_compat_prop_[v] == graph::Compat::COMPATIBLE) {
      boost::put(e_compat_prop_, e, graph::ETypes::C2C);
      c2c_edges_.push_back(e);
    } else if (v_compat_prop_[u] == graph::Compat::INCOMPATIBLE &&
               v_compat_prop_[v] == graph::Compat::INCOMPATIBLE) {
      boost::put(e_compat_prop_, e, graph::ETypes::I2I);
      i2i_edges_.push_back(e);
    }
  }

  VCompatProp &v_compat_prop_;
  ECompatProp &e_compat_prop_;
  std::vector<graph::Edge> &c2c_edges_;
  std::vector<graph::Edge> &c2i_edges_;
  std::vector<graph::Edge> &i2c_edges_;
  std::vector<graph::Edge> &i2i_edges_;
};

template <typename CompatProp, typename SubgraphProp>
struct subgraph_visitor_edges
    : public boost::base_visitor<
          subgraph_visitor_edges<CompatProp, SubgraphProp>> {
  subgraph_visitor_edges(
      std::map<int, std::vector<graph::Vertex>> &subgraph_map,
      const CompatProp &compat_prop, SubgraphProp &subgraph_prop)
      : subgraph_map_{subgraph_map}, compat_prop_{compat_prop},
        subgraph_prop_{subgraph_prop} {};

  typedef boost::on_tree_edge event_filter;
  void operator()(graph::Edge e, const graph::Graph &g) {
    graph::Vertex u = boost::source(e, g);
    graph::Vertex v = boost::target(e, g);
    if (compat_prop_[u] != compat_prop_[v]) {
      return;
    }

    if (subgraph_prop_[v] == subgraph_prop_[u]) {
      return;
    }

    std::vector<graph::Vertex> &v_subgraph = subgraph_map_[subgraph_prop_[v]];
    std::vector<graph::Vertex> &u_subgraph = subgraph_map_[subgraph_prop_[u]];
    for (auto it = v_subgraph.begin(); it != v_subgraph.end(); it++) {
      u_subgraph.push_back(*it);
      boost::put(subgraph_prop_, *it, subgraph_prop_[u]);
    }
    v_subgraph.clear();
  }
  std::map<int, std::vector<graph::Vertex>> &subgraph_map_;
  const CompatProp &compat_prop_;
  SubgraphProp &subgraph_prop_;
};

template <typename CompatProp, typename SubgraphProp>
struct subgraph_visitor_initialize
    : public boost::base_visitor<
          subgraph_visitor_initialize<CompatProp, SubgraphProp>> {
  subgraph_visitor_initialize(
      std::map<int, std::vector<graph::Vertex>> &subgraph_map,
      const CompatProp &compat_prop, SubgraphProp &subgraph_prop)
      : subgraph_map_{subgraph_map}, compat_prop_{compat_prop},
        subgraph_prop_{subgraph_prop} {};

  typedef boost::on_initialize_vertex event_filter;
  void operator()(graph::Vertex u, const graph::Graph &g) {
    boost::put(subgraph_prop_, u, u);
    subgraph_map_[u].push_back(u);
  }

  std::map<int, std::vector<graph::Vertex>> &subgraph_map_;
  const CompatProp &compat_prop_;
  SubgraphProp &subgraph_prop_;
};

struct record_predecessors : public boost::base_visitor<record_predecessors> {
  record_predecessors(
      std::map<graph::Vertex, std::vector<graph::Vertex>> &predecessors)
      : predecessors_{predecessors} {};

  typedef boost::on_tree_edge event_filter;
  void operator()(graph::Edge e, const graph::Graph &g) {
    graph::Vertex u = boost::source(e, g);
    graph::Vertex v = boost::target(e, g);

    auto &preds_u = predecessors_[u];
    auto &preds_v = predecessors_[v];

    std::copy(preds_u.begin(), preds_u.end(), std::back_inserter(preds_v));

    predecessors_[v].push_back(u);
  }

  std::map<graph::Vertex, std::vector<graph::Vertex>> &predecessors_;
};

template <typename Subgraph>
struct solve_cross_edges
    : public boost::base_visitor<solve_cross_edges<Subgraph>> {
  solve_cross_edges(
      std::map<graph::Vertex, std::vector<graph::Vertex>> &predecessors,
      std::map<int, std::vector<graph::Vertex>> &subgraphs,
      Subgraph &subgraph_prop)
      : predecessors_{predecessors}, subgraphs_{subgraphs},
        subgraph_prop_{subgraph_prop} {};

  typedef boost::on_forward_or_cross_edge event_filter;
  void operator()(graph::Edge e, const graph::Graph &g) {
    graph::Vertex u = boost::source(e, g);
    graph::Vertex v = boost::target(e, g);

    auto &preds_u = predecessors_[v];
    auto &preds_v = predecessors_[v];
    if (std::find(preds_v.begin(), preds_v.end(), u) != preds_v.end()) {
      std::cout << "forward edge: (" << u << " -> " << v << ")" << std::endl;
    } else {
      std::cout << "cross edge: (" << u << " -> " << v << ")";
      // Check for common ancestors
      std::vector<graph::Vertex> intersection;
      std::set_intersection(preds_u.begin(), preds_u.end(), preds_v.begin(),
                            preds_v.end(), std::back_inserter(intersection));

      std::cout << " with " << intersection.size() << " common predecessors.";
      std::cout << std::endl;

      // Add new subgraph
      int current_size = subgraphs_.size();
      std::cout << "moving " << v << " to subgraph " << current_size
                << std::endl;
      // subgraphs_[current_size] = std::vector<graph::Vertex>();
      // subgraphs_.emplace(current_size, std::vector<graph::Vertex>());
      subgraphs_[current_size].push_back(v);

      int old_subgraph_idx = subgraph_prop_[v];
      std::cout << "removing " << v << " from subgraph " << old_subgraph_idx
                << std::endl;
      auto &old_subgraph = subgraphs_[old_subgraph_idx];
      old_subgraph.erase(
          std::remove(old_subgraph.begin(), old_subgraph.end(), v));

      boost::put(subgraph_prop_, v, current_size);
    }
  }
  std::map<graph::Vertex, std::vector<graph::Vertex>> &predecessors_;
  std::map<int, std::vector<graph::Vertex>> &subgraphs_;
  Subgraph &subgraph_prop_;
};

template <typename VSubgraph>
void merge_subgraphs(int A_idx, graph::Subgraph &A, int B_idx,
                     graph::Subgraph &B, VSubgraph subgraph_prop) {
  for (auto it = A.nodes.begin(); it != A.nodes.end(); it++) {
    graph::Vertex u = *it;
    boost::put(subgraph_prop, u, B_idx);
  }

  std::move(A.nodes.begin(), A.nodes.end(), std::back_inserter(B.nodes));
  std::move(A.output_edges.begin(), A.output_edges.end(),
            std::back_inserter(B.interior_edges));
  std::move(A.interior_edges.begin(), A.interior_edges.end(),
            std::back_inserter(B.interior_edges));
}

struct cycles_visitor : public boost::base_visitor<cycles_visitor> {
  cycles_visitor(bool *out) : out_{out} { *out_ = false; }
  typedef boost::on_back_edge event_filter;
  void operator()(graph::Edge e, const graph::Graph &g) {
    std::cout << "Cycle found!!!" << std::endl;
    *out_ |= true;
  }

  bool *out_;
};

template <typename Subgraph>
struct record_cycles_visitor
    : public boost::base_visitor<record_cycles_visitor<Subgraph>> {
  record_cycles_visitor(std::vector<graph::Edge> &out, Subgraph &subgraph_prop)
      : out_{out}, subgraph_prop_{subgraph_prop} {}

  typedef boost::on_back_edge event_filter;
  // typedef boost::on_forward_or_cross_edge event_filter;
  void operator()(graph::Edge e, const graph::Graph &g) {
    graph::Vertex u = boost::source(e, g);
    graph::Vertex v = boost::target(e, g);
    std::cout << "Cycle found : " << e << " subgraphs " << subgraph_prop_[u]
              << " -> " << subgraph_prop_[v] << std::endl;
    out_.push_back(e);
  }

  std::vector<graph::Edge> &out_;
  Subgraph &subgraph_prop_;
};

} // namespace partition::algorithms::detail

#endif // ALGO_UNSUPPORTED_DETAIL_HPP
