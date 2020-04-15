#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <string>
#include <vector>
#include <functional>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/subgraph.hpp>

namespace boost {
  enum vertex_node_type_t {
			   vertex_node_type
  };

  enum vertex_compat_t {
			   vertex_compat
  };

  enum vertex_subgraph_index_t {
			   vertex_subgraph_index
  };

  enum edge_type_t {
		    edge_type
  };

  BOOST_INSTALL_PROPERTY(vertex, node_type);
  BOOST_INSTALL_PROPERTY(vertex, compat);
  BOOST_INSTALL_PROPERTY(vertex, subgraph_index);
  BOOST_INSTALL_PROPERTY(edge, type);
}  // boost

namespace partition {
namespace graph {
  enum class Compat {
		     UNKNOWN,
		     COMPATIBLE,
		     INCOMPATIBLE,
		     BOUNDARY,
		     DATA
  };

  enum class ETypes {
		     UNKNOWN,
		     C2C,
		     I2I,
		     C2I,
		     I2C
  };

  using vertex_node_type_t = boost::vertex_node_type_t;
  using vertex_name_t = boost::vertex_name_t;
  using vertex_compat_t = boost::vertex_compat_t;
  using vertex_subgraph_index_t = boost::vertex_subgraph_index_t;
  using vertex_index_t = boost::vertex_index_t;
  using edge_type_t = boost::edge_type_t;
  using edge_index_t = boost::edge_index_t;
  using edge_name_t = boost::edge_name_t;

  using VertexProperty = boost::property<vertex_name_t, std::string,
		      	 boost::property<vertex_node_type_t, std::string,
		         boost::property<vertex_compat_t, Compat,
			 boost::property<vertex_subgraph_index_t, int,
			 boost::property<boost::vertex_index_t, std::size_t>>>>>;

  using EdgeProperty = boost::property<edge_type_t, ETypes,
		       boost::property<boost::edge_index_t, std::size_t,
		       boost::property<edge_name_t, std::string>>>;

  using GraphProperty = boost::property<boost::graph_name_t, std::string>;

  using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
  				      VertexProperty,
  				      EdgeProperty,
  				      GraphProperty>;

  using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
  using Edge = boost::graph_traits<Graph>::edge_descriptor;

  struct Subgraph {
    std::vector<Vertex> nodes;
    std::vector<Edge> input_edges;
    std::vector<Edge> output_edges;
    std::vector<Edge> interior_edges;
  };

  using VertexNameProperty = boost::property_map<graph::Graph, graph::vertex_name_t>::type;
  using VertexCompatProperty = boost::property_map<graph::Graph, graph::vertex_compat_t>::type;
  using VertexTypeProperty = boost::property_map<graph::Graph, graph::vertex_node_type_t>::type;
  using VertexSubgraphIndexProperty = boost::property_map<graph::Graph, graph::vertex_subgraph_index_t>::type;
  using VertexIndexProperty = boost::property_map<graph::Graph, graph::vertex_index_t>::type;
  using EdgeTypeProperty = boost::property_map<graph::Graph, graph::edge_type_t>::type;
  using EdgeIndexProperty = boost::property_map<graph::Graph, graph::edge_index_t>::type;
  using EdgeNameProperty = boost::property_map<graph::Graph, graph::edge_name_t>::type;

  using VertexIterator = boost::graph_traits<graph::Graph>::vertex_iterator;
  using OutEdgeIterator = boost::graph_traits<graph::Graph>::out_edge_iterator;

  extern vertex_node_type_t vertex_node_type;
  extern vertex_name_t vertex_name;
  extern vertex_compat_t vertex_compat;
  extern vertex_subgraph_index_t vertex_subgraph_index;
  extern vertex_index_t vertex_index;
  extern edge_type_t edge_type;
  extern edge_index_t edge_index;
  extern edge_name_t edge_name;

} // graph
} // partition

#endif // GRAPH_HPP
