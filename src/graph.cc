#include <graph-partitioning/graph.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>

namespace partition {
namespace graph {

vertex_node_type_t vertex_node_type = boost::vertex_node_type;
vertex_name_t vertex_name = boost::vertex_name;
vertex_compat_t vertex_compat = boost::vertex_compat;
vertex_subgraph_index_t vertex_subgraph_index = boost::vertex_subgraph_index;
vertex_index_t vertex_index = boost::vertex_index;
edge_type_t edge_type = boost::edge_type;
edge_index_t edge_index = boost::edge_index;
edge_name_t edge_name = boost::edge_name;

} // graph
} // partition
