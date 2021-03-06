#ifndef ONNX_WRITER_HPP
#define ONNX_WRITER_HPP

#include <map>
#include <string>
#include <vector>

#include <graph-partitioning/graph.hpp>

namespace partition {
namespace writers {

class ONNXWriter {
public:
  static void to_bin(std::string orig_path, std::string out_dir,
                     graph::Graph &g_partitioned,
                     std::map<int, graph::Subgraph> &subgraphs);

}; // ONNXWriter

} // namespace writers
} // namespace partition

#endif // ONNX_WRITER_HPP
