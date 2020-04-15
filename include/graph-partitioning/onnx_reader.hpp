#ifndef ONNX_READER_HPP
#define ONNX_READER_HPP

#include <string>

#include <graph-partitioning/graph.hpp>


namespace partition {
namespace readers {

  class ONNXReader {
  public:
    static graph::Graph from_bin(std::string path);

  }; // ONNXReader

} // readers
} // partition

#endif // ONNX_READER_HPP
