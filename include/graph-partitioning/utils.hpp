#ifndef UTILS_HPP
#define UTILS_HPP

using namespace partition;

template <typename Label, typename Comp, typename Subgraph>
class subgraph_writer {
public:
  subgraph_writer(Label label, Comp comp, Subgraph subgraph) :
    label_(label),
    comp_(comp),
    subgraph_(subgraph)
  {};

  template <typename LabelWriter, typename EdgeWriter>
  void operator()(std::ostream& out,
		  graph::Graph& G,
		  std::map<int, graph::Subgraph>& subgraphs,
		  LabelWriter label_writer,
		  EdgeWriter edge_writer) {
    write_header(out);

    for (auto it = subgraphs.begin(); it != subgraphs.end(); it++) {
      write_subgraph(out, it->first, it->second, label_writer);
    }

    boost::graph_traits<graph::Graph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(G); ei != ei_end; ei++) {
      graph::Edge e = *ei;
      graph::Vertex u = boost::source(e, G);
      graph::Vertex v = boost::target(e, G);

      out << u << " -> " << v << " ";

      edge_writer(out, e);

      out << ";" << std::endl;
    }

    write_footer(out);
  }

  void write_header(std::ostream& out) {
    out << "digraph G {" << std::endl;
  }

  void write_footer(std::ostream& out) {
    out << "}" << std::endl;
  }

  template <typename LabelWriter>
  void write_subgraph(std::ostream& out, int id, const graph::Subgraph& vs, LabelWriter& label_writer) {
    out << "subgraph cluster_" << id << " {" << std::endl;

    out << "label = \"subgraph_" << id << "\";" << std::endl;

    for (auto it = vs.nodes.begin(); it != vs.nodes.end(); it++) {
      int u = *it;
      out << u << " ";

      label_writer(out, u);

      out << ";" << std::endl;
    }

    out << "}" << std::endl;
    out << std::endl;
  }

protected:
  Label label_;
  Comp comp_;
  Subgraph subgraph_;
};

template <typename Label, typename Comp, typename Subgraph>
class label_compat_writer {
public:
  label_compat_writer(Label label, Comp comp, Subgraph subgraph) :
    label_(label),
    comp_(comp),
    subgraph_(subgraph)
  {};
  void operator()(std::ostream& out, const graph::Vertex& u) const {
    out << "[label=\"" << label_[u];

    if (subgraph_[u] > -1) {
      out << ": " << subgraph_[u];
    }

    out << "\"" << " ";

    std::string color;

    switch (comp_[u]) {
    case graph::Compat::INCOMPATIBLE:
      color = "coral1";
      break;
    case graph::Compat::COMPATIBLE:
      color = "darkolivegreen1";
      break;
    case graph::Compat::DATA:
      color = "cornsilk";
      // color = "skyblue";
      break;
    case graph::Compat::BOUNDARY:
      color = "cornsilk";
      break;
    default:
      color = "gray100";
      break;
    }

    out << "style=filled fillcolor=" << color;

    out << "]";
  };
protected:
  Label label_;
  Comp comp_;
  Subgraph subgraph_;
};

template <typename EType, typename ELabel>
class edge_type_writer {
public:
  edge_type_writer(EType& etype, ELabel& elabel) : etype_{etype}, elabel_{elabel} {};
  void operator()(std::ostream& out, const graph::Edge& e) const {
    out << "[label=\"" << elabel_[e] << "\" ";
    switch (etype_[e]) {
    case graph::ETypes::I2C:
      out << "style=dashed color=darkolivegreen]";
      break;
    case graph::ETypes::C2I:
      out << "style=dashed color=coral3]";
      break;
    default:
      out << "]";
      break;
    }
  };
protected:
  EType& etype_;
  ELabel& elabel_;
};

#endif // UTILS_HPP
