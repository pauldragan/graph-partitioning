#include <iostream>
#include <string>
#include <vector>

#include <toml.hpp>

#include <graph-partitioning/graph.hpp>
#include <graph-partitioning/utils.hpp>

namespace partition::utils {

Config parse_config_file(std::string path) {
  Config conf;

  const auto data = toml::parse(path);

  conf.model_path = toml::find<std::string>(data, "model_path");
  conf.model_name = toml::find<std::string>(data, "model_name");
  conf.output_dir = toml::find<std::string>(data, "output_dir");
  conf.model_type = toml::find<std::string>(data, "model_type");
  conf.unsupported_ops =
      toml::find<std::vector<std::string>>(data, "unsupported_ops");
  conf.data_ops = toml::find<std::vector<std::string>>(data, "data_ops");
  conf.identity_ops =
      toml::find<std::vector<std::string>>(data, "identity_ops");

  return conf;
}

} // namespace partition::utils
