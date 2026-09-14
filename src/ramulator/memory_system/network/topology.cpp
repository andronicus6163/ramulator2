#include "ramulator/memory_system/network/topology.h"

#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <stdexcept>

#include <fmt/format.h>

namespace Ramulator {

namespace {

std::map<std::string, std::string> parse_attrs(const std::string& text) {
  static const std::regex attr_re(R"re(([A-Za-z_][\w\-]*)\s*=\s*"([^"]*)")re");
  std::map<std::string, std::string> attrs;
  for (auto it = std::sregex_iterator(text.begin(), text.end(), attr_re); it != std::sregex_iterator(); ++it) {
    attrs[(*it)[1]] = (*it)[2];
  }
  return attrs;
}

const std::string& require(const std::map<std::string, std::string>& attrs, const std::string& key,
                           const std::string& tag) {
  auto it = attrs.find(key);
  if (it == attrs.end()) {
    throw std::runtime_error(fmt::format("Topology: <{}> is missing attribute '{}'", tag, key));
  }
  return it->second;
}

std::vector<int> parse_list(const std::string& s) {
  std::vector<int> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) out.push_back(std::stoi(item));
  }
  return out;
}

}  // namespace

MemoryTopology MemoryTopology::from_xml_file(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error(fmt::format("Topology: cannot open '{}'", path));
  }
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  std::string xml = std::regex_replace(buffer.str(), std::regex(R"(<!--[\s\S]*?-->)"), "");

  MemoryTopology t;
  int cur_node = -1;
  auto check_link = [&](int link, const char* tag) {
    if (link < 0 || link >= t.num_nodes * t.links_per_node) {
      throw std::runtime_error(fmt::format("Topology: <{}> link {} out of range", tag, link));
    }
  };

  static const std::regex tag_re(R"(<\s*([A-Za-z_]\w*)([^>]*)>)");
  for (auto it = std::sregex_iterator(xml.begin(), xml.end(), tag_re); it != std::sregex_iterator(); ++it) {
    std::string name = (*it)[1];
    auto attrs = parse_attrs((*it)[2]);
    if (name == "memnodes") {
      t.num_nodes = std::stoi(require(attrs, "num", name));
      t.links_per_node = std::stoi(require(attrs, "linkspernode", name));
      if (t.num_nodes <= 0 || t.links_per_node <= 0) {
        throw std::runtime_error("Topology: memnodes num and linkspernode must be positive");
      }
      t.to_cpu.assign(t.num_nodes, std::vector<bool>(t.links_per_node, false));
      t.interconnections.assign(t.num_nodes * t.links_per_node, -1);
    } else if (name == "node") {
      cur_node = std::stoi(require(attrs, "id", name));
    } else if (name == "link") {
      if (attrs.count("tocpu") && attrs["tocpu"] == "true") {
        int link = std::stoi(require(attrs, "id", name));
        check_link(link, "link");
        t.to_cpu[cur_node][link % t.links_per_node] = true;
      }
    } else if (name == "interconnection") {
      int from = std::stoi(require(attrs, "from", name));
      int to = std::stoi(require(attrs, "to", name));
      check_link(from, "interconnection");
      check_link(to, "interconnection");
      t.interconnections[from] = to;
      if (!attrs.count("type") || attrs["type"] == "undirected") {
        t.interconnections[to] = from;
      }
    } else if (name == "memroutes") {
      if (require(attrs, "type", name) != "static") {
        throw std::runtime_error("Topology: only static memroutes are supported");
      }
      t.routes.assign(t.num_nodes, std::vector<std::vector<int>>(t.num_nodes));
      t.cpu_routes.assign(t.num_nodes, {});
    } else if (name == "route") {
      int src = std::stoi(require(attrs, "src", name));
      int dst = std::stoi(require(attrs, "dst", name));
      for (int next : parse_list(require(attrs, "next", name))) {
        if (next < src * t.links_per_node || next >= (src + 1) * t.links_per_node) {
          throw std::runtime_error(fmt::format("Topology: route {}->{} next link {} is not on node {}", src, dst, next, src));
        }
        t.routes[src][dst].push_back(next);
      }
    } else if (name == "cpuroute") {
      int dst = std::stoi(require(attrs, "dst", name));
      for (int next : parse_list(require(attrs, "next", name))) {
        check_link(next, "cpuroute");
        if (!t.to_cpu[next / t.links_per_node][next % t.links_per_node]) {
          throw std::runtime_error(fmt::format("Topology: cpuroute to {} uses non-CPU link {}", dst, next));
        }
        t.cpu_routes[dst].push_back(next);
      }
    }
  }

  if (t.num_nodes == 0) {
    throw std::runtime_error(fmt::format("Topology: '{}' has no <memnodes>", path));
  }
  if (t.routes.empty()) {
    t.routes.assign(t.num_nodes, std::vector<std::vector<int>>(t.num_nodes));
    t.cpu_routes.assign(t.num_nodes, {});
  }
  return t;
}

int MemoryTopology::cpu_route_link(int dst) const {
  return cpu_routes.at(dst).empty() ? -1 : cpu_routes[dst][0];
}

int MemoryTopology::route_link(int src, int dst) const {
  return routes.at(src).at(dst).empty() ? -1 : routes[src][dst][0];
}

int MemoryTopology::source_mode_links(int node) const {
  int n = 0;
  for (bool b : to_cpu.at(node)) n += b;
  return n;
}

int MemoryTopology::total_source_mode_links() const {
  int n = 0;
  for (int i = 0; i < num_nodes; i++) n += source_mode_links(i);
  return n;
}

}  // namespace Ramulator
