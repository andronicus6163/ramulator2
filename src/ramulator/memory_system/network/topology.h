#ifndef RAMULATOR_MEMORY_SYSTEM_NETWORK_TOPOLOGY_H
#define RAMULATOR_MEMORY_SYSTEM_NETWORK_TOPOLOGY_H

#include <string>
#include <vector>

namespace Ramulator {

// Memory-network topology in the MultiPIM XML format (memnodes / interconnection / memroutes).
struct MemoryTopology {
  int num_nodes = 0;
  int links_per_node = 0;
  std::vector<std::vector<bool>> to_cpu;               // [node][local link]
  std::vector<int> interconnections;                   // global link -> global link, -1 if none
  std::vector<std::vector<std::vector<int>>> routes;   // [src][dst] -> next global links
  std::vector<std::vector<int>> cpu_routes;            // [dst] -> source-mode global links

  static MemoryTopology from_xml_file(const std::string& path);

  int cpu_route_link(int dst) const;
  int route_link(int src, int dst) const;
  int source_mode_links(int node) const;
  int total_source_mode_links() const;
};

}  // namespace Ramulator

#endif
