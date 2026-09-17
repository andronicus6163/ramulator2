#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include <fmt/format.h>

#include "ramulator/base/param.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/i_controller.h"
#include "ramulator/memory_system/i_memory_system.h"
#include "ramulator/memory_system/network/booksim_interconnect.h"
#include "ramulator/memory_system/network/hmc_network_queries.h"
#include "ramulator/memory_system/network/topology.h"

namespace Ramulator {

// Multi-stack Hybrid Memory Cube network (MultiPIM model): per-stack logic layer with
// source-mode host links, inter-stack links with token flow control, and a Booksim2
// crossbar switch between links and vaults. PIM cores sit on the vaults (source_id = global vault).
class HMCNetworkMemorySystem final : public IMemorySystem, public Implementation, public IHMCNetworkQueries {
  RAMULATOR_REGISTER_IMPLEMENTATION(IMemorySystem, HMCNetworkMemorySystem, "HMCNetwork");

 private:
  enum class Addressing { RoChBgBaCuVaCl, RoBgBaCuVaChCl, RoBgBaChCuVaCl, RoChBaBgCuVaCl, CuVaRoChBgBaCl };
  enum IdealType { IdealLocalVault = 0, IdealLocalStack = 1 };

  struct Txn {
    Request req;
    int vault = -1;
    int pu_id = -1;
    int req_hops = 0;
    int resp_hops = 0;
    Clk_t arrive_hmc = 0;
    Clk_t arrive_vault = 0;
    Clk_t depart_vault = 0;
  };

  struct Packet {
    std::shared_ptr<Txn> txn;
    bool is_request = true;
    bool flow_control = false;
    int total_flits = 1;
    int rtc = 0;
    int cub = 0;
    int slid = -1;
    int tag = -1;
  };

  struct Link {
    int id = 0;
    int cub = 0;
    int device = 0;
    bool source_mode = false;
    Link* other = nullptr;
    std::deque<Packet> out;
    int out_occupied = 0;
    int tokens = 0;
    Clk_t next_packet_clk = 0;
    std::deque<Packet> in;
    int in_occupied = 0;
    int extracted_tokens = 0;
  };

  struct Vault {
    std::deque<Packet> responses;
    std::deque<Packet> rmab;
    bool next_response = true;
  };

  std::vector<IController*> m_controllers;
  unsigned int m_clock_ratio = 1;
  int m_stacks = 1;
  int m_stack_links = 4;
  int m_link_width = 16;
  float m_lane_speed_gbps = 30.0f;
  int m_payload_flits = 4;
  int m_max_block_bits = 7;
  int m_link_buffer = 32;
  int m_max_tags = 2048;
  int m_rmab_size = 32;
  bool m_quadrant = true;
  int m_ideal_memnet_type = IdealLocalVault;
  std::string m_topology_file;
  std::string m_switch_config;
  std::string m_addressing_str;
  Addressing m_addressing = Addressing::RoChBgBaCuVaCl;

  MemoryTopology m_topology;
  Clk_t m_clk = 0;
  int m_vpc = 0;
  int m_vpc_bits = 0;
  int m_cub_bits = 0;
  int m_tx_bytes = 0;
  int m_tx_bits = 0;
  int m_flit_bytes = 16;
  int m_burst_count = 1;
  double m_one_flit_cycles = 0;
  uint64_t m_capacity = 0;
  int m_bankgroup_bits = 0, m_bank_bits = 0, m_row_bits = 0, m_column_bits = 0;
  int m_bankgroup_level = -1, m_bank_level = -1, m_row_level = -1, m_column_level = -1, m_level_count = 0;

  std::vector<Link> m_links;
  std::vector<std::vector<Link*>> m_stack_link_order;
  std::vector<std::deque<int>> m_tags;
  std::vector<Vault> m_vaults;

  size_t s_num_read_requests = 0;
  size_t s_num_write_requests = 0;
  size_t s_rejected_link_full = 0;
  size_t s_rejected_no_tag = 0;
  size_t s_rejected_vault_full = 0;
  size_t s_rejected_rmab_full = 0;
  size_t s_completed_requests = 0;
  size_t s_memory_access_latency = 0;
  size_t s_request_packet_latency = 0;
  size_t s_response_packet_latency = 0;
  size_t s_pu_local_vault_requests = 0;
  size_t s_pu_local_memory_requests = 0;
  size_t s_pu_remote_memory_requests = 0;
  size_t s_ptw_requests = 0;
  std::vector<size_t> s_requests_per_vault;
  std::vector<size_t> s_accesses_by_hops;
  std::vector<size_t> s_switch_injected_packets;
  std::vector<size_t> s_switch_ejected_packets;
  std::vector<size_t> s_link_input_occupancy_sum;
  std::vector<size_t> s_link_output_occupancy_sum;
  std::vector<size_t> s_rmab_length_sum;

 public:
  void init() override {
    RAMULATOR_PARSE_PARAM(m_clock_ratio, unsigned int, "clock_ratio").required();
    RAMULATOR_PARSE_PARAM(m_stacks, int, "stacks").default_val(1);
    RAMULATOR_PARSE_PARAM(m_stack_links, int, "stack_links").default_val(4);
    RAMULATOR_PARSE_PARAM(m_link_width, int, "link_width").default_val(16);
    RAMULATOR_PARSE_PARAM(m_lane_speed_gbps, float, "lane_speed_gbps").default_val(30.0f);
    RAMULATOR_PARSE_PARAM(m_payload_flits, int, "payload_flits").default_val(4);
    RAMULATOR_PARSE_PARAM(m_max_block_bits, int, "max_block_bits").default_val(7);
    RAMULATOR_PARSE_PARAM(m_link_buffer, int, "link_buffer").default_val(32);
    RAMULATOR_PARSE_PARAM(m_max_tags, int, "max_tags").default_val(2048);
    RAMULATOR_PARSE_PARAM(m_rmab_size, int, "rmab_size").default_val(32);
    RAMULATOR_PARSE_PARAM(m_quadrant, bool, "quadrant").default_val(true);
    RAMULATOR_PARSE_PARAM(m_ideal_memnet_type, int, "ideal_memnet_type").default_val(0);
    RAMULATOR_PARSE_PARAM(m_topology_file, std::string, "topology_file").required();
    RAMULATOR_PARSE_PARAM(m_switch_config, std::string, "switch_config").required();
    RAMULATOR_PARSE_PARAM(m_addressing_str, std::string, "addressing").default_val("RoChBgBaCuVaCl");
    RAMULATOR_CREATE_CHILD_LIST(m_controllers, IController);

    if (m_addressing_str == "RoChBgBaCuVaCl") {
      m_addressing = Addressing::RoChBgBaCuVaCl;
    } else if (m_addressing_str == "RoBgBaCuVaChCl") {
      m_addressing = Addressing::RoBgBaCuVaChCl;
    } else if (m_addressing_str == "RoBgBaChCuVaCl") {
      m_addressing = Addressing::RoBgBaChCuVaCl;
    } else if (m_addressing_str == "RoChBaBgCuVaCl") {
      m_addressing = Addressing::RoChBaBgCuVaCl;
    } else if (m_addressing_str == "CuVaRoChBgBaCl") {
      // cube and vault in the most significant bits: every vault owns one contiguous physical range
      m_addressing = Addressing::CuVaRoChBgBaCl;
    } else {
      throw std::runtime_error(fmt::format("HMCNetwork: unknown addressing '{}'", m_addressing_str));
    }

    int vaults = static_cast<int>(m_controllers.size());
    if (m_stacks <= 0 || (m_stacks & (m_stacks - 1)) != 0) {
      throw std::runtime_error(fmt::format("HMCNetwork: stacks must be a power of two, got {}", m_stacks));
    }
    if (vaults == 0 || vaults % m_stacks != 0) {
      throw std::runtime_error(fmt::format("HMCNetwork: {} controllers cannot be split over {} stacks", vaults, m_stacks));
    }
    m_vpc = vaults / m_stacks;
    if ((m_vpc & (m_vpc - 1)) != 0) {
      throw std::runtime_error(fmt::format("HMCNetwork: vaults per stack must be a power of two, got {}", m_vpc));
    }
    if (m_quadrant && m_vpc % m_stack_links != 0) {
      throw std::runtime_error("HMCNetwork: quadrant routing needs vaults per stack divisible by stack_links");
    }
    m_vpc_bits = log2i(m_vpc);
    m_cub_bits = log2i(m_stacks);

    for (int i = 0; i < vaults; i++) {
      dynamic_cast<Implementation*>(m_controllers[i])->set_id(fmt::format("Vault {}", i));
      m_controllers[i]->set_channel_id(i);
      m_controllers[i]->m_clock_ratio = m_clock_ratio;
    }

    auto* ctrl = dynamic_cast<ControllerBase*>(m_controllers[0]);
    if (!ctrl) {
      throw std::runtime_error("HMCNetwork: vault controllers must derive from ControllerBase");
    }
    const DRAMSpec* spec = ctrl->m_device.m_spec;
    m_level_count = spec->level_count;
    m_bankgroup_level = spec->get_level_id("BankGroup");
    m_bank_level = spec->get_level_id("Bank");
    m_row_level = spec->get_level_id("Row");
    m_column_level = spec->get_level_id("Column");
    m_tx_bytes = spec->get_tx_bytes();
    m_tx_bits = log2i(m_tx_bytes);
    m_bankgroup_bits = log2i(spec->get_level_size("BankGroup"));
    m_bank_bits = log2i(spec->get_level_size("Bank"));
    m_row_bits = log2i(spec->get_level_size("Row"));
    m_column_bits = log2i(spec->get_level_size("Column")) - log2i(spec->internal_prefetch_size);

    m_capacity = static_cast<uint64_t>(spec->channel_width / 8) * vaults * spec->get_level_size("BankGroup") *
                 spec->get_level_size("Bank") * spec->get_level_size("Row") * spec->get_level_size("Column");
    m_burst_count = static_cast<int>(std::ceil(m_payload_flits / (m_tx_bytes / m_flit_bytes)));
    m_one_flit_cycles = (m_flit_bytes * 8.0 / (m_lane_speed_gbps * m_link_width)) / ctrl->get_tCK();

    m_topology = MemoryTopology::from_xml_file(m_topology_file);
    if (m_topology.num_nodes != m_stacks || m_topology.links_per_node != m_stack_links) {
      throw std::runtime_error(fmt::format("HMCNetwork: topology has {} nodes x {} links, config has {} stacks x {} links",
                                           m_topology.num_nodes, m_topology.links_per_node, m_stacks, m_stack_links));
    }
    build_links();

    Booksim::init(m_stacks, m_switch_config.c_str(), m_stack_links, m_vpc);

    m_vaults.resize(vaults);
    s_requests_per_vault.assign(vaults, 0);
    s_accesses_by_hops.assign(2 * m_stacks + 2, 0);
    s_switch_injected_packets.assign(m_stacks, 0);
    s_switch_ejected_packets.assign(m_stacks, 0);
    s_link_input_occupancy_sum.assign(m_stacks, 0);
    s_link_output_occupancy_sum.assign(m_stacks, 0);
    s_rmab_length_sum.assign(m_stacks, 0);

    m_stats.add("total_num_read_requests", s_num_read_requests);
    m_stats.add("total_num_write_requests", s_num_write_requests);
    m_stats.add("rejected_link_full", s_rejected_link_full);
    m_stats.add("rejected_no_tag", s_rejected_no_tag);
    m_stats.add("rejected_vault_full", s_rejected_vault_full);
    m_stats.add("rejected_rmab_full", s_rejected_rmab_full);
    m_stats.add("completed_requests", s_completed_requests);
    m_stats.add("memory_access_latency", s_memory_access_latency);
    m_stats.add("request_packet_latency", s_request_packet_latency);
    m_stats.add("response_packet_latency", s_response_packet_latency);
    m_stats.add("pu_local_vault_requests", s_pu_local_vault_requests);
    m_stats.add("pu_local_memory_requests", s_pu_local_memory_requests);
    m_stats.add("pu_remote_memory_requests", s_pu_remote_memory_requests);
    m_stats.add("ptw_requests", s_ptw_requests);
    m_stats.add("requests_per_vault", s_requests_per_vault);
    m_stats.add("accesses_by_hops", s_accesses_by_hops);
    m_stats.add("switch_injected_packets", s_switch_injected_packets);
    m_stats.add("switch_ejected_packets", s_switch_ejected_packets);
    m_stats.add("link_input_occupancy_sum", s_link_input_occupancy_sum);
    m_stats.add("link_output_occupancy_sum", s_link_output_occupancy_sum);
    m_stats.add("rmab_length_sum", s_rmab_length_sum);
  }

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
  }

  bool send(Request& req) override {
    if (req.size_bytes <= 0 || req.size_bytes > m_tx_bytes) {
      throw std::runtime_error(fmt::format(
          "Request size_bytes must be set by the frontend (got {}, tx_bytes = {}).", req.size_bytes, m_tx_bytes));
    }
    if (req.type_id != Request::Type::Read && req.type_id != Request::Type::Write) {
      throw std::runtime_error(fmt::format("HMCNetwork: unsupported request type {}", req.type_id));
    }

    auto txn = std::make_shared<Txn>();
    txn->vault = map_address(req.addr, req.addr_vec);
    req.hops = 0;
    txn->req = req;
    txn->arrive_hmc = m_clk;
    int target_vault = txn->vault;
    int target_cub = target_vault >> m_vpc_bits;

    Packet packet;
    packet.txn = txn;
    packet.cub = target_cub;
    packet.total_flits = req.type_id == Request::Type::Read ? 1 : 1 + m_payload_flits;

    if (req.flags & Request::Flag::PIM) {
      bool ideal = req.flags & Request::Flag::IdealMemNet;
      int pu = req.source_id;
      if (pu < 0 || pu >= static_cast<int>(m_vaults.size())) {
        throw std::runtime_error(fmt::format("HMCNetwork: PIM source_id {} is not a vault", pu));
      }
      if (ideal && m_ideal_memnet_type == IdealLocalStack) {
        pu = (target_vault / m_vpc) * m_vpc + (pu % m_vpc);
      }
      txn->pu_id = pu;
      packet.slid = m_topology.route_link(pu >> m_vpc_bits, target_cub);

      if (target_vault == pu || (ideal && m_ideal_memnet_type == IdealLocalVault)) {
        if (!vault_receive(target_vault, packet)) {
          s_rejected_vault_full++;
          return false;
        }
        s_pu_local_vault_requests++;
      } else {
        Vault& vault = m_vaults[pu];
        if (static_cast<int>(vault.rmab.size()) >= m_rmab_size) {
          s_rejected_rmab_full++;
          return false;
        }
        vault.rmab.push_back(packet);
        if ((target_vault / m_vpc) == (pu / m_vpc)) {
          s_pu_local_memory_requests++;
        } else {
          s_pu_remote_memory_requests++;
        }
      }
    } else {
      int slid = m_topology.cpu_route_link(target_cub);
      if (slid < 0) {
        throw std::runtime_error(fmt::format("HMCNetwork: no cpuroute to stack {}", target_cub));
      }
      Link& link = m_links[slid];
      if (m_tags[slid].empty()) {
        s_rejected_no_tag++;
        return false;
      }
      if (packet.total_flits > m_link_buffer - link.in_occupied) {
        s_rejected_link_full++;
        return false;
      }
      packet.slid = slid;
      packet.tag = m_tags[slid].front();
      m_tags[slid].pop_front();
      slave_receive(link, packet);
    }

    if (req.flags & Request::Flag::PTW) {
      s_ptw_requests++;
    }
    if (req.type_id == Request::Type::Read) {
      s_num_read_requests++;
    } else {
      s_num_write_requests++;
    }
    s_requests_per_vault[target_vault]++;
    return true;
  }

  void tick() override {
    m_clk++;
    for (int cub = 0; cub < m_stacks; cub++) {
      tick_logic_layer(cub);
    }
    for (auto* controller : m_controllers) {
      controller->tick();
    }
    Booksim::advance();
  }

  int get_clock_ratio() override {
    return m_clock_ratio;
  }

  float get_tCK() override {
    return m_controllers[0]->get_tCK();
  }

  int get_tx_bytes() override {
    return m_tx_bytes;
  }

  int estimate_hops(int source_id, Addr_t addr, uint32_t flags) override {
    AddrVec_t addr_vec;
    int target_vault = map_address(addr, addr_vec);
    bool ideal = flags & Request::Flag::IdealMemNet;
    if (ideal) {
      if ((flags & Request::Flag::PIM) && target_vault == source_id) {
        return 0;
      }
      return m_ideal_memnet_type == IdealLocalStack ? 2 : 0;
    }

    int target_cub = target_vault / m_vpc;
    if (flags & Request::Flag::PIM) {
      if (target_vault == source_id) {
        return 0;
      }
      int cur_cub = source_id / m_vpc;
      if (target_cub == cur_cub) {
        return 2;
      }
      int hops = (m_quadrant && in_quadrant(*route(cur_cub, target_cub), source_id)) ? 0 : 1;
      return 2 * walk_hops(hops, cur_cub, target_cub, target_vault);
    }

    int slid = m_topology.cpu_route_link(target_cub);
    int slid_cub = slid / m_stack_links;
    if (target_cub == slid_cub) {
      return (m_quadrant && in_quadrant(m_links[slid], target_vault)) ? 0 : 2;
    }
    return 2 * walk_hops(1, slid_cub, target_cub, target_vault);
  }

  int access_position(int source_id, Addr_t addr, uint32_t flags) override {
    AddrVec_t addr_vec;
    int target_vault = map_address(addr, addr_vec);
    if (target_vault == source_id) {
      return 0;
    }
    if (flags & Request::Flag::IdealMemNet) {
      return m_ideal_memnet_type == IdealLocalStack ? 1 : 0;
    }
    return (target_vault / m_vpc == source_id / m_vpc) ? 1 : 2;
  }

  int target_vault(Addr_t addr) override {
    AddrVec_t addr_vec;
    return map_address(addr, addr_vec);
  }

  int target_stack(Addr_t addr) override {
    return target_vault(addr) / m_vpc;
  }

  int num_stacks() const override {
    return m_stacks;
  }

  int vaults_per_stack() const override {
    return m_vpc;
  }

  uint64_t memory_size() const override {
    return m_capacity;
  }

 private:
  static int log2i(long value) {
    int n = 0;
    while (value >>= 1) {
      n++;
    }
    return n;
  }

  static int slice(long& addr, int bits) {
    int low = static_cast<int>(addr & ((1L << bits) - 1));
    addr >>= bits;
    return low;
  }

  static int leftmost_bit(int value) {
    if (value == 0) {
      return 0;
    }
    int n = 0;
    while (value >>= 1) {
      n++;
    }
    return 1 << n;
  }

  int map_address(Addr_t full_addr, AddrVec_t& addr_vec) const {
    addr_vec.assign(m_level_count, 0);
    long addr = static_cast<long>(full_addr) >> m_tx_bits;
    int col_low_bits = m_max_block_bits - m_tx_bits;
    int column = slice(addr, col_low_bits);
    int vault_bits = m_vpc_bits + m_cub_bits;
    int column_msb_bits = m_column_bits - col_low_bits;
    int vault = 0;
    int column_msb = 0;

    switch (m_addressing) {
      case Addressing::RoChBgBaCuVaCl:
        vault = slice(addr, vault_bits);
        addr_vec[m_bank_level] = slice(addr, m_bank_bits);
        addr_vec[m_bankgroup_level] = slice(addr, m_bankgroup_bits);
        column_msb = slice(addr, column_msb_bits);
        break;
      case Addressing::RoBgBaCuVaChCl:
        column_msb = slice(addr, column_msb_bits);
        vault = slice(addr, vault_bits);
        addr_vec[m_bank_level] = slice(addr, m_bank_bits);
        addr_vec[m_bankgroup_level] = slice(addr, m_bankgroup_bits);
        break;
      case Addressing::RoBgBaChCuVaCl:
        vault = slice(addr, vault_bits);
        column_msb = slice(addr, column_msb_bits);
        addr_vec[m_bank_level] = slice(addr, m_bank_bits);
        addr_vec[m_bankgroup_level] = slice(addr, m_bankgroup_bits);
        break;
      case Addressing::RoChBaBgCuVaCl:
        vault = slice(addr, vault_bits);
        addr_vec[m_bankgroup_level] = slice(addr, m_bankgroup_bits);
        addr_vec[m_bank_level] = slice(addr, m_bank_bits);
        column_msb = slice(addr, column_msb_bits);
        break;
      case Addressing::CuVaRoChBgBaCl:
        addr_vec[m_bank_level] = slice(addr, m_bank_bits);
        addr_vec[m_bankgroup_level] = slice(addr, m_bankgroup_bits);
        column_msb = slice(addr, column_msb_bits);
        addr_vec[m_row_level] = slice(addr, m_row_bits);
        vault = slice(addr, vault_bits);
        break;
    }
    addr_vec[m_column_level] = column | (column_msb << col_low_bits);
    if (m_addressing != Addressing::CuVaRoChBgBaCl) addr_vec[m_row_level] = slice(addr, m_row_bits);
    addr_vec[0] = vault;
    return vault;
  }

  void build_links() {
    m_links.resize(m_stacks * m_stack_links);
    m_tags.resize(m_links.size());
    m_stack_link_order.assign(m_stacks, {});
    for (int cub = 0; cub < m_stacks; cub++) {
      for (int pass = 0; pass < 2; pass++) {
        for (int j = 0; j < m_stack_links; j++) {
          bool source_mode = m_topology.to_cpu[cub][j];
          if (source_mode != (pass == 0)) {
            continue;
          }
          Link& link = m_links[cub * m_stack_links + j];
          link.id = cub * m_stack_links + j;
          link.cub = cub;
          link.device = j;
          link.source_mode = source_mode;
          link.tokens = m_link_buffer;
          m_stack_link_order[cub].push_back(&link);
        }
      }
    }
    for (auto& link : m_links) {
      for (int t = 0; t < m_max_tags; t++) {
        m_tags[link.id].push_back(t);
      }
      if (!link.source_mode) {
        int neighbor = m_topology.interconnections[link.id];
        if (neighbor != -1) {
          link.other = &m_links[neighbor];
        }
      }
    }
  }

  Link* route(int cur_cub, int target_cub) {
    int id = m_topology.route_link(cur_cub, target_cub);
    if (id < 0) {
      throw std::runtime_error(fmt::format("HMCNetwork: no route from stack {} to stack {}", cur_cub, target_cub));
    }
    return &m_links[id];
  }

  bool in_quadrant(const Link& link, int vault) const {
    return ((vault % m_vpc) / (m_vpc / m_stack_links)) == link.device;
  }

  int walk_hops(int hops, int cur_cub, int target_cub, int target_vault) {
    while (cur_cub != target_cub) {
      Link* link = route(cur_cub, target_cub);
      if (!link->other) {
        throw std::runtime_error(fmt::format("HMCNetwork: route link {} is not connected", link->id));
      }
      cur_cub = link->other->cub;
      if (cur_cub != target_cub || !in_quadrant(*link->other, target_vault)) {
        hops++;
      }
    }
    return hops;
  }

  int vault_device(int vault) const {
    return m_stack_links + vault % m_vpc;
  }

  // ── Endpoints ──────────────────────────────────────────────────────

  bool vault_receive(int vault, const Packet& packet) {
    Request vreq = packet.txn->req;
    vreq.addr_vec[0] = vault;
    vreq.burst_remaining = m_burst_count;
    auto txn = packet.txn;
    int cub = packet.cub, slid = packet.slid, tag = packet.tag;
    vreq.callback = [this, vault, txn, cub, slid, tag](Request&) { on_vault_done(vault, txn, cub, slid, tag); };
    Clk_t arrive = m_clk;
    if (!m_controllers[vault]->send(vreq)) {
      return false;
    }
    txn->arrive_vault = arrive;
    return true;
  }

  void on_vault_done(int vault, const std::shared_ptr<Txn>& txn, int cub, int slid, int tag) {
    txn->depart_vault = m_clk;
    bool pim = txn->req.flags & Request::Flag::PIM;
    bool ideal = txn->req.flags & Request::Flag::IdealMemNet;
    if (pim && ((ideal && m_ideal_memnet_type == IdealLocalVault) || txn->pu_id == vault)) {
      complete(*txn);
      return;
    }
    Packet response;
    response.txn = txn;
    response.is_request = false;
    response.cub = cub;
    response.slid = slid;
    response.tag = tag;
    response.total_flits = txn->req.type_id == Request::Type::Write ? 1 : 1 + m_payload_flits;
    m_vaults[vault].responses.push_back(std::move(response));
  }

  void complete(Txn& txn) {
    s_completed_requests++;
    s_memory_access_latency += m_clk - txn.arrive_hmc;
    s_request_packet_latency += txn.arrive_vault - txn.arrive_hmc;
    s_response_packet_latency += m_clk - txn.depart_vault;
    size_t bucket = std::min<size_t>(txn.req.hops, s_accesses_by_hops.size() - 1);
    s_accesses_by_hops[bucket]++;
    if (txn.req.callback) {
      txn.req.callback(txn.req);
    }
  }

  void host_receive(Packet& packet) {
    if (packet.flow_control) {
      return;
    }
    m_tags[packet.slid].push_back(packet.tag);
    complete(*packet.txn);
  }

  // ── Links ──────────────────────────────────────────────────────────

  void slave_receive(Link& link, Packet& packet) {
    link.tokens += packet.rtc;
    if (packet.flow_control) {
      return;
    }
    link.in_occupied += packet.total_flits;
    link.in.push_back(std::move(packet));
  }

  void master_push(Link& link, Packet packet) {
    link.out_occupied += packet.total_flits;
    link.out.push_back(std::move(packet));
  }

  void tick_link_master(Link& link) {
    if (m_clk < link.next_packet_clk) {
      return;
    }
    if (!link.out.empty() && link.tokens >= link.out.front().total_flits) {
      Packet packet = std::move(link.out.front());
      link.out.pop_front();
      link.out_occupied -= packet.total_flits;
      int rtc = leftmost_bit(link.extracted_tokens);
      link.extracted_tokens -= rtc;
      packet.rtc = rtc;
      int flits = packet.total_flits;
      if (!link.source_mode) {
        link.tokens -= flits;
        if (!link.other) {
          throw std::runtime_error(fmt::format("HMCNetwork: link {} forwards a packet but is not connected", link.id));
        }
        slave_receive(*link.other, packet);
      } else {
        host_receive(packet);
      }
      link.next_packet_clk = m_clk + static_cast<Clk_t>(std::ceil(flits * m_one_flit_cycles));
      return;
    }
    if (link.extracted_tokens > 0) {
      int rtc = leftmost_bit(link.extracted_tokens);
      link.extracted_tokens -= rtc;
      if (!link.source_mode) {
        Packet tret;
        tret.flow_control = true;
        tret.rtc = rtc;
        slave_receive(*link.other, tret);
      } else {
        link.next_packet_clk = m_clk + static_cast<Clk_t>(std::ceil(m_one_flit_cycles));
      }
      return;
    }
    link.next_packet_clk = m_clk + static_cast<Clk_t>(std::ceil(m_one_flit_cycles));
  }

  // ── Switch ─────────────────────────────────────────────────────────

  void inject(int cub, int src_device, int dst_device, const Packet& packet, int size) {
    Booksim::push(cub, src_device, dst_device, new Packet(packet), size, packet.is_request,
                  packet.txn->req.type_id == Request::Type::Read);
    s_switch_injected_packets[cub]++;
  }

  void eject(int cub, int device) {
    delete static_cast<Packet*>(Booksim::pop(cub, device));
    s_switch_ejected_packets[cub]++;
  }

  void tick_logic_layer(int cub) {
    for (Link* link : m_stack_link_order[cub]) {
      s_link_input_occupancy_sum[cub] += link->in_occupied;
      s_link_output_occupancy_sum[cub] += link->out_occupied;
    }
    for (int v = cub * m_vpc; v < (cub + 1) * m_vpc; v++) {
      s_rmab_length_sum[cub] += m_vaults[v].rmab.size();
    }
    for (Link* link : m_stack_link_order[cub]) {
      tick_link_master(*link);
    }
    std::set<int> used_vaults;
    std::set<int> used_links;
    link_eject(cub, used_links);
    vault_eject(cub, used_vaults);
    vault_inject(cub, used_links);
    link_inject(cub, used_vaults);
  }

  void link_eject(int cub, std::set<int>& used_links) {
    for (Link* link : m_stack_link_order[cub]) {
      if (used_links.count(link->id)) {
        continue;
      }
      auto* packet = static_cast<Packet*>(Booksim::top(cub, link->device));
      if (!packet) {
        continue;
      }
      if (packet->total_flits <= m_link_buffer - link->out_occupied) {
        master_push(*link, *packet);
        eject(cub, link->device);
        used_links.insert(link->id);
      }
    }
  }

  void vault_eject(int cub, std::set<int>& used_vaults) {
    for (int v = cub * m_vpc; v < (cub + 1) * m_vpc; v++) {
      if (used_vaults.count(v)) {
        continue;
      }
      auto* packet = static_cast<Packet*>(Booksim::top(cub, vault_device(v)));
      if (!packet) {
        continue;
      }
      if (packet->is_request) {
        if (!vault_receive(v, *packet)) {
          continue;
        }
        eject(cub, vault_device(v));
      } else {
        auto txn = packet->txn;
        eject(cub, vault_device(v));
        complete(*txn);
      }
      used_vaults.insert(v);
    }
  }

  void vault_inject(int cub, std::set<int>& used_links) {
    int vault_start = cub * m_vpc;
    int vault_end = vault_start + m_vpc;
    for (int v = vault_start; v < vault_end; v++) {
      Vault& vault = m_vaults[v];
      bool response;
      if (vault.next_response) {
        if (!vault.responses.empty()) {
          response = true;
          vault.next_response = false;
        } else if (!vault.rmab.empty()) {
          response = false;
        } else {
          continue;
        }
      } else {
        if (!vault.rmab.empty()) {
          response = false;
          vault.next_response = true;
        } else if (!vault.responses.empty()) {
          response = true;
        } else {
          continue;
        }
      }
      std::deque<Packet>& queue = response ? vault.responses : vault.rmab;
      Packet& packet = queue.front();
      Txn& txn = *packet.txn;

      if (m_quadrant) {
        Link* shortcut = nullptr;
        if (response) {
          int source_cub = packet.slid / m_stack_links;
          if (packet.slid != -1 && source_cub == cub) {
            if (in_quadrant(m_links[packet.slid], v)) {
              shortcut = &m_links[packet.slid];
            }
          } else if (txn.pu_id < vault_start || txn.pu_id >= vault_end) {
            Link* link = route(cub, source_cub);
            if (in_quadrant(*link, v)) {
              shortcut = link;
            }
          }
        } else if (txn.vault < vault_start || txn.vault >= vault_end) {
          Link* link = route(cub, packet.cub);
          if (in_quadrant(*link, v)) {
            shortcut = link;
          }
        }
        if (shortcut) {
          if (!used_links.count(shortcut->id) && packet.total_flits <= m_link_buffer - shortcut->out_occupied) {
            txn.req.hops++;
            (response ? txn.resp_hops : txn.req_hops)++;
            master_push(*shortcut, packet);
            queue.pop_front();
            used_links.insert(shortcut->id);
          }
          continue;
        }
      }

      int size = packet.total_flits * m_flit_bytes;
      if (!Booksim::has_buffer(cub, vault_device(v), size)) {
        continue;
      }
      int target;
      if (response) {
        int source_cub = packet.slid / m_stack_links;
        if (packet.slid != -1 && source_cub == cub) {
          target = m_links[packet.slid].device;
        } else if (txn.pu_id >= vault_start && txn.pu_id < vault_end) {
          target = vault_device(txn.pu_id);
        } else {
          target = route(cub, source_cub)->device;
        }
        txn.resp_hops++;
      } else {
        if (txn.vault >= vault_start && txn.vault < vault_end) {
          target = vault_device(txn.vault);
        } else {
          target = route(cub, packet.cub)->device;
        }
        txn.req_hops++;
      }
      txn.req.hops++;
      inject(cub, vault_device(v), target, packet, size);
      queue.pop_front();
    }
  }

  void link_inject(int cub, std::set<int>& used_vaults) {
    for (Link* link : m_stack_link_order[cub]) {
      if (link->in.empty()) {
        continue;
      }
      Packet& packet = link->in.front();
      Txn& txn = *packet.txn;

      if (m_quadrant) {
        int vault = -1;
        if (packet.is_request) {
          if (packet.cub == cub && in_quadrant(*link, txn.vault)) {
            vault = txn.vault;
          }
        } else if (packet.slid / m_stack_links == cub && !m_links[packet.slid].source_mode &&
                   in_quadrant(*link, txn.pu_id)) {
          vault = txn.pu_id;
        }
        if (vault != -1) {
          if (used_vaults.count(vault)) {
            continue;
          }
          if (packet.is_request) {
            if (!vault_receive(vault, packet)) {
              continue;
            }
            txn.req_hops++;
          } else {
            txn.resp_hops++;
          }
          txn.req.hops++;
          auto owned = packet.txn;
          bool is_request = packet.is_request;
          link->extracted_tokens += packet.total_flits;
          link->in_occupied -= packet.total_flits;
          link->in.pop_front();
          used_vaults.insert(vault);
          if (!is_request) {
            complete(*owned);
          }
          continue;
        }
      }

      int size = packet.total_flits * m_flit_bytes;
      if (!Booksim::has_buffer(cub, link->device, size)) {
        continue;
      }
      int target;
      if (packet.is_request) {
        target = packet.cub == cub ? vault_device(txn.vault) : route(cub, packet.cub)->device;
        txn.req_hops++;
      } else {
        int source_cub = packet.slid / m_stack_links;
        if (source_cub == cub) {
          const Link& source = m_links[packet.slid];
          target = source.source_mode ? source.device : vault_device(txn.pu_id);
        } else {
          target = route(cub, source_cub)->device;
        }
        txn.resp_hops++;
      }
      txn.req.hops++;
      inject(cub, link->device, target, packet, size);
      link->extracted_tokens += packet.total_flits;
      link->in_occupied -= packet.total_flits;
      link->in.pop_front();
    }
  }
};

}  // namespace Ramulator
