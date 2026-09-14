#include <cmath>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include <fmt/format.h>

#include "ramulator/base/param.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/i_controller.h"
#include "ramulator/memory_system/i_memory_system.h"

namespace Ramulator {

// Single-stack Hybrid Memory Cube: host links and a logic-layer switch in front of one
// DRAM controller per vault. pim_mode bypasses the links (PIM cores on the logic layer).
class HMCMemorySystem final : public IMemorySystem, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IMemorySystem, HMCMemorySystem, "HMC");

 private:
  struct Packet {
    Request req;
    std::function<void(Request&)> host_callback;
    int flits = 1;
    int slid = 0;
    int tag = 0;
  };

  struct Link {
    std::deque<Packet> input;
    std::deque<Packet> output;
    std::deque<int> tags;
    Clk_t next_packet_clk = 0;
  };

  enum class Addressing { RoCoBaVa, RoBaCoVa, RoCoBaBgVa };

  std::vector<IController*> m_controllers;
  unsigned int m_clock_ratio = 1;
  bool m_pim_mode = false;
  bool m_network_overhead = false;
  int m_host_links = 4;
  int m_link_width = 16;
  float m_lane_speed_gbps = 30.0f;
  int m_payload_flits = 16;
  int m_max_block_bits = 8;
  int m_link_buffer = 32;
  int m_max_tags = 2048;
  int m_pim_mesh_width = 6;
  int m_pim_burst_count = 2;
  std::string m_addressing_str;
  Addressing m_addressing = Addressing::RoCoBaVa;

  Clk_t m_clk = 0;
  int m_tx_bytes = 0;
  int m_tx_bits = 0;
  int m_host_burst_count = 1;
  double m_one_flit_cycles = 0;
  long m_max_address = 0;
  int m_vault_bits = 0, m_bankgroup_bits = 0, m_bank_bits = 0, m_row_bits = 0, m_column_bits = 0;
  int m_bankgroup_level = -1, m_bank_level = -1, m_row_level = -1, m_column_level = -1, m_level_count = 0;

  std::vector<Link> m_links;
  std::vector<std::deque<Packet>> m_vault_responses;

  size_t s_num_read_requests = 0;
  size_t s_num_write_requests = 0;
  size_t s_rejected_link_full = 0;
  size_t s_rejected_no_tag = 0;
  size_t s_rejected_vault_full = 0;
  size_t s_request_packet_latency = 0;
  size_t s_response_packet_latency = 0;
  size_t s_pim_mesh_hops = 0;
  std::vector<size_t> s_requests_per_vault;

 public:
  void init() override {
    RAMULATOR_PARSE_PARAM(m_clock_ratio, unsigned int, "clock_ratio").required();
    RAMULATOR_PARSE_PARAM(m_pim_mode, bool, "pim_mode").default_val(false);
    RAMULATOR_PARSE_PARAM(m_network_overhead, bool, "network_overhead").default_val(false);
    RAMULATOR_PARSE_PARAM(m_host_links, int, "host_links").default_val(4);
    RAMULATOR_PARSE_PARAM(m_link_width, int, "link_width").default_val(16);
    RAMULATOR_PARSE_PARAM(m_lane_speed_gbps, float, "lane_speed_gbps").default_val(30.0f);
    RAMULATOR_PARSE_PARAM(m_payload_flits, int, "payload_flits").default_val(16);
    RAMULATOR_PARSE_PARAM(m_max_block_bits, int, "max_block_bits").default_val(8);
    RAMULATOR_PARSE_PARAM(m_link_buffer, int, "link_buffer").default_val(32);
    RAMULATOR_PARSE_PARAM(m_max_tags, int, "max_tags").default_val(2048);
    RAMULATOR_PARSE_PARAM(m_pim_mesh_width, int, "pim_mesh_width").default_val(6);
    RAMULATOR_PARSE_PARAM(m_pim_burst_count, int, "pim_burst_count").default_val(2);
    RAMULATOR_PARSE_PARAM(m_addressing_str, std::string, "addressing").default_val("RoCoBaVa");
    RAMULATOR_CREATE_CHILD_LIST(m_controllers, IController);

    if (m_addressing_str == "RoCoBaVa") {
      m_addressing = Addressing::RoCoBaVa;
    } else if (m_addressing_str == "RoBaCoVa") {
      m_addressing = Addressing::RoBaCoVa;
    } else if (m_addressing_str == "RoCoBaBgVa") {
      m_addressing = Addressing::RoCoBaBgVa;
    } else {
      throw std::runtime_error(fmt::format("HMC: unknown addressing '{}'", m_addressing_str));
    }

    int vaults = static_cast<int>(m_controllers.size());
    if (vaults == 0 || (vaults & (vaults - 1)) != 0) {
      throw std::runtime_error(fmt::format("HMC: vault (controller) count must be a power of two, got {}", vaults));
    }
    if (m_host_links <= 0) {
      throw std::runtime_error(fmt::format("HMC: host_links must be positive"));
    }
    for (int i = 0; i < vaults; i++) {
      dynamic_cast<Implementation*>(m_controllers[i])->set_id(fmt::format("Vault {}", i));
      m_controllers[i]->set_channel_id(i);
      m_controllers[i]->m_clock_ratio = m_clock_ratio;
    }

    auto* ctrl = dynamic_cast<ControllerBase*>(m_controllers[0]);
    if (!ctrl) {
      throw std::runtime_error(fmt::format("HMC: vault controllers must derive from ControllerBase"));
    }
    const DRAMSpec* spec = ctrl->m_device.m_spec;
    m_level_count = spec->level_count;
    m_bankgroup_level = spec->get_level_id("BankGroup");
    m_bank_level = spec->get_level_id("Bank");
    m_row_level = spec->get_level_id("Row");
    m_column_level = spec->get_level_id("Column");

    m_tx_bytes = spec->get_tx_bytes();
    m_tx_bits = log2i(m_tx_bytes);
    m_vault_bits = log2i(vaults);
    m_bankgroup_bits = log2i(spec->get_level_size("BankGroup"));
    m_bank_bits = log2i(spec->get_level_size("Bank"));
    m_row_bits = log2i(spec->get_level_size("Row"));
    m_column_bits = log2i(spec->get_level_size("Column")) - log2i(spec->internal_prefetch_size);

    long capacity = spec->channel_width / 8;
    capacity *= vaults;
    capacity *= spec->get_level_size("BankGroup");
    capacity *= spec->get_level_size("Bank");
    capacity *= spec->get_level_size("Row");
    capacity *= spec->get_level_size("Column");
    m_max_address = capacity;

    m_host_burst_count = static_cast<int>(std::ceil(m_payload_flits / ((m_tx_bytes) / 16)));
    m_one_flit_cycles = (128.0 / (m_lane_speed_gbps * m_link_width)) / ctrl->get_tCK();

    m_links.resize(m_host_links);
    for (auto& link : m_links) {
      for (int t = 0; t < m_max_tags; t++) {
        link.tags.push_back(t);
      }
    }
    m_vault_responses.resize(vaults);
    s_requests_per_vault.assign(vaults, 0);

    m_stats.add("total_num_read_requests", s_num_read_requests);
    m_stats.add("total_num_write_requests", s_num_write_requests);
    m_stats.add("rejected_link_full", s_rejected_link_full);
    m_stats.add("rejected_no_tag", s_rejected_no_tag);
    m_stats.add("rejected_vault_full", s_rejected_vault_full);
    m_stats.add("request_packet_latency", s_request_packet_latency);
    m_stats.add("response_packet_latency", s_response_packet_latency);
    m_stats.add("pim_mesh_hops", s_pim_mesh_hops);
    m_stats.add("requests_per_vault", s_requests_per_vault);
  }

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
  }

  bool send(Request& req) override {
    if (req.size_bytes <= 0 || req.size_bytes > m_tx_bytes) {
      throw std::runtime_error(fmt::format(
          "Request size_bytes must be set by the frontend (got {}, tx_bytes = {}).", req.size_bytes, m_tx_bytes));
    }

    long addr = static_cast<long>(req.addr) & (m_max_address - 1);
    req.addr = addr;
    int vault = map_address(addr, req.addr_vec);
    req.intra_channel_addr = addr;

    if (m_pim_mode) {
      int dst_x = vault / m_pim_mesh_width, dst_y = vault % m_pim_mesh_width;
      int src = req.source_id < 0 ? 0 : req.source_id;
      int src_x = src / m_pim_mesh_width, src_y = src % m_pim_mesh_width;
      int hops = m_network_overhead ? std::abs(dst_x - src_x) + std::abs(dst_y - src_y) : 0;
      req.burst_remaining = m_pim_burst_count;
      if (!m_controllers[vault]->send(req)) {
        s_rejected_vault_full++;
        return false;
      }
      s_pim_mesh_hops += hops * (req.type_id == Request::Type::Read ? 6 : 5);
      count_request(req, vault);
      return true;
    }

    int slid = static_cast<int>((addr >> m_max_block_bits) % m_host_links);
    Link& link = m_links[slid];
    if (link.tags.empty()) {
      s_rejected_no_tag++;
      return false;
    }
    int flits = req.type_id == Request::Type::Read ? 1 : 1 + m_payload_flits;
    if (flits > m_link_buffer - static_cast<int>(link.input.size())) {
      s_rejected_link_full++;
      return false;
    }

    Packet packet;
    packet.flits = flits;
    packet.slid = slid;
    packet.tag = link.tags.front();
    link.tags.pop_front();
    packet.host_callback = std::move(req.callback);
    packet.req = req;
    packet.req.arrive = m_clk;
    packet.req.burst_remaining = m_host_burst_count;
    link.input.push_back(std::move(packet));
    count_request(req, vault);
    return true;
  }

  void tick() override {
    m_clk++;
    for (auto* controller : m_controllers) {
      controller->tick();
    }
    if (m_pim_mode) {
      return;
    }
    for (auto& link : m_links) {
      tick_link_master(link);
    }
    tick_switch();
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

  int map_address(long addr, AddrVec_t& addr_vec) const {
    addr_vec.assign(m_level_count, 0);
    addr >>= m_tx_bits;
    int column_low_bits = m_max_block_bits - m_tx_bits;
    int column = slice(addr, column_low_bits);
    int vault = 0;
    int column_msb = 0;

    switch (m_addressing) {
      case Addressing::RoCoBaVa:
        vault = slice(addr, m_vault_bits);
        addr_vec[m_bank_level] = slice(addr, m_bank_bits);
        addr_vec[m_bankgroup_level] = slice(addr, m_bankgroup_bits);
        column_msb = slice(addr, m_column_bits - column_low_bits);
        break;
      case Addressing::RoBaCoVa:
        vault = slice(addr, m_vault_bits);
        column_msb = slice(addr, m_column_bits - column_low_bits);
        addr_vec[m_bank_level] = slice(addr, m_bank_bits);
        addr_vec[m_bankgroup_level] = slice(addr, m_bankgroup_bits);
        break;
      case Addressing::RoCoBaBgVa:
        vault = slice(addr, m_vault_bits);
        addr_vec[m_bankgroup_level] = slice(addr, m_bankgroup_bits);
        addr_vec[m_bank_level] = slice(addr, m_bank_bits);
        column_msb = slice(addr, m_column_bits - column_low_bits);
        break;
    }
    addr_vec[m_column_level] = column | (column_msb << column_low_bits);
    addr_vec[m_row_level] = slice(addr, m_row_bits);
    addr_vec[0] = vault;
    return vault;
  }

  void count_request(const Request& req, int vault) {
    if (req.type_id == Request::Type::Read) {
      s_num_read_requests++;
    } else if (req.type_id == Request::Type::Write) {
      s_num_write_requests++;
    }
    s_requests_per_vault[vault]++;
  }

  void tick_link_master(Link& link) {
    if (m_clk < link.next_packet_clk) {
      return;
    }
    int flits = 1;
    if (!link.output.empty()) {
      Packet packet = std::move(link.output.front());
      link.output.pop_front();
      flits = packet.flits;
      link.tags.push_back(packet.tag);
      if (packet.req.type_id == Request::Type::Read) {
        s_response_packet_latency += m_clk - packet.req.depart;
      }
      if (packet.host_callback) {
        packet.host_callback(packet.req);
      }
    }
    link.next_packet_clk = m_clk + static_cast<Clk_t>(std::ceil(flits * m_one_flit_cycles));
  }

  void tick_switch() {
    std::set<int> used_vaults;
    for (auto& link : m_links) {
      if (link.input.empty()) {
        continue;
      }
      Packet& packet = link.input.front();
      int vault = packet.req.addr_vec[0];
      if (used_vaults.count(vault)) {
        continue;
      }
      Request vault_req = packet.req;
      Clk_t hmc_arrive = packet.req.arrive;
      auto response = std::make_shared<Packet>();
      response->host_callback = std::move(packet.host_callback);
      response->slid = packet.slid;
      response->tag = packet.tag;
      response->flits = packet.req.type_id == Request::Type::Write ? 1 : 1 + m_payload_flits;
      vault_req.callback = [this, vault, response](Request& done) {
        Packet out = *response;
        out.req = done;
        out.req.depart = m_clk;
        m_vault_responses[vault].push_back(std::move(out));
      };
      if (m_controllers[vault]->send(vault_req)) {
        s_request_packet_latency += m_clk - hmc_arrive;
        link.input.pop_front();
        used_vaults.insert(vault);
      } else {
        packet.host_callback = std::move(response->host_callback);
      }
    }

    std::set<int> used_links;
    for (auto& responses : m_vault_responses) {
      if (responses.empty()) {
        continue;
      }
      int slid = responses.front().slid;
      if (used_links.count(slid)) {
        continue;
      }
      Link& link = m_links[slid];
      if (responses.front().flits <= m_link_buffer - static_cast<int>(link.output.size())) {
        link.output.push_back(std::move(responses.front()));
        responses.pop_front();
        used_links.insert(slid);
      }
    }
  }
};

}  // namespace Ramulator
