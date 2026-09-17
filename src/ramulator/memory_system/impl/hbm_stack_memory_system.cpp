#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "ramulator/base/param.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/i_controller.h"
#include "ramulator/memory_system/i_memory_system.h"

namespace Ramulator {

// HBM stacks: a base (logic) die in front of one DRAM controller per channel, `stacks` of them.
// Host requests cross the host PHY/interposer (plus, with more than one stack, a switch hop to reach
// the stack). PIM requests (Request::Flag::PIM) come from base-die cores and pay a per-hop base-die
// interconnect cost to reach the target channel inside their own stack.
//
// A real HBM-PIM unit cannot reach another stack at all: there is no base-die path off the stack. Two models of
// what a cross-stack request costs, chosen with `inter_unit`:
//   "host"  (default)  out through its own PHY, across the switch and into the other stack's PHY
//                      (2 x host_phy + host_stack_link) -- stands for going through the host.
//   "link"             an explicit inter-unit link with a fixed traversal latency and finite bandwidth
//                      (`inter_unit_latency_ps`, `inter_unit_bw_gbps`), as NDP systems such as SynCron
//                      (HPCA'21 Table 5: 12.8 GB/s per direction, 40 ns per cache line) describe.
// Either way it is a MODELLING CHOICE, counted separately in pim_remote_stack_requests.
//
// `source_mapping` says what a PIM source id means:
//   "per_channel" (default)  one PIM unit per channel; its home is that channel and hops are the base-die distance
//                            to the target channel (HBM-PIM).
//   "per_stack"              `cores_per_stack` cores share a stack; anything inside their own stack is local (the
//                            on-die network that gets them there is modelled by the CPU simulator, not here).
// `addressing` says how an address picks a channel:
//   "channel_interleave" (default)  consecutive blocks round-robin over every channel of every stack.
//   "unit_major"                    the address's high bits pick the stack, low bits interleave channels inside it,
//                                   so a contiguous region lives in one stack -- what per-unit data placement needs.
class HBMStackMemorySystem final : public IMemorySystem, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IMemorySystem, HBMStackMemorySystem, "HBMStack");

 private:
  struct Txn {
    Request req;
    bool pim = false;
    int remaining = 0;
    int delay_ticks = 0;
    Clk_t arrive = 0;
  };

  struct Transfer {
    std::shared_ptr<Txn> txn;
    Addr_t addr = 0;
    AddrVec_t addr_vec;
    int channel = 0;
  };

  std::vector<IController*> m_controllers;
  unsigned int m_clock_ratio = 1;
  int m_host_phy_latency_ps = 0;
  int m_base_die_hop_ps = 0;
  int m_pim_mesh_width = 4;
  int m_ingress_buffer = 32;
  int m_request_bytes = 64;
  int m_stacks = 1;
  int m_host_stack_link_ps = 0;
  std::string m_source_mapping = "per_channel";
  std::string m_addressing = "channel_interleave";
  int m_cores_per_stack = 1;
  std::string m_inter_unit = "host";
  int m_inter_unit_latency_ps = 40000;    // SynCron Table 5: 40 ns per cache line
  float m_inter_unit_bw_gbps = 12.8f;     // SynCron Table 5: 12.8 GB/s per direction

  Clk_t m_clk = 0;
  int m_channels = 0;
  int m_tx_bytes = 0;
  int m_host_phy_ticks = 0;
  int m_hop_ticks = 0;
  int m_stack_link_ticks = 0;
  int m_channels_per_stack = 0;
  bool m_unit_major = false;
  bool m_source_per_stack = false;
  bool m_link_model = false;
  int m_inter_unit_ticks = 0;
  double m_bytes_per_tick = 0.0;
  uint64_t m_bytes_per_stack = 0;
  std::vector<Clk_t> m_link_free;         // [src_stack * stacks + dst_stack]: when that direction is free again
  std::vector<int> m_level_size;
  int m_column_level = -1;
  int m_column_per_tx = 1;
  uint64_t m_capacity = 0;

  std::multimap<Clk_t, Transfer> m_inbound;
  std::vector<std::deque<Transfer>> m_ready;
  std::vector<int> m_ingress;
  std::multimap<Clk_t, std::shared_ptr<Txn>> m_outbound;

  size_t s_host_requests = 0;
  size_t s_pim_requests = 0;
  size_t s_transfers = 0;
  size_t s_rejected_ingress_full = 0;
  size_t s_host_latency = 0;
  size_t s_pim_latency = 0;
  size_t s_pim_local_requests = 0;
  size_t s_pim_hops = 0;
  size_t s_pim_remote_stack_requests = 0;
  size_t s_inter_unit_bytes = 0;
  size_t s_inter_unit_queue_ticks = 0;
  std::vector<size_t> s_requests_per_channel;

 public:
  void init() override {
    RAMULATOR_PARSE_PARAM(m_clock_ratio, unsigned int, "clock_ratio").required();
    RAMULATOR_PARSE_PARAM(m_host_phy_latency_ps, int, "host_phy_latency_ps").default_val(0);
    RAMULATOR_PARSE_PARAM(m_base_die_hop_ps, int, "base_die_hop_ps").default_val(0);
    RAMULATOR_PARSE_PARAM(m_pim_mesh_width, int, "pim_mesh_width").default_val(4);
    RAMULATOR_PARSE_PARAM(m_ingress_buffer, int, "ingress_buffer").default_val(32);
    RAMULATOR_PARSE_PARAM(m_request_bytes, int, "request_bytes").default_val(64);
    RAMULATOR_PARSE_PARAM(m_stacks, int, "stacks").default_val(1);
    RAMULATOR_PARSE_PARAM(m_host_stack_link_ps, int, "host_stack_link_ps").default_val(0);
    RAMULATOR_PARSE_PARAM(m_source_mapping, std::string, "source_mapping").default_val("per_channel");
    RAMULATOR_PARSE_PARAM(m_addressing, std::string, "addressing").default_val("channel_interleave");
    RAMULATOR_PARSE_PARAM(m_cores_per_stack, int, "cores_per_stack").default_val(1);
    RAMULATOR_PARSE_PARAM(m_inter_unit, std::string, "inter_unit").default_val("host");
    RAMULATOR_PARSE_PARAM(m_inter_unit_latency_ps, int, "inter_unit_latency_ps").default_val(40000);
    RAMULATOR_PARSE_PARAM(m_inter_unit_bw_gbps, float, "inter_unit_bw_gbps").default_val(12.8f);
    RAMULATOR_CREATE_CHILD_LIST(m_controllers, IController);

    m_channels = static_cast<int>(m_controllers.size());
    if (m_channels == 0) {
      throw std::runtime_error("HBMStack: needs at least one controller");
    }
    if (m_pim_mesh_width <= 0) {
      throw std::runtime_error("HBMStack: pim_mesh_width must be positive");
    }
    if (m_stacks <= 0 || m_channels % m_stacks != 0) {
      throw std::runtime_error(fmt::format("HBMStack: {} controllers do not split over {} stacks", m_channels,
                                           m_stacks));
    }
    m_channels_per_stack = m_channels / m_stacks;
    if (m_source_mapping == "per_stack") {
      m_source_per_stack = true;
    } else if (m_source_mapping != "per_channel") {
      throw std::runtime_error("HBMStack: source_mapping must be per_channel or per_stack");
    }
    if (m_addressing == "unit_major") {
      m_unit_major = true;
    } else if (m_addressing != "channel_interleave") {
      throw std::runtime_error("HBMStack: addressing must be channel_interleave or unit_major");
    }
    if (m_inter_unit == "link") {
      m_link_model = true;
    } else if (m_inter_unit != "host") {
      throw std::runtime_error("HBMStack: inter_unit must be host or link");
    }
    if (m_cores_per_stack <= 0) {
      throw std::runtime_error("HBMStack: cores_per_stack must be positive");
    }
    for (int i = 0; i < m_channels; i++) {
      dynamic_cast<Implementation*>(m_controllers[i])->set_id(fmt::format("Channel {}", i));
      m_controllers[i]->set_channel_id(i);
      m_controllers[i]->m_clock_ratio = m_clock_ratio;
    }

    auto* ctrl = dynamic_cast<ControllerBase*>(m_controllers[0]);
    if (!ctrl) {
      throw std::runtime_error("HBMStack: controllers must derive from ControllerBase");
    }
    const DRAMSpec* spec = ctrl->m_device.m_spec;
    m_tx_bytes = spec->get_tx_bytes();
    if (m_request_bytes < m_tx_bytes) {
      m_request_bytes = m_tx_bytes;
    }
    m_level_size.assign(spec->level_count, 1);
    m_capacity = static_cast<uint64_t>(m_tx_bytes) / spec->internal_prefetch_size;
    for (int lvl = 1; lvl < spec->level_count; lvl++) {
      m_level_size[lvl] = spec->organization.level_sizes[lvl];
      m_capacity *= m_level_size[lvl];
    }
    m_capacity *= m_channels;
    m_bytes_per_stack = m_capacity / m_stacks;
    m_column_level = spec->get_level_id("Column");
    m_column_per_tx = spec->internal_prefetch_size;

    double tick_ps = ctrl->get_tCK() * 1000.0;
    m_host_phy_ticks = static_cast<int>(std::ceil(m_host_phy_latency_ps / tick_ps));
    m_hop_ticks = static_cast<int>(std::ceil(m_base_die_hop_ps / tick_ps));
    m_stack_link_ticks = static_cast<int>(std::ceil(m_host_stack_link_ps / tick_ps));
    m_inter_unit_ticks = static_cast<int>(std::ceil(m_inter_unit_latency_ps / tick_ps));
    m_bytes_per_tick = m_inter_unit_bw_gbps * (tick_ps / 1000.0);  // GB/s * ns per tick = bytes per tick
    m_link_free.assign(static_cast<size_t>(m_stacks) * m_stacks, 0);

    m_ready.resize(m_channels);
    m_ingress.assign(m_channels, 0);
    s_requests_per_channel.assign(m_channels, 0);

    m_stats.add("host_requests", s_host_requests);
    m_stats.add("pim_requests", s_pim_requests);
    m_stats.add("transfers", s_transfers);
    m_stats.add("rejected_ingress_full", s_rejected_ingress_full);
    m_stats.add("host_latency", s_host_latency);
    m_stats.add("pim_latency", s_pim_latency);
    m_stats.add("pim_local_requests", s_pim_local_requests);
    m_stats.add("pim_hops", s_pim_hops);
    m_stats.add("pim_remote_stack_requests", s_pim_remote_stack_requests);
    m_stats.add("stacks", m_stacks);
    m_stats.add("channels_per_stack", m_channels_per_stack);
    m_stats.add("host_stack_link_ticks", m_stack_link_ticks);
    m_stats.add("inter_unit_bytes", s_inter_unit_bytes);
    m_stats.add("inter_unit_queue_ticks", s_inter_unit_queue_ticks);
    m_stats.add("inter_unit_latency_ticks", m_inter_unit_ticks);
    m_stats.add("requests_per_channel", s_requests_per_channel);
    m_stats.add("host_phy_ticks", m_host_phy_ticks);
    m_stats.add("base_die_hop_ticks", m_hop_ticks);
  }

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
  }

  bool send(Request& req) override {
    if (req.size_bytes <= 0 || req.size_bytes > m_request_bytes) {
      throw std::runtime_error(fmt::format("HBMStack: size_bytes must be in (0, {}], got {}", m_request_bytes,
                                           req.size_bytes));
    }
    if (req.type_id != Request::Type::Read && req.type_id != Request::Type::Write) {
      throw std::runtime_error(fmt::format("HBMStack: unsupported request type {}", req.type_id));
    }

    int transfers = (req.size_bytes + m_tx_bytes - 1) / m_tx_bytes;
    Addr_t base = static_cast<Addr_t>((static_cast<uint64_t>(req.addr) % m_capacity) / m_tx_bytes * m_tx_bytes);
    std::vector<Transfer> parts(transfers);
    for (int k = 0; k < transfers; k++) {
      parts[k].addr = static_cast<Addr_t>((static_cast<uint64_t>(base) + k * m_tx_bytes) % m_capacity);
      parts[k].channel = map_address(parts[k].addr, parts[k].addr_vec);
    }
    for (int k = 0; k < transfers; k++) {
      int needed = static_cast<int>(std::count_if(parts.begin(), parts.end(),
                                                  [&](const Transfer& t) { return t.channel == parts[k].channel; }));
      if (m_ingress[parts[k].channel] + needed > m_ingress_buffer) {
        s_rejected_ingress_full++;
        return false;
      }
    }

    auto txn = std::make_shared<Txn>();
    txn->req = req;
    txn->pim = req.flags & Request::Flag::PIM;
    txn->remaining = transfers;
    txn->arrive = m_clk;
    txn->req.hops = 0;
    if (txn->pim) {
      s_pim_requests++;
      int src = req.source_id < 0 ? 0 : req.source_id;
      int src_stack = m_source_per_stack ? (src / m_cores_per_stack) % m_stacks
                                        : (src % m_channels) / m_channels_per_stack;
      int dst_stack = parts[0].channel / m_channels_per_stack;
      if (src_stack == dst_stack) {
        // Inside its own stack. With per_stack sources the on-die network that carries the request is the CPU
        // simulator's business, so there are no base-die hops to add here.
        int hops = m_source_per_stack ? 0 : channel_distance(src % m_channels, parts[0].channel);
        txn->req.hops = hops;
        txn->delay_ticks = hops * m_hop_ticks;
        s_pim_hops += hops;
        if (hops == 0) {
          s_pim_local_requests++;
        }
      } else if (m_link_model) {
        // Explicit inter-unit link: fixed traversal latency, and the payload occupies the link, so later
        // transfers queue behind it. delay_ticks is one traversal; the code below charges it on the way in and
        // again on the way out, which is the round trip.
        size_t dir = static_cast<size_t>(src_stack) * m_stacks + dst_stack;
        Clk_t ready = std::max(m_clk, m_link_free[dir]);
        Clk_t occupancy = static_cast<Clk_t>(std::ceil(req.size_bytes / m_bytes_per_tick));
        m_link_free[dir] = ready + occupancy;
        txn->delay_ticks = m_inter_unit_ticks + static_cast<int>(ready - m_clk);
        s_inter_unit_bytes += req.size_bytes;
        s_inter_unit_queue_ticks += ready - m_clk;
        s_pim_remote_stack_requests++;
      } else {
        // No base-die path off the stack, so this goes out through the host (modelling choice).
        txn->delay_ticks = 2 * m_host_phy_ticks + m_stack_link_ticks;
        s_pim_remote_stack_requests++;
      }
    } else {
      txn->delay_ticks = m_host_phy_ticks + m_stack_link_ticks;
      s_host_requests++;
    }

    for (auto& part : parts) {
      part.txn = txn;
      m_ingress[part.channel]++;
      s_requests_per_channel[part.channel]++;
      s_transfers++;
      m_inbound.emplace(m_clk + txn->delay_ticks, std::move(part));
    }
    return true;
  }

  void tick() override {
    m_clk++;
    for (auto it = m_inbound.begin(); it != m_inbound.end() && it->first <= m_clk;) {
      m_ready[it->second.channel].push_back(std::move(it->second));
      it = m_inbound.erase(it);
    }
    for (int ch = 0; ch < m_channels; ch++) {
      auto& ready = m_ready[ch];
      while (!ready.empty()) {
        Transfer& t = ready.front();
        Request vreq(t.addr, t.txn->req.type_id, t.txn->req.source_id, {});
        vreq.addr_vec = t.addr_vec;
        vreq.size_bytes = m_tx_bytes;
        vreq.flags = t.txn->req.flags;
        auto txn = t.txn;
        vreq.callback = [this, txn](Request&) { on_transfer_done(txn); };
        if (!m_controllers[ch]->send(vreq)) {
          break;
        }
        m_ingress[ch]--;
        ready.pop_front();
      }
    }
    for (auto* controller : m_controllers) {
      controller->tick();
    }
    for (auto it = m_outbound.begin(); it != m_outbound.end() && it->first <= m_clk;) {
      auto txn = std::move(it->second);
      it = m_outbound.erase(it);
      complete(*txn);
    }
  }

  int get_clock_ratio() override {
    return m_clock_ratio;
  }

  float get_tCK() override {
    return m_controllers[0]->get_tCK();
  }

  int get_tx_bytes() override {
    return m_request_bytes;
  }

 private:
  int map_address(Addr_t addr, AddrVec_t& addr_vec) const {
    addr_vec.assign(m_level_size.size(), 0);
    uint64_t byte_addr = static_cast<uint64_t>(addr);
    int stack = 0;
    if (m_unit_major) {
      // High bits pick the stack, so a contiguous region belongs to one NDP unit.
      stack = static_cast<int>(byte_addr / m_bytes_per_stack) % m_stacks;
      byte_addr %= m_bytes_per_stack;
    }
    uint64_t a = byte_addr / m_tx_bytes;
    int column_positions = m_level_size[m_column_level] / m_column_per_tx;
    addr_vec[m_column_level] = static_cast<int>(a % column_positions) * m_column_per_tx;
    a /= column_positions;
    int channels_here = m_unit_major ? m_channels_per_stack : m_channels;
    int channel = stack * (m_unit_major ? m_channels_per_stack : 0) + static_cast<int>(a % channels_here);
    a /= channels_here;
    for (int lvl = 1; lvl < static_cast<int>(m_level_size.size()); lvl++) {
      if (lvl == m_column_level) {
        continue;
      }
      addr_vec[lvl] = static_cast<int>(a % m_level_size[lvl]);
      a /= m_level_size[lvl];
    }
    addr_vec[0] = channel;
    return channel;
  }

  // Mesh distance inside one stack's base die; both ids are reduced to their position in that stack.
  int channel_distance(int unit, int channel) const {
    int src = unit % m_channels_per_stack;
    int dst = channel % m_channels_per_stack;
    return std::abs(src / m_pim_mesh_width - dst / m_pim_mesh_width) +
           std::abs(src % m_pim_mesh_width - dst % m_pim_mesh_width);
  }

  void on_transfer_done(const std::shared_ptr<Txn>& txn) {
    if (--txn->remaining > 0) {
      return;
    }
    if (txn->delay_ticks == 0) {
      complete(*txn);
    } else {
      m_outbound.emplace(m_clk + txn->delay_ticks, txn);
    }
  }

  void complete(Txn& txn) {
    (txn.pim ? s_pim_latency : s_host_latency) += m_clk - txn.arrive;
    if (txn.req.callback) {
      txn.req.callback(txn.req);
    }
  }
};

}  // namespace Ramulator
