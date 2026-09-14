#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#include <fmt/format.h>

#include "ramulator/base/param.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/i_controller.h"
#include "ramulator/memory_system/i_memory_system.h"

namespace Ramulator {

// One HBM stack: a base (logic) die in front of one DRAM controller per channel.
// Host requests cross the host PHY/interposer; PIM requests (Request::Flag::PIM) come from
// base-die cores and pay a per-hop base-die interconnect cost to reach the target channel.
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

  Clk_t m_clk = 0;
  int m_channels = 0;
  int m_tx_bytes = 0;
  int m_host_phy_ticks = 0;
  int m_hop_ticks = 0;
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
  std::vector<size_t> s_requests_per_channel;

 public:
  void init() override {
    RAMULATOR_PARSE_PARAM(m_clock_ratio, unsigned int, "clock_ratio").required();
    RAMULATOR_PARSE_PARAM(m_host_phy_latency_ps, int, "host_phy_latency_ps").default_val(0);
    RAMULATOR_PARSE_PARAM(m_base_die_hop_ps, int, "base_die_hop_ps").default_val(0);
    RAMULATOR_PARSE_PARAM(m_pim_mesh_width, int, "pim_mesh_width").default_val(4);
    RAMULATOR_PARSE_PARAM(m_ingress_buffer, int, "ingress_buffer").default_val(32);
    RAMULATOR_PARSE_PARAM(m_request_bytes, int, "request_bytes").default_val(64);
    RAMULATOR_CREATE_CHILD_LIST(m_controllers, IController);

    m_channels = static_cast<int>(m_controllers.size());
    if (m_channels == 0) {
      throw std::runtime_error("HBMStack: needs at least one controller");
    }
    if (m_pim_mesh_width <= 0) {
      throw std::runtime_error("HBMStack: pim_mesh_width must be positive");
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
    m_column_level = spec->get_level_id("Column");
    m_column_per_tx = spec->internal_prefetch_size;

    double tick_ps = ctrl->get_tCK() * 1000.0;
    m_host_phy_ticks = static_cast<int>(std::ceil(m_host_phy_latency_ps / tick_ps));
    m_hop_ticks = static_cast<int>(std::ceil(m_base_die_hop_ps / tick_ps));

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
      int hops = channel_distance(req.source_id, parts[0].channel);
      txn->req.hops = hops;
      txn->delay_ticks = hops * m_hop_ticks;
      s_pim_requests++;
      s_pim_hops += hops;
      if (hops == 0) {
        s_pim_local_requests++;
      }
    } else {
      txn->delay_ticks = m_host_phy_ticks;
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
    uint64_t a = static_cast<uint64_t>(addr) / m_tx_bytes;
    int column_positions = m_level_size[m_column_level] / m_column_per_tx;
    addr_vec[m_column_level] = static_cast<int>(a % column_positions) * m_column_per_tx;
    a /= column_positions;
    int channel = static_cast<int>(a % m_channels);
    a /= m_channels;
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

  int channel_distance(int source_id, int channel) const {
    int src = source_id < 0 ? 0 : source_id % m_channels;
    return std::abs(src / m_pim_mesh_width - channel / m_pim_mesh_width) +
           std::abs(src % m_pim_mesh_width - channel % m_pim_mesh_width);
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
