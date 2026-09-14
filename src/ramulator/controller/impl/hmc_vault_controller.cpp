#include <deque>

#include "ramulator/base/base.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/refresh/i_refresh_manager.h"
#include "ramulator/controller/rowpolicy/i_row_policy.h"

namespace Ramulator {

class HMCVaultController : public ControllerBase {
  RAMULATOR_REGISTER_IMPLEMENTATION_DERIVED(IController, HMCVaultController, ControllerBase, "HMCVault")

 private:
  int m_burst_count = 1;
  int m_write_latency = 0;
  std::deque<Request> m_pending_writes;

 public:
  void init() override {
    init_base();
    RAMULATOR_PARSE_PARAM(m_burst_count, int, "burst_count").default_val(1);
    RAMULATOR_PARSE_PARAM(m_write_latency, int, "write_latency").default_val(-1);
    if (m_write_latency < 0) {
      m_write_latency = m_device.m_spec->get_timing_value("nBL");
    }
  }

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    setup_base(frontend, memory_system);
  }

  bool send(Request& req) override {
    if (req.burst_remaining <= 0) {
      req.burst_remaining = m_burst_count;
    }
    return ControllerBase::send(req);
  }

  void tick() override;

 private:
  void serve_completed_writes() {
    while (!m_pending_writes.empty() && m_pending_writes.front().depart <= m_clk) {
      auto& req = m_pending_writes.front();
      if (req.callback) {
        req.callback(req);
      }
      m_pending_writes.pop_front();
    }
  }
};

void HMCVaultController::tick() {
  tick_prologue();
  serve_completed_writes();

  m_refresh->tick();

  m_rowpolicy->pre_schedule();
  for (auto* p : m_plugins) {
    p->pre_schedule();
  }

  Candidate cand = pick_best_ready_from(m_active_buffer, {});
  if (!cand.valid) {
    cand = pick_priority_if();
  }
  if (!cand.valid && m_priority_buffer.size() == 0) {
    cand = pick_rw_if();
  }

  if (cand.valid) {
    m_rowpolicy->try_upgrade_command(*cand.it);

    if (!cand.it->is_stat_updated) {
      update_request_stats(cand.it);
    }

    m_device.issue_command(cand.it->command, cand.it->addr_vec, m_clk);

    m_rowpolicy->on_issue(*cand.it);
    for (auto* p : m_plugins) {
      p->on_issue(*cand.it);
    }

    if (cand.it->command == cand.it->final_command) {
      if (cand.it->type_id >= 0 && --cand.it->burst_remaining > 0) {
        // Remaining bursts re-issue the same command once timing allows.
      } else if (cand.it->type_id == Request::Type::Write) {
        if (cand.buffer == &m_active_buffer) {
          m_active_per_bank[m_device.get_flat_bank_id(cand.it->addr_vec)]--;
        } else if (cand.buffer == &m_write_buffer) {
          m_buffered_write_addrs.erase(cand.it->addr);
        }
        cand.it->depart = m_clk + m_write_latency;
        s_num_write_reqs_served++;
        if (m_write_latency == 0) {
          Request done = *cand.it;
          cand.buffer->remove(cand.it);
          if (done.callback) {
            done.callback(done);
          }
        } else {
          m_pending_writes.push_back(*cand.it);
          cand.buffer->remove(cand.it);
        }
      } else {
        retire_request(cand.it, *cand.buffer);
      }
    } else if (m_device.m_spec->command_meta[cand.it->command].is_opening) {
      promote_to_active(cand.it, *cand.buffer);
    }
  }

  m_rowpolicy->post_schedule();
  for (auto* p : m_plugins) {
    p->post_schedule();
  }
}

}  // namespace Ramulator
