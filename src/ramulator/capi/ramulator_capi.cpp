#include "ramulator/capi/ramulator_capi.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ramulator/base/config.h"
#include "ramulator/base/factory.h"
#include "ramulator/frontend/i_frontend.h"
#include "ramulator/memory_system/i_memory_system.h"
#include "ramulator/memory_system/network/hmc_network_queries.h"

struct r2_sim {
  Ramulator::IFrontEnd* frontend = nullptr;
  Ramulator::IMemorySystem* memory_system = nullptr;
  Ramulator::IHMCNetworkQueries* hmc = nullptr;
  bool finalized = false;
};

namespace {

thread_local std::string g_last_error;

r2_sim* create(const Ramulator::ConfigNode& config) {
  auto* sim = new r2_sim();
  sim->frontend = Ramulator::Factory::create_frontend(config);
  sim->memory_system = Ramulator::Factory::create_memory_system(config);
  sim->frontend->connect_memory_system(sim->memory_system);
  sim->memory_system->connect_frontend(sim->frontend);
  sim->hmc = dynamic_cast<Ramulator::IHMCNetworkQueries*>(sim->memory_system);
  return sim;
}

Ramulator::ConfigNode with_override(const Ramulator::ConfigNode& node, const std::vector<std::string>& path,
                                    size_t depth, const char* value) {
  Ramulator::ConfigNode copy = node;
  if (depth + 1 == path.size()) {
    copy.set(path[depth], Ramulator::ConfigNode(value));
  } else {
    copy.set(path[depth], with_override(node[path[depth]], path, depth + 1, value));
  }
  return copy;
}

}  // namespace

extern "C" {

const char* r2_last_error(void) { return g_last_error.c_str(); }

r2_sim* r2_create_from_file(const char* yaml_path) {
  try {
    return create(Ramulator::Config::parse_config_file(yaml_path));
  } catch (const std::exception& e) {
    g_last_error = e.what();
    return nullptr;
  }
}

r2_sim* r2_create_from_file_cores(const char* yaml_path, int num_cores) {
  try {
    auto config = Ramulator::Config::parse_config_file(yaml_path);
    if (num_cores > 0) {
      auto frontend = config["frontend"];
      frontend.set("num_cores", num_cores);
      config.set("frontend", frontend);
    }
    return create(config);
  } catch (const std::exception& e) {
    g_last_error = e.what();
    return nullptr;
  }
}

r2_sim* r2_create_ex(const char* yaml_path, int num_cores, int num_overrides,
                     const char* const* keys, const char* const* values) {
  try {
    auto config = Ramulator::Config::parse_config_file(yaml_path);
    for (int i = 0; i < num_overrides; i++) {
      std::vector<std::string> path;
      std::stringstream ss(keys[i]);
      std::string part;
      while (std::getline(ss, part, '.')) path.push_back(part);
      if (!path.empty()) config = with_override(config, path, 0, values[i]);
    }
    if (num_cores > 0) {
      auto frontend = config["frontend"];
      frontend.set("num_cores", num_cores);
      config.set("frontend", frontend);
    }
    return create(config);
  } catch (const std::exception& e) {
    g_last_error = e.what();
    return nullptr;
  }
}

r2_sim* r2_create_from_string(const char* yaml_text) {
  try {
    return create(Ramulator::Config::parse_config_string(yaml_text));
  } catch (const std::exception& e) {
    g_last_error = e.what();
    return nullptr;
  }
}

void r2_destroy(r2_sim* sim) {
  if (!sim) return;
  delete sim->frontend;
  delete sim->memory_system;
  delete sim;
}

int r2_send(r2_sim* sim, int type, uint64_t addr, int source_id, int size_bytes,
            uint64_t token, r2_callback cb, void* ctx) {
  try {
    return sim->frontend->receive_external_requests(
               type, static_cast<Ramulator::Addr_t>(addr), source_id,
               [cb, ctx, token](Ramulator::Request& req) {
                 if (cb) cb(ctx, token, static_cast<uint64_t>(req.addr), req.type_id, req.source_id);
               },
               size_bytes)
               ? 1
               : 0;
  } catch (const std::exception& e) {
    g_last_error = e.what();
    return -1;
  }
}

int r2_send_ex(r2_sim* sim, int type, uint64_t addr, int source_id, int size_bytes,
               uint32_t flags, uint64_t token, r2_callback_ex cb, void* ctx) {
  try {
    Ramulator::Request req(static_cast<Ramulator::Addr_t>(addr), type, source_id,
                           [cb, ctx, token](Ramulator::Request& r) {
                             if (cb) cb(ctx, token, static_cast<uint64_t>(r.addr), r.type_id, r.source_id, r.hops);
                           });
    req.size_bytes = size_bytes;
    req.flags = flags;
    return sim->memory_system->send(req) ? 1 : 0;
  } catch (const std::exception& e) {
    g_last_error = e.what();
    return -1;
  }
}

void r2_tick(r2_sim* sim) { sim->memory_system->tick(); }

double r2_get_tck_ns(r2_sim* sim) { return sim->memory_system->get_tCK(); }

int r2_get_clock_ratio(r2_sim* sim) { return sim->memory_system->get_clock_ratio(); }

int r2_get_tx_bytes(r2_sim* sim) { return sim->memory_system->get_tx_bytes(); }

void r2_finalize(r2_sim* sim) {
  if (sim->finalized) return;
  sim->frontend->finalize();
  sim->memory_system->finalize();
  sim->finalized = true;
}

uint64_t r2_get_memory_size(r2_sim* sim) { return sim->hmc ? sim->hmc->memory_size() : 0; }

int r2_hmc_estimate_hops(r2_sim* sim, int source_id, uint64_t addr, uint32_t flags) {
  return sim->hmc ? sim->hmc->estimate_hops(source_id, static_cast<Ramulator::Addr_t>(addr), flags) : -1;
}

int r2_hmc_access_position(r2_sim* sim, int source_id, uint64_t addr, uint32_t flags) {
  return sim->hmc ? sim->hmc->access_position(source_id, static_cast<Ramulator::Addr_t>(addr), flags) : -1;
}

int r2_hmc_target_vault(r2_sim* sim, uint64_t addr) {
  return sim->hmc ? sim->hmc->target_vault(static_cast<Ramulator::Addr_t>(addr)) : -1;
}

int r2_hmc_target_stack(r2_sim* sim, uint64_t addr) {
  return sim->hmc ? sim->hmc->target_stack(static_cast<Ramulator::Addr_t>(addr)) : -1;
}

int r2_hmc_num_stacks(r2_sim* sim) { return sim->hmc ? sim->hmc->num_stacks() : -1; }

int r2_hmc_vaults_per_stack(r2_sim* sim) { return sim->hmc ? sim->hmc->vaults_per_stack() : -1; }

int r2_write_stats(r2_sim* sim, const char* path) {
  std::ofstream ofs(path);
  if (!ofs) {
    g_last_error = std::string("cannot open ") + path;
    return -1;
  }
  sim->frontend->print_stats(ofs);
  sim->memory_system->print_stats(ofs);
  return ofs ? 0 : -1;
}

}  // extern "C"
