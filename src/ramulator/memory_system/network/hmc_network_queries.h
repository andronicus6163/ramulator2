#ifndef RAMULATOR_MEMORY_SYSTEM_NETWORK_HMC_NETWORK_QUERIES_H
#define RAMULATOR_MEMORY_SYSTEM_NETWORK_HMC_NETWORK_QUERIES_H

#include <cstdint>

#include "ramulator/base/type.h"

namespace Ramulator {

// Host-simulator queries exposed by multi-stack HMC memory systems.
class IHMCNetworkQueries {
 public:
  virtual ~IHMCNetworkQueries() = default;
  virtual int estimate_hops(int source_id, Addr_t addr, uint32_t flags) = 0;
  virtual int access_position(int source_id, Addr_t addr, uint32_t flags) = 0;
  virtual int target_vault(Addr_t addr) = 0;
  virtual int target_stack(Addr_t addr) = 0;
  virtual int num_stacks() const = 0;
  virtual int vaults_per_stack() const = 0;
  virtual uint64_t memory_size() const = 0;
};

}  // namespace Ramulator

#endif
