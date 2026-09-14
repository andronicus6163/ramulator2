#ifndef RAMULATOR_CAPI_H
#define RAMULATOR_CAPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct r2_sim r2_sim;

enum { R2_READ = 0, R2_WRITE = 1 };

enum { R2_FLAG_PIM = 1, R2_FLAG_PTW = 2, R2_FLAG_IDEAL_MEMNET = 4 };
typedef void (*r2_callback)(void* ctx, uint64_t token, uint64_t addr, int type, int source_id);
typedef void (*r2_callback_ex)(void* ctx, uint64_t token, uint64_t addr, int type, int source_id, int hops);

r2_sim* r2_create_from_file(const char* yaml_path);
r2_sim* r2_create_from_string(const char* yaml_text);
r2_sim* r2_create_from_file_cores(const char* yaml_path, int num_cores);
/* keys are dotted config paths, e.g. "memory_system.rmab_size"; values are YAML scalars */
r2_sim* r2_create_ex(const char* yaml_path, int num_cores, int num_overrides,
                     const char* const* keys, const char* const* values);
void r2_destroy(r2_sim* sim);

int r2_send(r2_sim* sim, int type, uint64_t addr, int source_id, int size_bytes,
            uint64_t token, r2_callback cb, void* ctx);
int r2_send_ex(r2_sim* sim, int type, uint64_t addr, int source_id, int size_bytes,
               uint32_t flags, uint64_t token, r2_callback_ex cb, void* ctx);
void r2_tick(r2_sim* sim);

double r2_get_tck_ns(r2_sim* sim);
int r2_get_clock_ratio(r2_sim* sim);
int r2_get_tx_bytes(r2_sim* sim);

uint64_t r2_get_memory_size(r2_sim* sim);
/* HMC network queries; return -1 when the memory system is not a multi-stack HMC */
int r2_hmc_estimate_hops(r2_sim* sim, int source_id, uint64_t addr, uint32_t flags);
int r2_hmc_access_position(r2_sim* sim, int source_id, uint64_t addr, uint32_t flags);
int r2_hmc_target_vault(r2_sim* sim, uint64_t addr);
int r2_hmc_target_stack(r2_sim* sim, uint64_t addr);
int r2_hmc_num_stacks(r2_sim* sim);
int r2_hmc_vaults_per_stack(r2_sim* sim);
int r2_write_stats(r2_sim* sim, const char* path);
void r2_finalize(r2_sim* sim);

const char* r2_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
