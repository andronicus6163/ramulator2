#ifndef RAMULATOR_MEMORY_SYSTEM_NETWORK_BOOKSIM_INTERCONNECT_H
#define RAMULATOR_MEMORY_SYSTEM_NETWORK_BOOKSIM_INTERCONNECT_H

namespace Ramulator::Booksim {

// Thin wrapper over the (MultiPIM-modified) Booksim 2 HMC switch interconnect.
// subnet = stack index, port = link device index or stack_links + local vault index.
void init(unsigned subnets, const char* config_file, unsigned links_per_stack, unsigned vaults_per_stack);
bool has_buffer(unsigned subnet, unsigned input, unsigned size);
void push(unsigned subnet, unsigned input, unsigned output, void* data, unsigned size, bool is_request, bool is_read);
void* pop(unsigned subnet, unsigned output);
void* top(unsigned subnet, unsigned output);
void advance();
bool busy();
unsigned flit_size();

}  // namespace Ramulator::Booksim

#endif
