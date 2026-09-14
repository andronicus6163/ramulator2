#include "booksim_interconnect.h"

#include "globals.hpp"
#include "interconnect_interface.hpp"


namespace Ramulator::Booksim {

void init(unsigned subnets, const char* config_file, unsigned links_per_stack, unsigned vaults_per_stack) {
  g_icnt_interface = InterconnectInterface::New(subnets, config_file);
  g_icnt_interface->CreateInterconnect(links_per_stack, vaults_per_stack);
  g_icnt_interface->Init();
}

bool has_buffer(unsigned subnet, unsigned input, unsigned size) {
  return g_icnt_interface->HasBuffer(subnet, input, size);
}

void push(unsigned subnet, unsigned input, unsigned output, void* data, unsigned size, bool is_request, bool is_read) {
  g_icnt_interface->Push(subnet, input, output, data, size, is_request, is_read);
}

void* pop(unsigned subnet, unsigned output) {
  return g_icnt_interface->Pop(subnet, output);
}

void* top(unsigned subnet, unsigned output) {
  return g_icnt_interface->Top(subnet, output);
}

void advance() {
  g_icnt_interface->Advance();
}

bool busy() {
  return g_icnt_interface->Busy();
}

unsigned flit_size() {
  return g_icnt_interface->GetFlitSize();
}

}  // namespace Ramulator::Booksim
