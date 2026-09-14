add_library(ramulator-memorysystem OBJECT)

target_sources(
  ramulator-memorysystem PRIVATE
  i_memory_system.h

  channel_mapper/i_channel_mapper.h
  channel_mapper/impl/cache_line_interleave.cpp
  channel_mapper/impl/gem5_port_interleave.cpp
  channel_mapper/impl/passthrough_channel_mapper.cpp

  impl/generic_dram_system.cpp
  impl/hmc_memory_system.cpp
  impl/hmc_network_memory_system.cpp
  impl/hbm_stack_memory_system.cpp

  network/hmc_network_queries.h
  network/topology.h
  network/topology.cpp
)

target_link_libraries(
  ramulator
  PRIVATE
  ramulator-memorysystem
)

add_subdirectory(network)
