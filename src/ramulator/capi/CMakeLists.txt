add_library(ramulator-capi OBJECT)

target_sources(
  ramulator-capi PRIVATE
  ramulator_capi.h ramulator_capi.cpp
)

target_link_libraries(
  ramulator
  PRIVATE
  ramulator-capi
)
