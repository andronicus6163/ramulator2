enable_language(C)
find_program(FLEX_EXECUTABLE flex REQUIRED)
find_program(BISON_EXECUTABLE bison REQUIRED)

set(BOOKSIM_SRC ${CMAKE_CURRENT_SOURCE_DIR}/booksim2)
set(BOOKSIM_GEN ${CMAKE_CURRENT_BINARY_DIR}/booksim2)
file(MAKE_DIRECTORY ${BOOKSIM_GEN})

add_custom_command(
  OUTPUT ${BOOKSIM_GEN}/y.tab.c ${BOOKSIM_GEN}/y.tab.h
  COMMAND ${BISON_EXECUTABLE} -y -d ${BOOKSIM_SRC}/config.y
  WORKING_DIRECTORY ${BOOKSIM_GEN}
  DEPENDS ${BOOKSIM_SRC}/config.y
)
add_custom_command(
  OUTPUT ${BOOKSIM_GEN}/lex.yy.c
  COMMAND ${FLEX_EXECUTABLE} ${BOOKSIM_SRC}/config.l
  WORKING_DIRECTORY ${BOOKSIM_GEN}
  DEPENDS ${BOOKSIM_SRC}/config.l ${BOOKSIM_GEN}/y.tab.h
)

file(GLOB BOOKSIM_CXX_SRCS ${BOOKSIM_SRC}/*.cpp ${BOOKSIM_SRC}/*/*.cpp)

add_library(ramulator-booksim OBJECT
  ${BOOKSIM_CXX_SRCS}
  ${BOOKSIM_GEN}/y.tab.c
  ${BOOKSIM_GEN}/lex.yy.c
  booksim_interconnect.cpp
)
target_include_directories(ramulator-booksim PRIVATE
  ${BOOKSIM_SRC} ${BOOKSIM_SRC}/arbiters ${BOOKSIM_SRC}/allocators ${BOOKSIM_SRC}/routers
  ${BOOKSIM_SRC}/networks ${BOOKSIM_SRC}/power ${BOOKSIM_GEN}
)
set_target_properties(ramulator-booksim PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS ON
  C_VISIBILITY_PRESET hidden
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  POSITION_INDEPENDENT_CODE ON
)
target_compile_options(ramulator-booksim PRIVATE -w)

target_link_libraries(
  ramulator
  PRIVATE
  ramulator-booksim
)
