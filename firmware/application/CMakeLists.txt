cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
list(APPEND EXTRA_ZEPHYR_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/dummysensor
  )

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
add_subdirectory(lib)

project(fjalar)
target_sources(app PRIVATE
    src/flight_state.c
    src/sensors.c
    src/telecommand.c
    src/telemetry.c
    src/actuation.c
    src/main.c
    src/filter.c
)

# target_link_libraries(app filter) # This line is important to specify MyLib is used by app