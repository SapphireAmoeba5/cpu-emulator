cmake_minimum_required(VERSION 3.24 FATAL_ERROR)

project(cpu-emulator)

set(CMAKE_C_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

set(SRC src)
set(SRCS ${SRC}/main.c ${SRC}/cpu.c ${SRC}/address_bus.c ${SRC}/memory.c ${SRC}/port_bus.c ${SRC}/command_args.c ${SRC}/config_file_parse.c)

set(INCLUDE include)

add_executable(cpu-emulator ${SRCS})

target_include_directories(cpu-emulator PRIVATE ${INCLUDE})

target_link_libraries(cpu-emulator pthread)

install(TARGETS cpu-emulator)