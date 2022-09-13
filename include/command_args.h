#pragma once

#include <port_bus.h>
#include <address_bus.h>
#include <cpu/cpu.h>

#define COMMAND_LINE_OK (1)
#define COMMAND_LINE_INVALID_ARGUMENTS (2)
#define COMMAND_LINE_INVALID_CONFIG_FILE (3)
#define COMMAND_LINE_INVALID_INPUT_FILE (4)

typedef struct {
    char* program_path;
    char* cpu_config_path;
} command_args_info;

int parse_command_line_args(int argc, char** argv, command_args_info* cmd_info);