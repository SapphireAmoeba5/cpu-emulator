#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>

#include <debug.h>
#include <command_args.h>
#include <unistd.h>

int parse_command_line_args(int argc, char** argv, command_args_info* cmd_info) {
    cmd_info->program_path = NULL;
    cmd_info->cpu_config_path = NULL;

    for(int i = 1; i < argc; i++) {
        char* cur_arg = argv[i];

        if(strcmp(cur_arg, "--config") == 0) {
            if(i + 1 >= argc) {
                return COMMAND_LINE_INVALID_ARGUMENTS;
            }
            cmd_info->cpu_config_path = argv[i + 1];
            i++;
        } else {
            cmd_info->program_path = cur_arg;
        }
    }

    return COMMAND_LINE_OK;
}