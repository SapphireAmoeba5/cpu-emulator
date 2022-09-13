#include "command_args.h"
#include "port_bus.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <sys/syslimits.h>

#include <cpu/cpu.h>
#include <debug.h>
#include <types.h>
#include <address_bus.h>
#include <memory.h>
#include <config_file_parse.h>

bool load_program_into_cpu_memory(CPU* cpu, u8* program, size_t program_length, size_t entry_point) {
    // Convert the entry point into an address to copy the program to
    u64 addr_entry_point = entry_point+ 16;
    DEBUG_PRINT("Address entry point %llu\n", addr_entry_point);
    cpu_write(cpu, &addr_entry_point, 0x00, sizeof(u64));

    // The default stack address is an arbitrary value. Soon this may be hardcoded into the cpu directly
    u64 addr_stack = 0xff0000;
    DEBUG_PRINT("Top of stack %llu\n", addr_stack);
    cpu_write(cpu, &addr_stack, 0x08, sizeof(u64));

    cpu_write(cpu, program, 16, program_length);

    return true;
}

bool load_file_into_cpu_memory(CPU* cpu, const char* filepath) {
    FILE* file = fopen(filepath, "rb");

    if(file == NULL) {
        printf("Cannot open file\n");
        return false;
    }

    size_t file_size;
    fseek(file, 0, SEEK_END);
    file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    DEBUG_PRINT("File size is %zu bytes\n", file_size);

    u8* buffer = malloc(file_size);
    fread(buffer, 1, file_size, file);

    u64 off_entry_point;
    memcpy(&off_entry_point, buffer, sizeof(u64));

    DEBUG_PRINT("File offset entry point %llu\n", off_entry_point);

    /* Add 8 to buffer before passing into this function to get rid of the first 8 bytes, which is the entry point of the program, 
       which this function doesn't need */
    bool result = load_program_into_cpu_memory(cpu, buffer + 8, file_size - 8, off_entry_point - 8);

    free(buffer);
    fclose(file);

    return result;
}

bool initialize_cpu_from_arguments(CPU* cpu, command_args_info* cmd_info) {
    if(cmd_info->cpu_config_path != NULL) {
        if(parse_cpu_config_file(cmd_info->cpu_config_path, cpu_get_address_bus(cpu), cpu_get_port_bus(cpu)) != true) {
            return false;
        }
    }

    if(cmd_info->program_path != NULL) {
        if(load_file_into_cpu_memory(cpu, cmd_info->program_path) != true) {
            return false;
        } 
    }

    return true;
}

int main(int argc, char** argv) {
    DEBUG_PRINT("Running a debug build\n");

    address_bus* addr_bus = create_address_bus();
    port_bus* port_bus = port_bus_create();
    
    CPU* cpu = create_cpu(addr_bus, port_bus);

    command_args_info cmd_info;
    parse_command_line_args(argc, argv, &cmd_info);

    if(initialize_cpu_from_arguments(cpu, &cmd_info) != true) {
        destroy_address_bus(addr_bus);
        port_bus_destroy(port_bus);
        destroy_cpu(cpu);
        free_config_handles();

        return -1;
    }

    cpu_reset(cpu);
    
    DEBUG_EXECUTE(printf("\n"));

    while(1) {
        cpu_clock(cpu);
        if(is_halted(cpu) == false) {
            DEBUG_EXECUTE(printf("\n"));
        }
    }
    
    destroy_cpu(cpu);
    destroy_address_bus(addr_bus);
    port_bus_destroy(port_bus);
    free_config_handles();
}
