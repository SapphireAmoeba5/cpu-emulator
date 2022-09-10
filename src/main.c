#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#include <cpu/cpu.h>
#include <debug.h>
#include <types.h>

#include <address_bus.h>
#include <memory.h>

int load_file_into_cpu_memory(CPU* cpu, const char* filepath) {
    FILE* file = fopen(filepath, "rb");

    if(file == NULL) {
        printf("Cannot open file\n");
        return 1;
    }

    size_t file_size;
    fseek(file, 0, SEEK_END);
    file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    DEBUG_PRINT("File size is %zu bytes\n", file_size);

    u8* buffer = malloc(file_size);
    fread(buffer, 1, file_size, file);

    // This variable holds the offset into the file where the CPU should start execution
    u64 off_entry_point;
    // Read the first 8 bytes of the file, containing the file entry point offset
    memcpy(&off_entry_point, buffer, sizeof(u64));

    DEBUG_PRINT("File offset entry point %lu\n", off_entry_point);

    // Subtract 8 because the first 8 bytes aren't CPU instructions, and add 16 because the first 2 bytes in memory are reserved.
    u64 addr_entry_point = (off_entry_point - 8) + 16;
    DEBUG_PRINT("Address entry point %lu\n", addr_entry_point);
    cpu_write(cpu, &addr_entry_point, 0x00, sizeof(u64));

    u64 addr_stack = 0xff0000;
    DEBUG_PRINT("Top of stack %lu\n", addr_stack);
    cpu_write(cpu, &addr_stack, 0x08, sizeof(u64));

    cpu_write(cpu, buffer + 8, 16, file_size - 8);


    free(buffer);
    fclose(file);

    return 0;
}

int main(int argc, char** argv) {
    DEBUG_PRINT("Running a debug build\n");

    address_bus* addr_bus = create_address_bus();
    memory* mem = memory_create(addr_bus);
    
    CPU* cpu = create_cpu(addr_bus);

    if(argc == 2) {
        DEBUG_EXECUTE(printf("\n"));

        load_file_into_cpu_memory(cpu, argv[1]);

        DEBUG_EXECUTE(printf("\n"));
    }
    else if(argc != 1) {
        printf("Invalid use of command line arguments\n");
        return 1;
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
    memory_destroy(mem);
    destroy_address_bus(addr_bus);
}
