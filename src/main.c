#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "cpu.h"
#include "debug.h"
#include "types.h"

int load_file_into_cpu_memory(CPU* cpu, size_t memory_size, const char* filepath) {
    FILE* file = fopen(filepath, "rb");

    size_t file_size;
    fseek(file, 0, SEEK_END);
    file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    DEBUG_PRINT("File size is %zu bytes\n", file_size);
    // Subtract 8 from file_size because the first 8 bytes is the entry point of the file.
    // and subtract 16 from memory_size because the first 16 bytes hold the entry point in memory, and the top of the stack.
    if(file_size - 8 >= (memory_size - 16)) {
        DEBUG_PRINT("File \"%s\" is too large to fit in memory", filepath);
        fclose(file);
        return 1;
    }

    u8* buffer = malloc(file_size);
    fread(buffer, 1, file_size, file);

    // This variable holds the offset into the file where the CPU should start execution
    u64 off_entry_point;
    memcpy(&off_entry_point, buffer, sizeof(u64));

    DEBUG_PRINT("File offset entry point %lu\n", off_entry_point);

    // Subtract 8 because the first 8 bytes aren't CPU instructions, and add 16 because the first 2 bytes in memory are reserved.
    u64 addr_entry_point = (off_entry_point - 8) + 16;
    DEBUG_PRINT("Address entry point %lu\n", addr_entry_point);
    cpu_write(cpu, &addr_entry_point, 0x00, sizeof(u64));

    u64 addr_stack = memory_size - 1;
    DEBUG_PRINT("Top of stack %lu\n", addr_stack);
    cpu_write(cpu, &addr_stack, 0x08, sizeof(u64));

    cpu_write(cpu, buffer + 8, 16, file_size - 8);


    free(buffer);
    fclose(file);

    return 0;
}

int main(int argc, char** argv) {
    DEBUG_PRINT("Running a debug build\n");

    const size_t memory_size = 1024 * 1024;
    CPU* cpu = create_cpu(memory_size);

    if(argc == 2) {
        DEBUG_EXECUTE(printf("\n"));

        load_file_into_cpu_memory(cpu, memory_size, argv[1]);

        DEBUG_EXECUTE(printf("\n"));
    }
    else if(argc != 1) {
        printf("Invalid use of command line arguments\n");
        return 1;
    }

    cpu_reset(cpu);

    DEBUG_EXECUTE(printf("\n"));

    while(is_halted(cpu) == false) {
        cpu_clock(cpu);

        if(is_halted(cpu) == false) {
            DEBUG_EXECUTE(printf("\n"));
        }
    }
    destroy_cpu(cpu);
}
