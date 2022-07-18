#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "cpu.h"
#include "debug.h"
#include "types.h"

int main(void) {
    DEBUG_PRINT("Running a debug build\n");

    CPU* cpu = create_cpu(1024 * 1024);

    uint64_t start_address = 0x1000;
    cpu_write(cpu, &start_address, 0, 8);

    uint64_t stack_address = 0x8000;
    cpu_write(cpu, &stack_address, 8, 8);

    cpu_reset(cpu);

    uint8_t* memory = cpu_raw_memory(cpu);

    u8 program[] = {0x02, 0b11000001, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x14, 0b11010010,
                    0x14, 0b11011011,
                    0x34, 0b11000011,
                    0x41, 0b00000011, 0b00001010, 0b00000000, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                    0x03, 0b11010000, 0x01, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                    0x11, 0b11010000, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x25, 0b00000000, 0b00000000, 0x10, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    cpu_write(cpu, program, 0x1000, sizeof(program));

   // srand((unsigned)time(0));

   // for(int i = 0x1000; i < 1024 * 1024; i++) {
   //     memory[i] = rand() % 256;
   // }

    while(is_halted(cpu) == false) {
        cpu_clock(cpu);
        DEBUG_EXECUTE(printf("\n"));
    }

    dump_cpu_memory(cpu, "mem_dump.bin");

    destroy_cpu(cpu);
}
