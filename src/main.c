#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "cpu.h"
#include "debug.h"
#include "instructions.h"


int main(void) {
    DEBUG_PRINT("Running a debug build\n");

    CPU* cpu = create_cpu(1024 * 1024);

    uint64_t start_address = 0x1000;
    cpu_write(cpu, &start_address, 0, 8);

    uint64_t stack_address = 0x8000;
    cpu_write(cpu, &stack_address, 8, 8);

    cpu_reset(cpu);

    uint8_t* memory = cpu_raw_memory(cpu);

    //memory[0x1000] = 0x02;
    //memory[0x1001] = 0b11000001;

    //uint64_t data = 100;
    //cpu_write(cpu, &data, 0x1002, 8);

    //memory[0x100A] = 0x21;
    //memory[0x100B] = 0b00000001;

    //memory[0x100C] = 0x31;
    //memory[0x100D] = 0b00000010;

    srand((unsigned)time(0));

    for(int i = 0x1000; i < 1024 * 1024; i++) {
        memory[i] = rand() % 256;
    }


    while(is_halted(cpu) == false) {
        cpu_clock(cpu);
        DEBUG_EXECUTE(printf("\n"));
    }

    destroy_cpu(cpu);
}
