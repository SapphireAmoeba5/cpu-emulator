#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "types.h"

typedef struct {
    size_t memory_size;
    u8* memory;

    bool halted;

    u64 x0, x1, x2, x3, x4;
    u64 ip;
    u64 sp;

    u8 flag_negative;
    u8 flag_overflow;
    u8 flag_zero;
    u8 flag_carry;
} cpu_state;

typedef struct impl_cpu CPU;

CPU* create_cpu(size_t memory_size);
void destroy_cpu(CPU* cpu);
void cpu_reset(CPU* cpu);
void cpu_clock(CPU* cpu);
bool is_halted(CPU* cpu);

void cpu_write(CPU* cpu, void* data, size_t address, size_t size);

void cpu_read(CPU* cpu, void* dest, size_t address, size_t size);

uint8_t* cpu_raw_memory(CPU* cpu);
void dump_cpu_memory(CPU* cpu, const char* filename);
void print_registers(CPU* cpu);
void get_cpu_state(CPU* cpu, cpu_state* state);
