#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct impl_cpu CPU;

CPU* create_cpu();
void destroy_cpu(CPU* cpu);
void cpu_reset(CPU* cpu);
void cpu_clock(CPU* cpu);
bool is_halted(CPU* cpu);

void cpu_write(CPU* cpu, void* data, size_t address, size_t size);

void cpu_read(CPU* cpu, void* dest, size_t address, size_t size);

uint8_t* cpu_raw_memory(CPU* cpu);
void dump_cpu_memory(CPU* cpu, const char* filename);
