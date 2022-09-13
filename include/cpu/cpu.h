#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "types.h"
#include <address_bus.h>
#include <port_bus.h>

typedef struct impl_cpu CPU;

CPU* create_cpu(address_bus* addr_bus, port_bus* port_bus);
void destroy_cpu(CPU* cpu);
void cpu_reset(CPU* cpu);
void cpu_clock(CPU* cpu);
bool is_halted(CPU* cpu);

void cpu_write(CPU* cpu, void* data, u64 address, size_t size);

void cpu_read(CPU* cpu, void* dest, u64 address, size_t size);

address_bus* cpu_get_address_bus(CPU* cpu);
port_bus* cpu_get_port_bus(CPU* cpu);

void print_registers(CPU* cpu);