#pragma once

#include <address_bus.h>

typedef struct memory_impl memory;

memory* memory_create(address_bus* addr_bus);
void memory_destroy(memory* mem);