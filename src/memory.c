#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <memory.h>
#include <debug.h>

#define MEMORY_SIZE 0x10000000

typedef struct memory_impl {
    u8* data;
    address_bus* addr_bus;
} memory;

void memory_write_callback(void* src, u64 address, u64 offset, size_t len, void* private_data) {
    memory* mem = (memory*)private_data;
    
    memcpy(&mem->data[offset], src, len);
}

void memory_read_callback(void* dest, u64 address, u64 offset, size_t len, void* private_data) {
    memory* mem = (memory*)private_data;

    memcpy(dest, &mem->data[offset], len);
}

memory* memory_create(address_bus* addr_bus) {
    memory* mem = calloc(1, sizeof(memory));

    mem->addr_bus = addr_bus;
    mem->data = calloc(MEMORY_SIZE, 1);

    address_bus_add_device(addr_bus, 0x0, MEMORY_SIZE, mem, memory_write_callback, memory_read_callback);

    return mem;
}

void memory_destroy(memory* mem) {
    free(mem->data);
}