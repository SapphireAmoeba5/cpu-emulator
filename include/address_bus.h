#pragma once

#include <stdlib.h>
#include <types.h>
#include <stdbool.h>

// Private data is a pointer to any type of data that only the callback function can use. An example of usage can be is a pointer to struct data.
typedef void(*address_bus_read_callback_function)(void* dest, u64 addr, u64 offset, size_t len, void* private_data);
typedef void(*address_bus_write_callback_function)(void* src, u64 addr, u64 offset, size_t len, void* private_data);

typedef struct address_bus_entry_impl {
    u64 address;
    size_t length;
    address_bus_read_callback_function read_callback_func;
    address_bus_write_callback_function write_callback_func;
    void* private_data;
} address_bus_entry;

typedef struct address_bus_impl address_bus;

address_bus_entry create_address_bus_entry(u64 address, size_t length, void* private_data, address_bus_write_callback_function write_callback, address_bus_read_callback_function read_callback);

address_bus* create_address_bus();
void destroy_address_bus(address_bus* addr_bus);

void address_bus_write(address_bus* addr_bus, void* data, u64 address, size_t len);  
void address_bus_read(address_bus* addr_bus, void* data, u64 address, size_t len); 

bool address_bus_add_device(address_bus* addr_bus, u64 address, u64 len, void* private_data, address_bus_write_callback_function write_callback, address_bus_read_callback_function read_callback);

bool address_bus_insert(address_bus* addr_bus, u64 where, address_bus_entry* entry);

void print_entries(address_bus* addr_bus);