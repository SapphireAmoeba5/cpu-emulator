#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>

#include "address_bus.h"
#include <debug.h>
#include <types.h>

#define MAX_ENTRY_LENGTH 64
#define ENTRY_NOT_FOUND ((size_t)-1)

typedef void(*read_callback_t)(void* dest, u64 addr, size_t len);
typedef void(*write_callback_t)(void* src, u64 addr, size_t len);

typedef struct {
    u64 address;
    size_t length;
    read_callback_t read_callback_func;
    write_callback_t write_callback_func;
} address_bus_entry;

typedef struct address_bus_impl {
    size_t entries_length;
    address_bus_entry entries[MAX_ENTRY_LENGTH];
} address_bus;

// Returns ENTRY_NOT_FOUND if no entry is found at the target address
size_t search_for_address(address_bus* addr_bus, u64 target_address) {
    size_t start = 0;
    size_t mid = addr_bus->entries_length / 2;
    size_t end = addr_bus->entries_length;

    while(start < end) {
        address_bus_entry* entry = &addr_bus->entries[mid];

        if(target_address >= entry->address && target_address <= entry->address + entry->length) {
            return mid;
        } else if(target_address > entry->address) {
            end = mid;
            mid = start + ((end - start) / 2);
        } else if(target_address < entry->address) {
            end = mid + 1;
            mid = start + ((end - start) / 2);
        }
    }

    return ENTRY_NOT_FOUND;
}

address_bus* create_address_bus() {
    address_bus* addr_bus = calloc(1, sizeof(address_bus));
    return addr_bus;
}

void address_bus_write(address_bus* addr_bus, void* data, u64 addr, size_t len) {
    size_t entry_index = search_for_address(addr_bus, addr);

    if(entry_index == ENTRY_NOT_FOUND) {
        return;
    }

    address_bus_entry* addr_bus_entry = &addr_bus->entries[entry_index];


}
