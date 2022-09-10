#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "address_bus.h"
#include <debug.h>
#include <types.h>

#define MAX_ENTRY_LENGTH 64
#define ENTRY_NOT_FOUND ((size_t)-1)



typedef struct address_bus_impl {
    size_t entries_length;
    address_bus_entry entries[MAX_ENTRY_LENGTH];
} address_bus;

address_bus_entry create_address_bus_entry(u64 address, size_t length, void* private_data, write_callback_function write_callback, read_callback_function read_callback) {
    address_bus_entry e;

    e.address = address;
    e.length = length;
    e.read_callback_func = read_callback;
    e.write_callback_func = write_callback;
    e.private_data = private_data;

    return e;
}

// Returns ENTRY_NOT_FOUND if no entry is found at the target address
size_t search_for_address(address_bus* addr_bus, u64 target_address, u64 length ) {

    // We use linear search for now
    for(int i = 0; i < addr_bus->entries_length; i++) {
        address_bus_entry* entry = &addr_bus->entries[i];

        if(target_address >= entry->address && target_address <= entry->address + entry->length) {
            // DEBUG_PRINT("Found address: %p". (void*)entry->address);
        }
    }

    return 0;

    // size_t start = 0;
    // size_t mid = addr_bus->entries_length / 2;
    // size_t end = addr_bus->entries_length;

    // while(start < end) {
    //     address_bus_entry* entry = &addr_bus->entries[mid];

    //     if(target_address >= entry->address && target_address <= entry->address + entry->length) {
    //         return mid;
    //     } else if(target_address > entry->address) {
    //         end = mid;
    //         mid = start + ((end - start) / 2);
    //     } else if(target_address < entry->address) {
    //         end = mid + 1;
    //         mid = start + ((end - start) / 2);
    //     }
    // }
    
    return ENTRY_NOT_FOUND;
}

bool address_bus_insert(address_bus* addr_bus, u64 where, address_bus_entry* entry) {
    if(addr_bus->entries_length >= MAX_ENTRY_LENGTH) {
        return false;
    }

    for(int i = addr_bus->entries->length; i > where; i--) {
        addr_bus->entries[i] = addr_bus->entries[i - 1];
    }

    addr_bus->entries[where] = *entry;

    addr_bus->entries_length += 1;

    return true;
}

address_bus* create_address_bus() {
    address_bus* addr_bus = calloc(1, sizeof(address_bus));
    return addr_bus;
}

void destroy_address_bus(address_bus* addr_bus) {
    free(addr_bus);
}

bool address_bus_add_device(address_bus* addr_bus, u64 address, u64 len, void* private_data, write_callback_function write_callback, read_callback_function read_callback) {
    if(len == 0) {
        return true;
    }
    
    // Check for overlapping address
    for(int i = 0; i < addr_bus->entries_length; i++) {
        address_bus_entry* entry = &addr_bus->entries[i];
        if(address >= entry->address && address < entry->address + entry->length || entry->address >= address && entry->address < address + len) {
            return false;
        }
    }

    bool found_where = false;
    size_t i;
 
    for(i = 0; i < addr_bus->entries_length; i++) {
        if(i > 0) {
            if(addr_bus->entries[i - 1].address < address && addr_bus->entries[i].address >= address) {
                address_bus_entry e = create_address_bus_entry(address, len, private_data, write_callback, read_callback);
                if(address_bus_insert(addr_bus, i, &e) == false) {
                    return false;
                }
                found_where = true;
            }
        }
    }
    if(!found_where) {
        address_bus_entry e = create_address_bus_entry(address, len, private_data, write_callback, read_callback);

        DEBUG_PRINT("Inserting address bus entry at %p with length %lu\n", (void*)address, len);
        return address_bus_insert(addr_bus, i, &e);
    }

    return true;
}

void address_bus_write(address_bus* addr_bus, void* data, u64 address, size_t len) {
    for(size_t i = 0; i < addr_bus->entries_length; i++) {
        address_bus_entry* e = &addr_bus->entries[i];

        if(address >= e->address && address < e->address + e->length) {
            u64 target_length = len > e->address + e->length - address ? e->address + e->length - address : len;
            u64 offset = address - e->address;

            if(e->write_callback_func != NULL) {
                e->write_callback_func(data, address, offset, target_length, e->private_data);
            }
        } else if(e->address > address && e->address < address + len) {
            u64 relative_length = len - (e->address - address);

            u64 target_length = relative_length > e->length ? e->length : relative_length;
            u64 offset = 0;

            if(e->write_callback_func != NULL) {
                e->write_callback_func((char*)data + (e->address - address), e->address, offset, target_length, e->private_data);
            }
        }
    }
}

void address_bus_read(address_bus* addr_bus, void* data, u64 address, size_t len) {
    memset(data, 0xff, len); 

    for(size_t i = 0; i < addr_bus->entries_length; i++) {
        address_bus_entry* e = &addr_bus->entries[i];

        if(address >= e->address && address < e->address + e->length) {
            u64 target_length = len > e->address + e->length - address ? e->address + e->length - address : len;
            u64 offset = address - e->address;

            if(e->read_callback_func != NULL) {
                e->read_callback_func(data, address, offset, target_length, e->private_data);
            }

        } else if(e->address > address && e->address < address + len) {
            u64 relative_length = len - (e->address - address);

            u64 target_length = relative_length > e->length ? e->length : relative_length;
            u64 offset = 0;

            if(e->read_callback_func != NULL) {
                e->read_callback_func((char*)data + (e->address - address), e->address, offset, target_length, e->private_data);
            }
        }
    }
}

void print_entries(address_bus* addr_bus) {
    for(size_t i = 0; i < addr_bus->entries_length; i++) {
        address_bus_entry* e = &addr_bus->entries[i];

        printf("Address: %p (%lu) : Length: %lu : Write callback %p : Read callback %p\n", (void*)e->address, e->address, e->length, e->write_callback_func, e->read_callback_func);
    }
}