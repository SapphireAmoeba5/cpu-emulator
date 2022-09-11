#include <stdlib.h>
#include <string.h>

#include <port_bus.h>

typedef struct port_bus_impl {
    port_bus_entry* ports[0xffff];
} port_bus;

port_bus* port_bus_create() {
    port_bus* new_port_bus = calloc(1, sizeof(port_bus));

    return new_port_bus;
}

void port_bus_destroy(port_bus* port_bus) {
    for (size_t i = 0; i < 0xffff; i++) {
        if (port_bus->ports[i] != NULL) {
            free(port_bus->ports[i]);
        }
    }

    free(port_bus);
}

bool port_bus_add_device(port_bus* port_bus, u16 port, port_bus_write_callback write_callback, port_bus_read_callback read_callback, void* private_data) {
    if (port_bus->ports[port] != NULL) {
        return false;
    }

    port_bus_entry* entry = malloc(sizeof(port_bus_entry));

    entry->write_callback = write_callback;
    entry->read_callback = read_callback;
    entry->private_data = private_data;

    port_bus->ports[port] = entry;

    return true;
}

void port_bus_remove_device(port_bus* port_bus, u16 port) {
    port_bus_entry* e = port_bus->ports[port];

    if (e != NULL) {
        free(e);
    }
}

void port_bus_write(port_bus* port_bus, u16 port, u64 value) {
    port_bus_entry* e = port_bus->ports[port];

    if (e != NULL && e->write_callback != NULL) {
        e->write_callback(value, e->private_data);
    }
}

void port_bus_read(port_bus* port_bus, u16 port, u64* dest) {
    port_bus_entry* e = port_bus->ports[port];

    if (e != NULL && e->read_callback != NULL) {
        e->read_callback(dest, e->private_data);
    }
    else if (dest != NULL) {
        *dest = 0xffffffffffffffff;
    }
}