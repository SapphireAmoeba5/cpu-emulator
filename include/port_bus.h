#pragma once

#include <address_bus.h>
#include <types.h>

typedef void(*port_bus_write_callback)(u64 value, void* private_data);
typedef void(*port_bus_read_callback)(u64* dest, void* private_data);

typedef struct {
    port_bus_write_callback write_callback;
    port_bus_read_callback read_callback;
    void* private_data;
} port_bus_entry;

typedef struct port_bus_impl port_bus;

port_bus* port_bus_create();
void port_bus_destroy(port_bus* port_bus);

bool port_bus_add_device(port_bus* port_bus, u16 port, port_bus_write_callback write_callback, port_bus_read_callback read_callback, void* private_data);
void port_bus_remove_device(port_bus*  port_bus, u16 port);

void port_bus_write(port_bus* port_bus, u16 port, u64 value);
void port_bus_read(port_bus* port_bus, u16 port, u64* dest);