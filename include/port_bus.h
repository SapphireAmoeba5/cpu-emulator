#pragma once

#include "address_bus.h"

typedef struct port_bus_impl port_bus;

port_bus* port_bus_create(address_bus* addr_bus);
void port_bus_destroy(port_bus* port_bus);

bool port_bus_add_device(port_bus* port_bus, u)