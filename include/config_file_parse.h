#pragma once

#include <address_bus.h>
#include <port_bus.h>

typedef bool(*library_address_device_initialize)(u64 base_address, u64 length, void** private_data);
typedef bool(*library_port_device_initialize)(u16 port, void** private_data);

typedef void(*library_shutdown_callback)();


bool parse_cpu_config_string(char* config, address_bus* address_bus, port_bus* port_bus);
bool parse_cpu_config_file(const char* path, address_bus* address_bus, port_bus* port_bus);

// This should only be called at the end of the program, when the CPU will not be executing anymore instructions
void free_config_handles();