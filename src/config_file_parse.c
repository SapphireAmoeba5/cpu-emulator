#include "address_bus.h"
#include "port_bus.h"
#include <config_file_parse.h>
#include <inttypes.h>

#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/syslimits.h>
#include <unistd.h>
#include <errno.h>

typedef struct {
    void* handle;
    library_shutdown_callback shutdown_callback;
} library_handle_entry;

// An array that contains all the handles to each shared library loaded
static library_handle_entry* library_handles = NULL;
static size_t handles_length = 0;
static size_t handles_capacity = 0;

bool insert_handle(void* handle, library_shutdown_callback shutdown_callback) {
    if(handles_length >= handles_capacity) {
        library_handle_entry* new_buffer = realloc(library_handles, (handles_capacity + 10) * sizeof(library_handle_entry));
        if(new_buffer == NULL) {
            return false;
        }
        library_handles = new_buffer;
    }

    library_handle_entry entry;
    entry.handle = handle;
    entry.shutdown_callback = shutdown_callback;

    library_handles[handles_length] = entry;
    handles_length++;

    return true;
}

bool prepare_port_device_library(void* library_handle, const char* device_name, u16 port_number, port_bus_write_callback* write_callback, port_bus_read_callback* read_callback, library_shutdown_callback* shutdown_callback,void** private_data) {
    char* initialize_function_identifier = NULL;
    asprintf(&initialize_function_identifier, "%s_port_device_initialize", device_name);

    char* shutdown_function_identifier = NULL;
    asprintf(&shutdown_function_identifier, "%s_port_device_shutdown", device_name);

    char* write_callback_identifier = NULL;
    asprintf(&write_callback_identifier, "%s_port_device_write_callback", device_name);

    char* read_callback_identifier = NULL;
    asprintf(&read_callback_identifier, "%s_port_device_read_callback", device_name);

    library_port_device_initialize initialize_function = dlsym(library_handle, initialize_function_identifier);
    library_shutdown_callback shutdown_function = dlsym(library_handle, shutdown_function_identifier);

    port_bus_write_callback write_callback_function = dlsym(library_handle, write_callback_identifier);
    port_bus_read_callback read_callback_function = dlsym(library_handle, read_callback_identifier);

    free(initialize_function_identifier);
    free(shutdown_function_identifier);
    free(write_callback_identifier);
    free(read_callback_identifier);

    void* returned_private_data = NULL;
    if(initialize_function != NULL) {
        bool result = initialize_function(port_number, &returned_private_data);

        if(result != true) {
            printf("Error initializing port device with device name \"%s\"", device_name);
            return false;
        }
    }

    *write_callback = write_callback_function;
    *read_callback = read_callback_function;
    *shutdown_callback = shutdown_function;
    *private_data = returned_private_data;

    return true;
}

// Takes ownership of the libray handle.
bool register_new_port_device(void* library_handle, u16 port_number, const char* device_name, port_bus* prt_bus) {
    void* private_data;
    port_bus_write_callback write_callback;
    port_bus_read_callback read_callback;
    library_shutdown_callback shutdown_callback;

    bool result = prepare_port_device_library(library_handle, device_name, port_number, &write_callback, &read_callback, &shutdown_callback, &private_data);
    if(result != true) {
        return false;
    }

    result = insert_handle(library_handle, shutdown_callback);
    if(result != true) {
        return false;
    }

    return port_bus_add_device(prt_bus, port_number, write_callback, read_callback, private_data);
}

bool parse_new_port_device(char** arguments, size_t arguments_length, size_t line_number, port_bus* prt_bus) {
    // The correct amount of arguments a port device needs to be properly configured. Any more or less is an error
    const int port_device_argument_count = 3;

    if(arguments_length == 0) {
        printf("Not enough arguments. Port device requries arguments in the format <library_path> <port_number> <port_identifier> :: line:%zu\n", line_number);
        return false;
    }

    if(arguments_length > port_device_argument_count) {
        printf("Too many arguments. Port device requries arguments in the format <library_path> <port_number> <port_identifier> :: line:%zu\n", line_number);
        return false;
    }


    const char* path_to_library = arguments[0];
    const char* device_name = arguments[2];

    char* end_ptr = NULL;
    errno = 0;
    size_t port_number = strtoumax(arguments[1], &end_ptr, 10);

    if(errno != 0) {
        printf("Port number is not a valid number :: line:%zu\n", line_number);
        return false;
    }

    if(port_number > 0xffff) {
        printf("Port number is too large. Allowed port number range is 0-65535 :: line:%zu\n", line_number);
        return false;
    }

    void* library_handle = dlopen(path_to_library, RTLD_LAZY);

    return register_new_port_device(library_handle, (u16)port_number, device_name, prt_bus);
}

bool prepare_address_device_library(void* library_handle, const char* device_name, u64 base_address, u64 length, address_bus_write_callback_function* write_callback, address_bus_read_callback_function* read_callback, library_shutdown_callback* shutdown_callback, void** private_data) {
    char* initialize_function_identifier = NULL;
    asprintf(&initialize_function_identifier, "%s_address_device_initialize", device_name);

    char* shutdown_function_identifier = NULL;
    asprintf(&shutdown_function_identifier, "%s_address_device_shutdown", device_name);

    char* write_callback_identifier = NULL;
    asprintf(&write_callback_identifier, "%s_address_device_write_callback", device_name);

    char* read_callback_identifier = NULL;
    asprintf(&read_callback_identifier, "%s_address_device_read_callback", device_name);

    library_address_device_initialize initialize_function = dlsym(library_handle, initialize_function_identifier);
    library_shutdown_callback shutdown_function = dlsym(library_handle, shutdown_function_identifier);

    address_bus_write_callback_function write_callback_function = dlsym(library_handle, write_callback_identifier);
    address_bus_read_callback_function read_callback_function = dlsym(library_handle, read_callback_identifier);

    free(initialize_function_identifier);
    free(shutdown_function_identifier);
    free(write_callback_identifier);
    free(read_callback_identifier);

    void* returned_private_data = NULL;
    if(initialize_function != NULL) {
        bool result = initialize_function(base_address, length, &returned_private_data);

        if(result != true) {
            printf("Error initializing port device with device name \"%s\"", device_name);
            return false;
        }
    }

    *write_callback = write_callback_function;
    *read_callback = read_callback_function;
    *shutdown_callback = shutdown_function;
    *private_data = returned_private_data;

    return true;
}

bool register_new_address_device(void* library_handle, u64 base_address, u64 length, const char* device_name, address_bus* addr_bus) {
    address_bus_write_callback_function write_callback;
    address_bus_read_callback_function read_callback;
    library_shutdown_callback shutdown_callback;
    void* private_data;

    bool result = prepare_address_device_library(library_handle, device_name, base_address, length, &write_callback, &read_callback, &shutdown_callback, &private_data);

    if(result != true) {
        return false;
    }

    result = insert_handle(library_handle, shutdown_callback);

    if(result != true) {
        return false;
    }

    return address_bus_add_device(addr_bus, base_address, length, private_data, write_callback, read_callback);
}

bool parse_new_address_device(char** arguments, size_t arguments_length, size_t line_number, address_bus* addr_bus) {
    // The corrent amount of arguments an address device needs to be properly configured. Any more or less is an error
    const int address_device_argument_count = 4;

    if(arguments_length != address_device_argument_count) {
        printf("Invalid number of arguments. An address device requires %d arguments <library_path> <base_address> <length> <device_name> :: line:%zu\n", address_device_argument_count, line_number);
        return -1;
    }

    const char* path_to_library = arguments[0];
    const char* device_name = arguments[3];

    errno = 0;
    char* end_ptr;
    size_t base_address = strtoumax(arguments[1], &end_ptr, 10);

    if(errno != 0) {
        printf("Expected a valid number for base address :: line:%zu", line_number);
        return -1;
    }

    errno = 0;
    size_t length = strtoumax(arguments[2], &end_ptr, 10);
    
    if(errno != 0) {
        printf("Expected a valid number for the length :: line:%zu", line_number);
        return -1;
    }

    void* library_handle = dlopen(path_to_library, RTLD_LAZY);

    return register_new_address_device(library_handle, base_address, length, device_name, addr_bus);
}

bool parse_cpu_config_file_line(char* line, size_t line_number, address_bus* addr_bus, port_bus* prt_bus) {
    char* arguments[16];
    size_t arguments_length = 0;

    char* token;
    while((token = strsep(&line, " ")) != NULL) {
        if(arguments_length >= 16) {
            printf("Config file line %zu: Maximum number of arguments is 16\n", line_number);
            return false;
        }

        if(strlen(token) != 0) {
            arguments[arguments_length] = token;
            arguments_length++;
        }
    }

    if(strcmp("port-device", arguments[0]) == 0) {
        bool result = parse_new_port_device(&arguments[1], arguments_length - 1, line_number, prt_bus);
        if(result != true) {
            return false;
        }
    }
    else if(strcmp("address-device", arguments[0]) == 0) {
        bool result = parse_new_address_device(&arguments[1], arguments_length - 1, line_number, addr_bus);
        if(result != true) {
            return false;
        }
    }
    else {
        printf("Invalid device!\n");
        return false;
    }

    return true;
}

bool parse_cpu_config_string(char* config_file, address_bus* addr_bus, port_bus* prt_bus) {
    size_t line_number = 0;
    char* token;
    while((token = strsep(&config_file, "\n")) != NULL) {
        if(strlen(token) != 0) {
            line_number++;
            bool ret = parse_cpu_config_file_line(token, line_number, addr_bus, prt_bus);

            if(ret == false) {
                return false;
            }
        }
    }

    return true;
}

bool parse_cpu_config_file(const char* path, address_bus* addr_bus, port_bus* prt_bus) {
    FILE* config_file = fopen(path, "r");

    fseek(config_file, 0, SEEK_END);
    size_t file_size = ftell(config_file);
    rewind(config_file);

    if(config_file == NULL) {
        printf("Cannot open config file at \"%s\"\n", path);
        return false;
    }
    
    char original_working_directory[PATH_MAX];
    getcwd(original_working_directory, PATH_MAX);
        
    chdir(path);

    char* config_file_contents = malloc(file_size + 1);
    config_file_contents[file_size] = 0;

    fread(config_file_contents, 1, file_size, config_file);
    fclose(config_file);

    bool result = parse_cpu_config_string(config_file_contents, addr_bus, prt_bus);

    free(config_file_contents);
    chdir(original_working_directory);
    return result;
}

void free_config_handles() {
    for(size_t i = 0; i < handles_length; i++) {
        library_handle_entry entry = library_handles[i];
        if(entry.shutdown_callback != NULL) {
            entry.shutdown_callback();
        }
        dlclose(entry.handle);
    }
    free(library_handles);

    library_handles = NULL;
    handles_length = 0;
    handles_capacity = 0;
}