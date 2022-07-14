#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "cpu.h"
#include "debug.h"
#include "instructions.h"

// Functions declarations
uint8_t fetch_byte(CPU* cpu);
uint16_t fetch_word(CPU* cpu);
uint32_t fetch_dword(CPU* cpu);
uint64_t fetch_qword(CPU* cpu);

// XXX refers to an invalid instruction opcode
void XXX(CPU* cpu);

void HLT(CPU* cpu);
void MOV(CPU* cpu);
void PLC(CPU* cpu);

void ADD(CPU* cpu);
void SUB(CPU* cpu);
void MUL(CPU* cpu);
void DIV(CPU* cpu);

void OR(CPU* cpu);
void XOR(CPU* cpu);
void AND(CPU* cpu);
void NOT(CPU* cpu);
void NEG(CPU* cpu);

void CMP(CPU* cpu);
void PUSH(CPU* cpu);
void POP(CPU* cpu);

void NOP(CPU* cpu);

typedef struct {
    const char* mnemonic;
    void(*operation)(CPU* cpu);
} instruction;

static instruction instruction_lookup[256] = {
   //           0             1             2             3             4             5             6             7             8             9             A             B             C             D             E             F
   /* 0 */ {"HLT", HLT}, {"MOV", MOV}, {"PLC", PLC}, {"ADD", ADD},  {"OR", OR},  {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 1 */ {"XXX", XXX}, {"CMP", CMP}, {"XXX", XXX}, {"SUB", SUB}, {"XOR", XOR}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 2 */ {"XXX", XXX}, {"PUSH", PUSH}, {"XXX", XXX}, {"MUL", MUL}, {"AND", AND}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 3 */ {"XXX", XXX}, {"POP", POP}, {"XXX", XXX}, {"DIV", DIV}, {"NOT", NOT}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 4 */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"NEG", NEG}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 5 */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 6 */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 7 */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 8 */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 9 */ {"NOP", NOP}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* A */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* B */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* C */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* D */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* E */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* F */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}
};


typedef struct impl_cpu {
    size_t memory_size; /* Size of memory in bytes */
    uint8_t* memory;

    uint64_t fetched; /* Last fetched value */
    bool halted;

    uint64_t x0, x1, x2, x3, x4; /* General purpose registers. Can be used however the programmer wants */
    uint64_t sp; /* Stack pointer */
    uint64_t ip; /* Instruction pointer */

    /* CPU flags */
    uint8_t flag_negative: 1;
    uint8_t flag_overflow: 1;
    uint8_t flag_zero: 1;
    uint8_t flag_carry: 1;
} CPU;

CPU* create_cpu(size_t memory_size) {
    CPU* cpu = calloc(1, sizeof(CPU));

    cpu->memory = calloc(memory_size, 1);
    cpu->memory_size = memory_size;

    DEBUG_PRINT("Created cpu(%p) with memory size of %zu\n", cpu, memory_size);
    return cpu;
}

void destroy_cpu(CPU* cpu) {
    DEBUG_PRINT("Destroyed cpu(%p)\n", cpu);

    free(cpu->memory);
    free(cpu);
}

void cpu_reset(CPU* cpu) {
    DEBUG_PRINT("Resetting cpu(%p)\n", cpu);

    cpu->x0 = 0;
    cpu->x1 = 0;
    cpu->x2 = 0;
    cpu->x3 = 0;
    cpu->x4 = 0;

    // Resetting ip and sp are pointless because they get overwritten later
    cpu->ip = 0;
    cpu->sp = 0;

    cpu->fetched = 0;
    cpu->halted = false;

    cpu->flag_negative = 0;
    cpu->flag_overflow = 0;
    cpu->flag_zero = 0;
    cpu->flag_carry = 0;

    // CPU reads 8 bytes from address 0x0 to find an address to jump to in order to start executing
    cpu_read(cpu, &cpu->ip, 0x0, 8);
    DEBUG_PRINT("Beginning execution at %#lx\n", cpu->ip);

    // CPU reads 8 bytes from address 0x8 to find the top of the stack
    cpu_read(cpu, &cpu->sp, 0x8, 8);
    DEBUG_PRINT("Top of stack %#lx\n", cpu->sp);
}

void cpu_clock(CPU* cpu) {
    if(cpu->halted == true) {
        return;
    }

    uint8_t opcode = fetch_byte(cpu);

    DEBUG_PRINT("Executing instruction %s opcode %#x\n", instruction_lookup[opcode].mnemonic, opcode);

    instruction_lookup[opcode].operation(cpu);
}

uint8_t* cpu_raw_memory(CPU* cpu) {
    return cpu->memory;
}

void dump_cpu_memory(CPU* cpu, const char* filename) {
    FILE* file = fopen(filename, "wb");
    fwrite(cpu->memory, 1, cpu->memory_size, file);
    fclose(file);
}

bool is_halted(CPU* cpu) {
    return cpu->halted;
}

// Fetch a byte at the address of the instruction pointer
uint8_t fetch_byte(CPU* cpu) {
    uint8_t byte;
    cpu_read(cpu, &byte, cpu->ip, 1);
    cpu->fetched = byte;
    cpu->ip++;
    return byte;
}

// Fetch a word (2 bytes) at the address of the instruction pointer
uint16_t fetch_word(CPU* cpu) {
    uint16_t word;
    cpu_read(cpu, &word, cpu->ip, 2);
    cpu->fetched = word;
    cpu->ip += 2;
    return word;
}

// Fetch a dword (4 bytes) at the address of the instruction pointer
uint32_t fetch_dword(CPU* cpu) {
    uint32_t dword;
    cpu_read(cpu, &dword, cpu->ip, 4);
    cpu->fetched = dword;
    cpu->ip += 4;
    return dword;
}

// Fetch a qword (8 bytes) at the address of the instruction pointer
uint64_t fetch_qword(CPU* cpu) {
    uint64_t qword;
    cpu_read(cpu, &qword, cpu->ip, 8);
    cpu->fetched = qword;
    cpu->ip += 8;
    return qword;
}

void cpu_write(CPU* cpu, void* data, size_t address, size_t size) {
    DEBUG_PRINT("Writing %zu bytes to %#zx\n", size, address);
    memcpy(&cpu->memory[address], data, size);
}

void cpu_read(CPU* cpu, void* dest, size_t address, size_t size) {
    DEBUG_PRINT("Reading %zu bytes at %#zx\n", size, address);
    memcpy(dest, &cpu->memory[address], size);
}

void print_registers(CPU* cpu) {
   printf("CPU State:\n"
                "\t\tx0: %#lx(%lu)\n"
                "\t\tx1: %#lx(%lu)\n"
                "\t\tx2: %#lx(%lu)\n"
                "\t\tx3: %#lx(%lu)\n"
                "\t\tx4: %#lx(%lu)\n"
                "\t\tip: %#lx(%lu)\n"
                "\t\tsp: %#lx(%lu)\n"

              "\n\t\tnegative: %d\n"
                "\t\toverflow: %d\n"
                "\t\tzero: %d\n"
                "\t\tcarry: %d\n", cpu->x0, cpu->x0, cpu->x1, cpu->x1, cpu->x2, cpu->x2, cpu->x3, cpu->x3, cpu->x4, cpu->x4, cpu->ip, cpu->ip, cpu->sp, cpu->sp, cpu->flag_negative, cpu->flag_overflow, cpu->flag_zero, cpu->flag_carry);

}

// Get a pointer to the register ID specified by reg_index. If reg_index is an invalid register ID this functions returns NULL.
// Note that a reg_index of 0 is valid depending on certain instructions and can simply mean no register is supplied.
uint64_t* get_reg_ptr(CPU* cpu, uint8_t reg_id) {
    DEBUG_PRINT("Reg ID of %d\n", reg_id);

    // Registers are encoded in 3 bits. This allows for 8 registers, but only 7 can be used because all bits set to 0 means no register is supplied which can be a valid register
    // This code should not be executable under proper circumstances but its worth to check.
    if(reg_id > 0b111) {
        DEBUG_PRINT("Register ID is too large\n");
        return NULL;
    }

    if(reg_id == 0) {
        DEBUG_PRINT("No register supplied\n");
        return NULL;
    }

    switch(reg_id) {
        case 1: return &cpu->x0;
        case 2: return &cpu->x1;
        case 3: return &cpu->x2;
        case 4: return &cpu->x3;
        case 5: return &cpu->x4;
        case 6: return &cpu->sp;
        case 7: return &cpu->ip;
    }

    DEBUG_PRINT("Unknown error when getting register pointer\n");
    return NULL;
}

const char* get_reg_name_from_ptr(CPU* cpu, uint64_t* reg_ptr) {
    if(reg_ptr == &cpu->x0) {
        return "x0";
    }
    else if(reg_ptr == &cpu->x1) {
        return "x1";
    }
    else if(reg_ptr == &cpu->x2) {
        return "x2";
    }
    else if(reg_ptr == &cpu->x3) {
        return "x3";
    }
    else if(reg_ptr == &cpu->x4) {
        return "x4";
    }
    else if(reg_ptr == &cpu->ip) {
        return "ip";
    }
    else if(reg_ptr == &cpu->sp) {
        return "sp";
    }
    else {
        return "None";
    }
}

const char* get_reg_name(uint8_t reg_id) {
    switch(reg_id) {
        case 1: return "x0";
        case 2: return "x1";
        case 3: return "x2";
        case 4: return "x3";
        case 5: return "x4";
        case 6: return "sp";
        case 7: return "ip";
        default: return "None";
    }
}

// Write a value with the given size into the pointer given. Only the last 2 bits of 'size' are checked.
// reg_ptr must be a valid pointer.
void write_value_to_register(CPU* cpu, uint64_t* reg_ptr, uint64_t value, uint8_t size) {
    uint8_t size_lower_bits = size & 0b11;

    switch(size_lower_bits) {
        case 0:
            DEBUG_PRINT("Writing byte %lu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr &= 0xffffffffffffff00;
            *reg_ptr |= (value & 0x00000000000000ff);
            break;
        case 1:
            DEBUG_PRINT("Writing word %lu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr &= 0xffffffffffff0000;
            *reg_ptr |= (value & 0x000000000000ffff);
            break;
        case 2:
            DEBUG_PRINT("Writing dword %lu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr &= 0xffffffff00000000;
            *reg_ptr |= (value & 0x00000000ffffffff);
            break;
        case 3:
            DEBUG_PRINT("Writing qword %lu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr = value;
            break;
    }
}

// CPU instructions

void XXX(CPU* cpu) {
    DEBUG_PRINT("Invalid instruction with opcode %#lx\n", cpu->fetched);
}


void HLT(CPU* cpu) {
    DEBUG_EXECUTE(print_registers(cpu));

    cpu->halted = true;
}

void MOV(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint8_t dst_id = (cpu->fetched >> 3) & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(src_ptr == NULL || dst_ptr == NULL) {
        DEBUG_PRINT("Invalid register ID for MOV %d and/or %d\n", src_id, dst_id);
        // TODO: Transfer execution to error handling procedure
        return;
    }

    write_value_to_register(cpu, dst_ptr, *src_ptr, size);
}

void PLC(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t dst_id = cpu->fetched & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid register ID for PLC %d\n", dst_id);
        return;
    }

    switch(size) {
        case 0:
            fetch_byte(cpu);
            break;
        case 1:
            fetch_word(cpu);
            break;
        case 2:
            fetch_dword(cpu);
            break;
        case 3:
            fetch_qword(cpu);
            break;
    }

    write_value_to_register(cpu, dst_ptr, cpu->fetched, size);
}

// Adds src and dst reg and then puts the result in the dst register
void ADD(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint8_t dst_id = (cpu->fetched >> 3) & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer in ADD instruction\n");
        // TODO: Implement a callback procedure the programmer can define to start executing upon any error
        return;
    }

    uint64_t right_operator;
    if(src_ptr == NULL) {
        switch(size) {
            case 0:
                fetch_byte(cpu);
                break;
            case 1:
                fetch_word(cpu);
                break;
            case 2:
                fetch_dword(cpu);
                break;
            case 3:
                fetch_qword(cpu);
                break;
        }
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    uint64_t result = *dst_ptr + right_operator;

    uint64_t size_bytes = 1 << size;

    // We need to do all this bitshift and bit operations because the size of the operation can change and the alternative is writing 4 different branches for each size
    uint8_t dst_sign_bit = (*dst_ptr >> (size_bytes * 8 - 1)) & 1;
    uint8_t src_sign_bit = (right_operator >> (size_bytes * 8 - 1)) & 1;
    uint8_t result_sign_bit = (result >> (size_bytes * 8 - 1)) & 1;

    cpu->flag_overflow = dst_sign_bit == src_sign_bit && dst_sign_bit != result_sign_bit;
    cpu->flag_carry = (*dst_ptr > result);
    cpu->flag_negative = result_sign_bit;
    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void SUB(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint8_t dst_id = (cpu->fetched >> 3) & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer in SUB instruction\n");
        // TODO: Implement a callback procedure the programmer can define to start executing upon any error
        return;
    }

    uint64_t right_operator;
    if(src_ptr == NULL) {
        switch(size) {
            case 0:
                fetch_byte(cpu);
                break;
            case 1:
                fetch_word(cpu);
                break;
            case 2:
                fetch_dword(cpu);
                break;
            case 3:
                fetch_qword(cpu);
                break;
        }

        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    uint64_t result = *dst_ptr - right_operator;

    uint64_t size_bytes = 1 << size;
    uint8_t dst_sign_bit = (*dst_ptr >> (size_bytes * 8 - 1)) & 1;
    uint8_t src_sign_bit = (right_operator >> (size_bytes * 8 - 1)) & 1;
    uint8_t result_sign_bit = (result >> (size_bytes * 8 - 1)) & 1;

    cpu->flag_overflow = dst_sign_bit != src_sign_bit && dst_sign_bit != result_sign_bit;
    cpu->flag_carry = (*dst_ptr < result);
    cpu->flag_negative = result_sign_bit;
    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void MUL(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint8_t dst_id = (cpu->fetched >> 3) & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer in MUL instruction\n");
        // TODO: Implement a callback procedure the programmer can define to start executing upon any error
        return;
    }

    uint64_t right_operator;
    if(src_ptr == NULL) {
        switch(size) {
            case 0:
                fetch_byte(cpu);
                break;
            case 1:
                fetch_word(cpu);
                break;
            case 2:
                fetch_dword(cpu);
                break;
            case 3:
                fetch_qword(cpu);
                break;
        }
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    uint64_t result = *dst_ptr * right_operator;

    cpu->flag_zero = result == 0;
    cpu->flag_negative = (int64_t)result < 0;
    cpu->flag_carry = *dst_ptr > result;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void DIV(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint8_t dst_id = (cpu->fetched >> 3) & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer in DIV instruction\n");
        // TODO: Implement a callback procedure the programmer can define to start executing upon any error
        return;
    }

    uint64_t right_operator;
    if(src_ptr == NULL) {
        switch(size) {
            case 0:
                fetch_byte(cpu);
                break;
            case 1:
                fetch_word(cpu);
                break;
            case 2:
                fetch_dword(cpu);
                break;
            case 3:
                fetch_qword(cpu);
                break;
        }
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    if(right_operator == 0) {
        DEBUG_PRINT("Divide by ZERO error!\n");
        return;
    }

    uint64_t result = *dst_ptr / right_operator;

    cpu->flag_zero = result == 0;
    cpu->flag_negative = (int64_t)result < 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void OR(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint8_t dst_id = (cpu->fetched >> 3) & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        return;
    }

    uint64_t right_operand;

    if(src_ptr == NULL) {
        switch(size) {
            case 0:
                fetch_byte(cpu);
                break;
            case 1:
                fetch_word(cpu);
                break;
            case 2:
                fetch_dword(cpu);
                break;
            case 3:
                fetch_qword(cpu);
                break;
        }
        right_operand = cpu->fetched;
    }
    else {
        right_operand = *src_ptr;
    }

    uint64_t result = *dst_ptr | right_operand;

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void XOR(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint8_t dst_id = (cpu->fetched >> 3) & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        return;
    }

    uint64_t right_operand;

    if(src_ptr == NULL) {
        switch(size) {
            case 0:
                fetch_byte(cpu);
                break;
            case 1:
                fetch_word(cpu);
                break;
            case 2:
                fetch_dword(cpu);
                break;
            case 3:
                fetch_qword(cpu);
                break;
        }
        right_operand = cpu->fetched;
    }
    else {
        right_operand = *src_ptr;
    }

    uint64_t result = *dst_ptr ^ right_operand;

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void AND(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint8_t dst_id = (cpu->fetched >> 3) & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        return;
    }

    uint64_t right_operand;

    if(src_ptr == NULL) {
        switch(size) {
            case 0:
                fetch_byte(cpu);
                break;
            case 1:
                fetch_word(cpu);
                break;
            case 2:
                fetch_dword(cpu);
                break;
            case 3:
                fetch_qword(cpu);
                break;
        }
        right_operand = cpu->fetched;
    }
    else {
        right_operand = *src_ptr;
    }

    uint64_t result = *dst_ptr & right_operand;

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void NOT(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t dst_id = cpu->fetched & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        return;
    }

    uint64_t result = ~(*dst_ptr);

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void NEG(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t dst_id = cpu->fetched & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        return;
    }

    uint64_t result = -(*dst_ptr);

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void CMP(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint8_t dst_id = (cpu->fetched >> 3) & 0b111;
    uint8_t size = (cpu->fetched >> 6) & 0b11;

    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        return;
    }

    uint64_t right_operator;
    if(src_ptr == NULL) {
        switch(size) {
            case 0:
                fetch_byte(cpu);
                break;
            case 1:
                fetch_word(cpu);
                break;
            case 2:
                fetch_dword(cpu);
                break;
            case 3:
                fetch_qword(cpu);
                break;
        }

        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }
    DEBUG_PRINT("Comparing %lu with %lu\n", *dst_ptr, right_operator);

    uint64_t result = *dst_ptr - right_operator;

    uint8_t dst_sign_bit = *dst_ptr >> ((1 << size) * 8 - 1) & 1;
    uint8_t src_sign_bit = right_operator >> ((1 << size) * 8 - 1) & 1;
    uint8_t result_sign_bit = result >> ((1 << size) * 8 - 1) & 1;

    cpu->flag_zero = result == 0;
    cpu->flag_carry = *dst_ptr < result;
    cpu->flag_negative = result_sign_bit & 1;
    cpu->flag_overflow = dst_sign_bit != src_sign_bit && dst_sign_bit != result_sign_bit;
}

void PUSH(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t src_id = cpu->fetched & 0b111;
    uint64_t* src_ptr = get_reg_ptr(cpu, src_id);

    if(src_ptr == NULL) {
        DEBUG_PRINT("Invalid SRC register\n");
        return;
    }

    cpu->sp -= 8;
    cpu_write(cpu, src_ptr, cpu->sp, 8);
    DEBUG_PRINT("Wrote %lu to address %lu from register %s\n", *src_ptr, cpu->sp, get_reg_name(src_id));
}

void POP(CPU* cpu) {
    fetch_byte(cpu);

    uint8_t dst_id = cpu->fetched & 0b111;
    uint64_t* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register!\n");
        return;
    }

    cpu_read(cpu, dst_ptr, cpu->sp, 8);
    cpu->sp += 8;
    DEBUG_PRINT("Read %lu into register %s from address %lu\n", *dst_ptr, get_reg_name(dst_id), cpu->sp - 8);
}

void NOP(CPU* cpu) {
    // Do nothing
}
