#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "types.h"
#include "cpu.h"
#include "debug.h"

// Functions declarations
u8 fetch_byte(CPU* cpu);
u16 fetch_word(CPU* cpu);
u32 fetch_dword(CPU* cpu);
u64 fetch_qword(CPU* cpu);

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

void STR(CPU* cpu);
void LDR(CPU* cpu);
void LEA(CPU* cpu);

void JMP(CPU* cpu);
void JZ(CPU* cpu);
void JNZ(CPU* cpu);
void JO(CPU* cpu);
void JNO(CPU* cpu);
void JS(CPU* cpu);
void JNS(CPU* cpu);
void JC(CPU* cpu);
void JNC(CPU* cpu);
void JBE(CPU* cpu);
void JA(CPU* cpu);
void JL(CPU* cpu);
void JGE(CPU* cpu);
void JLE(CPU* cpu);
void JG(CPU* cpu);

void NOP(CPU* cpu);

typedef struct {
    const char* mnemonic;
    void(*operation)(CPU* cpu);
} instruction;

static instruction instruction_lookup[256] = {
   //           0             1             2             3             4             5             6             7             8             9             A             B             C             D             E             F
   /* 0 */ {"HLT", HLT}, {"MOV", MOV}, {"PLC", PLC}, {"ADD", ADD},  {"OR", OR},  {"JMP", JMP}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 1 */ {"XXX", XXX}, {"CMP", CMP}, {"XXX", XXX}, {"SUB", SUB}, {"XOR", XOR}, {"JZ", JZ}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 2 */ {"XXX", XXX}, {"PUSH", PUSH}, {"XXX", XXX}, {"MUL", MUL}, {"AND", AND}, {"JNZ", JNZ}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 3 */ {"XXX", XXX}, {"POP", POP}, {"XXX", XXX}, {"DIV", DIV}, {"NOT", NOT}, {"JO", JO}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 4 */ {"XXX", XXX}, {"STR", STR}, {"XXX", XXX}, {"XXX", XXX}, {"NEG", NEG}, {"JNO", JNO}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 5 */ {"XXX", XXX}, {"LDR", LDR}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JS", JS}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 6 */ {"XXX", XXX}, {"LEA", LEA}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JNS", JNS}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 7 */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JC", JC}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 8 */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JNC", JNC}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 9 */ {"NOP", NOP}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JBE", JBE}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* A */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JA", JA}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* B */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JL", JL}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* C */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JGE", JGE}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* D */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JLE", JLE}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* E */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JG", JG}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* F */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}
};


typedef struct impl_cpu {
    size_t memory_size; /* Size of memory in bytes */
    u8* memory;

    u64 fetched; /* Last fetched value */
    bool halted;

    u64 x0, x1, x2, x3, x4; /* General purpose registers. Can be used however the programmer wants */
    u64 sp; /* Stack pointer */
    u64 ip; /* Instruction pointer */

    /* CPU flags */
    u8 flag_negative;
    u8 flag_overflow;
    u8 flag_zero;
    u8 flag_carry;
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

    u8 opcode = fetch_byte(cpu);

    DEBUG_PRINT("Executing instruction %s opcode %#x\n", instruction_lookup[opcode].mnemonic, opcode);

    instruction_lookup[opcode].operation(cpu);
}

u8* cpu_raw_memory(CPU* cpu) {
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
u8 fetch_byte(CPU* cpu) {
    u8 byte;
    cpu_read(cpu, &byte, cpu->ip, 1);
    cpu->fetched = byte;
    cpu->ip++;
    return byte;
}

// Fetch a word (2 bytes) at the address of the instruction pointer
u16 fetch_word(CPU* cpu) {
    u16 word;
    cpu_read(cpu, &word, cpu->ip, 2);
    cpu->fetched = word;
    cpu->ip += 2;
    return word;
}

// Fetch a dword (4 bytes) at the address of the instruction pointer
u32 fetch_dword(CPU* cpu) {
    u32 dword;
    cpu_read(cpu, &dword, cpu->ip, 4);
    cpu->fetched = dword;
    cpu->ip += 4;
    return dword;
}

// Fetch a qword (8 bytes) at the address of the instruction pointer
u64 fetch_qword(CPU* cpu) {
    u64 qword;
    cpu_read(cpu, &qword, cpu->ip, 8);
    cpu->fetched = qword;
    cpu->ip += 8;
    return qword;
}

u64 fetch_sized(CPU* cpu, u64 size) {
    // There is some sort of internal error is size is not equal to 1, 2, 4, or 8
    assert(size == 1 || size == 2 || size == 4 || size == 8);

    u64 ret = 0;

    cpu_read(cpu, &ret, cpu->ip, size);
    cpu->fetched = ret;

    cpu->ip += size;
    return ret;
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

void get_cpu_state(CPU* cpu, cpu_state* state) {
    state->memory_size = cpu->memory_size;
    state->memory = cpu->memory;

    cpu->halted = cpu->halted;

    state->x0 = cpu->x0;
    state->x1 = cpu->x1;
    state->x2 = cpu->x2;
    state->x3 = cpu->x3;
    state->x4 = cpu->x4;

    state->ip = cpu->ip;
    state->sp = cpu->sp;

    state->flag_negative = cpu->flag_negative;
    state->flag_carry = cpu->flag_carry;
    state->flag_zero = cpu->flag_zero;
    state->flag_overflow = cpu->flag_overflow;
}

// Get a pointer to the register ID specified by reg_index. If reg_index is an invalid register ID this functions returns NULL.
// Note that a reg_index of 0 is valid depending on certain instructions and can simply mean no register is supplied.
u64* get_reg_ptr(CPU* cpu, u8 reg_id) {
    DEBUG_PRINT("Reg ID of %d\n", reg_id);

    if(reg_id > 0b111) {
        // This code should not be executed. If it is then there in an internal error.
        printf("Internal runtime error. reg_id is larger than 7, but this should never be the case. Exiting\n");
        exit(1);
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

    DEBUG_PRINT("Unknown error when getting register pointer. Exiting\n");
    exit(1);
}

const char* get_reg_name_from_ptr(CPU* cpu, u64* reg_ptr) {
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

const char* get_reg_name(u8 reg_id) {
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

// Parses bytes following the instruction pointer and calculates an effective address from them.
// All instructions that support addressing (I.E Not Push, or Pop) use the same encoding format.
size_t get_effective_address(CPU* cpu, u8* size) {
    DEBUG_PRINT("Parsing memory address\n");

    fetch_byte(cpu);

    u8 base_id = cpu->fetched & 0b111;
    u8 index_id = (cpu->fetched >> 3) & 0b111;
    // The multiplier is either 1, 2, 4 or 8.
    u8 multiplier = 1 << ((cpu->fetched >> 6) & 0b11);

    // Base and index are added together in the final memory address calculation
    u64 base_value, index_value;

    u64* base_ptr = get_reg_ptr(cpu, base_id);
    u64* index_ptr = get_reg_ptr(cpu, index_id);

    // These need to be here in case one one of the offset registers is the instruction pointer
    u64 fetched = fetch_byte(cpu);
    u64 const_offset = fetch_qword(cpu);

    if(base_ptr != NULL) {
        base_value = *base_ptr;
    }
    else {
        base_value = 0;
    }

    if(index_ptr != NULL) {
        index_value = *index_ptr;
    }
    else {
        index_value = 0;
    }


    *size = 1 << ((fetched >> 6) & 0b11);

    size_t address = (base_value + index_value + const_offset) * multiplier;
    DEBUG_PRINT("Parsed address %#zx (%zu)\n", address, address);
    return address;
}

// Write a value with the given size into the pointer given.
// reg_ptr must be a valid pointer.
// If an instruction has the option to write to an 1, 2, 4, or 8 byte variant of a register, this function must be used, unless the instruction intenionally needs to write to it in a custom way
void write_value_to_register(CPU* cpu, u64* reg_ptr, u64 value, u8 size) {
    assert(size == 1 || size == 2 || size == 4 || size == 8);

    switch(size) {
        case 1:
            DEBUG_PRINT("Writing byte %lu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr &= 0xffffffffffffff00;
            *reg_ptr |= (value & 0x00000000000000ff);
            break;
        case 2:
            DEBUG_PRINT("Writing word %lu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr &= 0xffffffffffff0000;
            *reg_ptr |= (value & 0x000000000000ffff);
            break;
        case 4:
            DEBUG_PRINT("Writing dword %lu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr &= 0xffffffff00000000;
            *reg_ptr |= (value & 0x00000000ffffffff);
            break;
        case 8:
            DEBUG_PRINT("Writing qword %lu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr = value;
            break;
    }
}

// Exceptions don't just mean something went wrong. It could be a syscall, or some kind of other interupt
// The programmer can define their own exception call table. If an exception that does not have a defined procedure is called then the CPU is reset.
void trigger_exception(CPU* cpu) {
    DEBUG_PRINT("Exceptions not implemented yet\n");
    exit(1);
}

// CPU instructions

void XXX(CPU* cpu) {
    DEBUG_PRINT("Invalid instruction with opcode %#lx\n", cpu->fetched);
    //trigger_exception(cpu);
}


void HLT(CPU* cpu) {
    DEBUG_EXECUTE(print_registers(cpu));

    cpu->halted = true;
}

void MOV(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        trigger_exception(cpu);
    }

    u64 move_value;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        move_value = cpu->fetched;
    }
    else {
        move_value = *src_ptr;
    }

    write_value_to_register(cpu, dst_ptr, move_value, size);
}

void PLC(CPU* cpu) {
    fetch_byte(cpu);

    u8 dst_id = cpu->fetched & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid register ID for PLC %d\n", dst_id);
        trigger_exception(cpu);
    }

    fetch_sized(cpu, size);

    write_value_to_register(cpu, dst_ptr, cpu->fetched, size);
}

// Adds src and dst reg and then puts the result in the dst register
void ADD(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer in ADD instruction\n");
        trigger_exception(cpu);
    }

    u64 right_operator;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    u64 result = *dst_ptr + right_operator;

    // We need to do all this bitshift and bit operations because the size of the operation can change and the alternative is writing 4 different branches for each size
    u8 dst_sign_bit = (*dst_ptr >> (size * 8 - 1)) & 1;
    u8 src_sign_bit = (right_operator >> (size * 8 - 1)) & 1;
    u8 result_sign_bit = (result >> (size * 8 - 1)) & 1;

    cpu->flag_overflow = dst_sign_bit == src_sign_bit && dst_sign_bit != result_sign_bit;
    cpu->flag_carry = (*dst_ptr > result);
    cpu->flag_negative = result_sign_bit;
    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void SUB(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer in SUB instruction\n");
        trigger_exception(cpu);
    }

    u64 right_operator;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    u64 result = *dst_ptr - right_operator;

    u8 dst_sign_bit = (*dst_ptr >> (size * 8 - 1)) & 1;
    u8 src_sign_bit = (right_operator >> (size * 8 - 1)) & 1;
    u8 result_sign_bit = (result >> (size * 8 - 1)) & 1;

    cpu->flag_overflow = dst_sign_bit != src_sign_bit && dst_sign_bit != result_sign_bit;
    cpu->flag_carry = (*dst_ptr < result);
    cpu->flag_negative = result_sign_bit;
    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void MUL(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer in MUL instruction\n");
        trigger_exception(cpu);
    }

    u64 right_operator;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    u64 result = *dst_ptr * right_operator;

    cpu->flag_zero = result == 0;
    cpu->flag_negative = (int64_t)result < 0;
    cpu->flag_carry = *dst_ptr > result;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void DIV(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer in DIV instruction\n");
        trigger_exception(cpu);
    }

    u64 right_operator;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    if(right_operator == 0) {
        DEBUG_PRINT("Divide by ZERO error!\n");
        trigger_exception(cpu);
    }

    u64 result = *dst_ptr / right_operator;

    cpu->flag_zero = result == 0;
    cpu->flag_negative = (int64_t)result < 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void OR(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        trigger_exception(cpu);
    }

    u64 right_operand;

    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operand = cpu->fetched;
    }
    else {
        right_operand = *src_ptr;
    }

    u64 result = *dst_ptr | right_operand;

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void XOR(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        trigger_exception(cpu);
    }

    u64 right_operand;

    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operand = cpu->fetched;
    }
    else {
        right_operand = *src_ptr;
    }

    u64 result = *dst_ptr ^ right_operand;

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void AND(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        trigger_exception(cpu);
    }

    u64 right_operand;

    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operand = cpu->fetched;
    }
    else {
        right_operand = *src_ptr;
    }

    u64 result = *dst_ptr & right_operand;

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void NOT(CPU* cpu) {
    fetch_byte(cpu);

    u8 dst_id = cpu->fetched & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        trigger_exception(cpu);
    }

    u64 result = ~(*dst_ptr);

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void NEG(CPU* cpu) {
    fetch_byte(cpu);

    u8 dst_id = cpu->fetched & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        trigger_exception(cpu);
    }

    u64 result = -(*dst_ptr);

    cpu->flag_zero = result == 0;

    write_value_to_register(cpu, dst_ptr, result, size);
}

void CMP(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        trigger_exception(cpu);
    }

    u64 right_operator;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }
    DEBUG_PRINT("Comparing %lu with %lu\n", *dst_ptr, right_operator);

    u64 result = *dst_ptr - right_operator;

    u8 dst_sign_bit = *dst_ptr >> (size * 8 - 1) & 1;
    u8 src_sign_bit = right_operator >> (size * 8 - 1) & 1;
    u8 result_sign_bit = result >> (size * 8 - 1) & 1;

    cpu->flag_zero = result == 0;
    cpu->flag_carry = *dst_ptr < result;
    cpu->flag_negative = result_sign_bit & 1;
    cpu->flag_overflow = dst_sign_bit != src_sign_bit && dst_sign_bit != result_sign_bit;
}

void PUSH(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u64* src_ptr = get_reg_ptr(cpu, src_id);

    if(src_ptr == NULL) {
        DEBUG_PRINT("Invalid SRC register\n");
        trigger_exception(cpu);
    }

    cpu->sp -= 8;
    cpu_write(cpu, src_ptr, cpu->sp, 8);
    DEBUG_PRINT("Wrote %lu to address %lu from register %s\n", *src_ptr, cpu->sp, get_reg_name(src_id));
}

void POP(CPU* cpu) {
    fetch_byte(cpu);

    u8 dst_id = cpu->fetched & 0b111;
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register!\n");
        trigger_exception(cpu);
    }

    cpu_read(cpu, dst_ptr, cpu->sp, 8);
    cpu->sp += 8;
    DEBUG_PRINT("Read %lu into register %s from address %lu\n", *dst_ptr, get_reg_name(dst_id), cpu->sp - 8);
}

void STR(CPU* cpu) {
    fetch_byte(cpu);

    u64* src_ptr = get_reg_ptr(cpu, cpu->fetched & 0b111);
    if(src_ptr == NULL) {
        DEBUG_PRINT("Invalid SRC register\n");
        trigger_exception(cpu);
    }

    u8 size;
    u64 effective_address = get_effective_address(cpu, &size);

    // TODO: Make this endian-independent
    cpu_write(cpu, src_ptr, effective_address, size);
}

void LDR(CPU* cpu) {
    fetch_byte(cpu);

    u64* dst_ptr = get_reg_ptr(cpu, cpu->fetched & 0b111);
    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        trigger_exception(cpu);
    }

    u8 size;
    u64 effective_address = get_effective_address(cpu, &size);

    u64 derefrenced = 0;
    cpu_read(cpu, &derefrenced, effective_address, size);

    write_value_to_register(cpu, dst_ptr, derefrenced, size);
}

void LEA(CPU* cpu) {
    fetch_byte(cpu);

    u64* dst_ptr = get_reg_ptr(cpu, cpu->fetched & 0b111);
    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        trigger_exception(cpu);
    }

    u8 size; /* Not used */
    u64 effective_address = get_effective_address(cpu, &size);

    *dst_ptr = effective_address;
}

void JMP(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    cpu->ip = address;
}

void JZ(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_zero == 1) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JNZ(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_zero == 0) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JO(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_overflow == 1) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JNO(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_overflow == 0) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JS(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_negative == 1) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JNS(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_negative == 0) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JC(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_carry == 1) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JNC(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_carry == 0) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JBE(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_carry == 1 || cpu->flag_zero == 1) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JA(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_carry == 0 && cpu->flag_zero == 0) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JL(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_negative != cpu->flag_overflow) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JGE(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_negative == cpu->flag_overflow) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JLE(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_zero == 1 || cpu->flag_negative != cpu->flag_overflow) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JG(CPU* cpu) {
    u8 size;
    size_t address = get_effective_address(cpu, &size);

    if(cpu->flag_zero == 0 && cpu->flag_negative == cpu->flag_overflow) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void NOP(CPU* cpu) {
    // Do nothing
}
