#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <address_bus.h>
#include "types.h"
#include "cpu/cpu.h"
#include "debug.h"
#include "cpu/reserved_idt_entries.h"

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
void PUSHF(CPU* cpu);
void POPF(CPU* cpu);

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
void CALL(CPU* cpu);
void RET(CPU* cpu);

void LIDT(CPU* cpu);
void RETI(CPU* cpu);
void INT(CPU* cpu);
void CLI(CPU* cpu);
void STI(CPU* cpu);

void IN(CPU* cpu);
void OUT(CPU* cpu);

void NOP(CPU* cpu);

typedef struct {
    const char* mnemonic;
    void(*operation)(CPU* cpu);
} instruction;

static instruction instruction_lookup[256] = {
   //           0             1             2             3             4             5             6             7             8             9             A             B             C             D             E             F
   /* 0 */ {"HLT", HLT}, {"MOV", MOV}, {"XXX", XXX}, {"ADD", ADD},  {"OR", OR},  {"JMP", JMP}, {"CALL", CALL}, {"XXX", XXX}, {"LIDT", LIDT}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 1 */ {"IN", IN}, {"CMP", CMP}, {"XXX", XXX}, {"SUB", SUB}, {"XOR", XOR}, {"JZ", JZ}, {"RET", RET}, {"XXX", XXX}, {"INT", INT}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 2 */ {"OUT", OUT}, {"PUSH", PUSH}, {"XXX", XXX}, {"MUL", MUL}, {"AND", AND}, {"JNZ", JNZ}, {"XXX", XXX}, {"XXX", XXX}, {"RETI", RETI}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 3 */ {"XXX", XXX}, {"POP", POP}, {"XXX", XXX}, {"DIV", DIV}, {"NOT", NOT}, {"JO", JO}, {"XXX", XXX}, {"XXX", XXX}, {"CLI", CLI}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 4 */ {"XXX", XXX}, {"STR", STR}, {"XXX", XXX}, {"XXX", XXX}, {"NEG", NEG}, {"JNO", JNO}, {"XXX", XXX}, {"XXX", XXX}, {"STI", STI}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 5 */ {"XXX", XXX}, {"LDR", LDR}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JS", JS}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 6 */ {"XXX", XXX}, {"LEA", LEA}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JNS", JNS}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 7 */ {"XXX", XXX}, {"PUSHF", PUSHF}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JC", JC}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 8 */ {"XXX", XXX}, {"POPF", POPF}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JNC", JNC}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* 9 */ {"NOP", NOP}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JBE", JBE}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* A */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JA", JA}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* B */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JL", JL}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* C */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JGE", JGE}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* D */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JLE", JLE}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* E */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"JG", JG}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX},
   /* F */ {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}, {"XXX", XXX}
};

#define FLAG_NEGATIVE 0
#define FLAG_OVERFLOW 1
#define FLAG_ZERO 2
#define FLAG_CARRY 3
#define FLAG_INTERRUPT 4

typedef struct impl_cpu {
    address_bus* addr_bus;
    port_bus* port_bus;

    u64 fetched; /* Last fetched value */
    bool halted;

    u64 idt; /* Address of the interrupt descriptor table.
                The interrupt descriptor table is 256 entries long with each entry being 8 bytes.
                Each entry is an address to transfer execution control to when an interrupt occurs */

    u64 x0, x1, x2, x3, x4; /* General purpose registers. Can be used however the programmer wants */
    u64 sp; /* Stack pointer */
    u64 ip; /* Instruction pointer */

    u64 cpu_flags;
} CPU;

CPU* create_cpu(address_bus* addr_bus, port_bus* port_bus) {
    // By default everything in the CPU is zero initialized
    CPU* cpu = calloc(1, sizeof(CPU));

    cpu->addr_bus = addr_bus;
    cpu->port_bus = port_bus;

    DEBUG_PRINT("Created cpu(%p)\n", cpu);
    return cpu;
}

void destroy_cpu(CPU* cpu) {
    DEBUG_PRINT("Destroyed cpu(%p)\n", cpu);
    
    free(cpu);
}

// Reset the CPU to a known state. The CPU reads 8 bytes from address 0x0 and starts executing at that address, and reads 8 bytes from 0x8 and sets the stack pointer to that value
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
    cpu->idt = 0;

    cpu->cpu_flags = 0;

    // CPU reads 8 bytes from address 0x0 to find an address to jump to in order to start executing
    cpu_read(cpu, &cpu->ip, 0x0, 8);
    DEBUG_PRINT("Beginning execution at %#llx\n", cpu->ip);

    // CPU reads 8 bytes from address 0x8 to find the top of the stack
    cpu_read(cpu, &cpu->sp, 0x8, 8);
    DEBUG_PRINT("Top of stack %#llx\n", cpu->sp);
}

void cpu_clock(CPU* cpu) {
    if(cpu->halted == true) {
        return;
    }

    u8 opcode = fetch_byte(cpu);

    DEBUG_PRINT("Executing instruction %s opcode %#x\n", instruction_lookup[opcode].mnemonic, opcode);

    instruction_lookup[opcode].operation(cpu);
}

address_bus* cpu_get_address_bus(CPU* cpu) {
    return cpu->addr_bus;
}

port_bus* cpu_get_port_bus(CPU* cpu) {
    return cpu->port_bus;
}

bool is_halted(CPU* cpu) {
    return cpu->halted;
}

void set_flag(CPU* cpu, u8 bit, bool value) {
    cpu->cpu_flags &= ~(1U << bit);
    cpu->cpu_flags |= (u64)value << bit;
}

bool get_flag(CPU* cpu, u8 bit) {
    return cpu->cpu_flags >> bit & 0x01;
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

void fetch_n(CPU* cpu, void* dest, size_t size) {
    cpu_read(cpu, dest, cpu->ip, size);
    cpu->ip += size;
}

// Fetch a qword (8 bytes) at the address of the instruction pointer
u64 fetch_qword(CPU* cpu) {
    u64 qword;
    cpu_read(cpu, &qword, cpu->ip, 8);
    cpu->fetched = qword;
    cpu->ip += 8;
    return qword;
}

void push_byte(CPU* cpu, u8 value) {
    cpu->sp -= 1;
    cpu_write(cpu, &value, cpu->sp, 1);
}

void push_word(CPU* cpu, u16 value) {
    cpu->sp -= 2;
    cpu_write(cpu, &value, cpu->sp, 2);
}

void push_dword(CPU* cpu, u32 value) {
    cpu->sp -= 4;
    cpu_write(cpu, &value, cpu->sp, 4);
}

void push_qword(CPU* cpu, u64 value) {
    cpu->sp -= 8;
    cpu_write(cpu, &value, cpu->sp, 8);
}

u8 pop_byte(CPU* cpu) {
    u8 value;
    cpu_read(cpu, &value, cpu->sp, 1);
    cpu->sp += 1;
    return value;
}

u16 pop_word(CPU* cpu) {
    u16 value;
    cpu_read(cpu, &value, cpu->sp, 2);
    cpu->sp += 2;
    return value;
}

u32 pop_dword(CPU* cpu) {
    u32 value;
    cpu_read(cpu, &value, cpu->sp, 4);
    cpu->sp += 4;
    return value;
}

u64 pop_qword(CPU* cpu) {
    u64 value;
    cpu_read(cpu, &value, cpu->sp, 8);
    cpu->sp += 8;
    return value;
}

void push_cpu_flags(CPU* cpu) {
    push_qword(cpu, cpu->cpu_flags);
}

void pop_cpu_flags(CPU* cpu) {
    cpu->cpu_flags = pop_qword(cpu);
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

void cpu_write(CPU* cpu, void* data, u64 address, size_t size) {
    DEBUG_PRINT("Writing %zu bytes to %#llx\n", size, address);
    address_bus_write(cpu->addr_bus, data, address, size);
}

void cpu_read(CPU* cpu, void* dest, u64 address, size_t size) {
    DEBUG_PRINT("Reading %zu bytes at %#llx\n", size, address);
    address_bus_read(cpu->addr_bus, dest, address, size);
}

void print_registers(CPU* cpu) {
   printf("CPU State:\n"
                "\t\tx0: %#llx(%llu) : Signed: %lld\n"
                "\t\tx1: %#llx(%llu) : Signed: %lld\n"
                "\t\tx2: %#llx(%llu) : Signed: %lld\n"
                "\t\tx3: %#llx(%llu) : Signed: %lld\n"
                "\t\tx4: %#llx(%llu) : Signed: %lld\n"
                "\t\tip: %#llx(%llu)\n"
                "\t\tsp: %#llx(%llu)\n"

              "\n\t\tnegative: %d\n"
                "\t\toverflow: %d\n"
                "\t\tzero: %d\n"
                "\t\tcarry: %d\n", cpu->x0, cpu->x0, (i64)cpu->x0, cpu->x1, cpu->x1, (i64)cpu->x1, cpu->x2, cpu->x2, (i64)cpu->x2, cpu->x3, cpu->x3, (i64)cpu->x3, cpu->x4, cpu->x4, (i64)cpu->x4, cpu->ip, cpu->ip, cpu->sp, cpu->sp, (i32)get_flag(cpu, FLAG_NEGATIVE), (i32)get_flag(cpu, FLAG_OVERFLOW), (i32)get_flag(cpu, FLAG_ZERO), (i32)get_flag(cpu, FLAG_CARRY));

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
// All instructions that support addressing use the same encoding format.
size_t get_effective_address(CPU* cpu) {
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

    // We must fetch all values we need for the future here in case one of the index registers is the instruction pointer.
    // The reason for this is to always have the instruction pointer point to the first byte of the next instruction when we calculate the actual address instead of some abritary point in the middle of the instruction.
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
            DEBUG_PRINT("Writing byte %llu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr &= 0xffffffffffffff00;
            *reg_ptr |= (value & 0x00000000000000ff);
            break;
        case 2:
            DEBUG_PRINT("Writing word %llu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr &= 0xffffffffffff0000;
            *reg_ptr |= (value & 0x000000000000ffff);
            break;
        case 4:
            DEBUG_PRINT("Writing dword %llu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr &= 0xffffffff00000000;
            *reg_ptr |= (value & 0x00000000ffffffff);
            break;
        case 8:
            DEBUG_PRINT("Writing qword %llu to register %s\n", value, get_reg_name_from_ptr(cpu, reg_ptr));
            *reg_ptr = value;
            break;
    }
}
//
// The actual code to handle CPU interrupts
void interrupt_handler(CPU* cpu, u8 idt_entry) {
    const int sizeof_idt_entry = 8;

    /// 0 is an invalid location for the IDT. Any interrupt that occurs is irrecoverable so we must reset the cpu to a known state
    if(cpu->idt == 0) {
        cpu_reset(cpu);
        return;
    }

    u64 idt_entry_address = cpu->idt + (idt_entry * sizeof_idt_entry);

    u64 handler_address;
    cpu_read(cpu, &handler_address, idt_entry_address, sizeof(u64));

    // A handler address of zero implies this entry was unitialized
    if(handler_address == 0) {
        cpu_reset(cpu);
        return;
    }

    push_cpu_flags(cpu);
    push_qword(cpu, cpu->ip);

    cpu->ip = handler_address;

    // TODO Push CPU flags, and old instruction pointer to stack

}

// Triggers an interrupt that cannot be disabled
void non_maskable_interrupt(CPU* cpu, u8 idt_entry) {
    DEBUG_PRINT("Non maskable interrupt for entry %d\n", idt_entry);
    interrupt_handler(cpu, idt_entry);
}

// Triggers an interrupt request that can be disabled if the interrupt disable flag is set
void interrupt_request(CPU* cpu, u8 idt_entry) {
    DEBUG_PRINT("Interrupt request for entry %d\n", idt_entry);
    if(get_flag(cpu, FLAG_INTERRUPT) == false) {
        DEBUG_PRINT("Interrupts disabled\n");
        return;
    }

    interrupt_handler(cpu, idt_entry);
}

// CPU instructions

void XXX(CPU* cpu) {
    DEBUG_PRINT("Invalid instruction with opcode %#llx\n", cpu->fetched);
    non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
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

    u64 move_value;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        move_value = cpu->fetched;
    }
    else {
        move_value = *src_ptr;
    }

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    write_value_to_register(cpu, dst_ptr, move_value, size);
}

// Adds src and dst reg and then puts the result in the dst register
void ADD(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    u64 right_operator;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = *dst_ptr + right_operator;

    u8 dst_sign_bit = (*dst_ptr >> (size * 8 - 1)) & 1;
    u8 src_sign_bit = (right_operator >> (size * 8 - 1)) & 1;
    u8 result_sign_bit = (result >> (size * 8 - 1)) & 1;

    set_flag(cpu, FLAG_OVERFLOW, dst_sign_bit == src_sign_bit && dst_sign_bit != result_sign_bit);
    set_flag(cpu, FLAG_CARRY, *dst_ptr > result);
    set_flag(cpu, FLAG_NEGATIVE, result_sign_bit);
    set_flag(cpu, FLAG_ZERO, result == 0);

    write_value_to_register(cpu, dst_ptr, result, size);
}

void SUB(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    u64 right_operator;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = *dst_ptr - right_operator;

    u8 dst_sign_bit = (*dst_ptr >> (size * 8 - 1)) & 1;
    u8 src_sign_bit = (right_operator >> (size * 8 - 1)) & 1;
    u8 result_sign_bit = (result >> (size * 8 - 1)) & 1;

    set_flag(cpu, FLAG_OVERFLOW, dst_sign_bit != src_sign_bit && dst_sign_bit != result_sign_bit);
    set_flag(cpu, FLAG_CARRY, *dst_ptr < result);
    set_flag(cpu, FLAG_NEGATIVE, result_sign_bit);
    set_flag(cpu, FLAG_ZERO, result == 0);

    write_value_to_register(cpu, dst_ptr, result, size);
}

void MUL(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    u64 right_operator;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination pointer\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = *dst_ptr * right_operator;

    set_flag(cpu, FLAG_ZERO, result == 0);
    set_flag(cpu, FLAG_NEGATIVE, (i64)result < 0);
    set_flag(cpu, FLAG_CARRY, *dst_ptr > result);

    write_value_to_register(cpu, dst_ptr, result, size);
}

void DIV(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

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
        non_maskable_interrupt(cpu, DIVIDE_BY_ZERO);
        return;
    }

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = *dst_ptr / right_operator;

    set_flag(cpu, FLAG_ZERO, result == 0);
    set_flag(cpu, FLAG_NEGATIVE, (i64)result < 0);

    write_value_to_register(cpu, dst_ptr, result, size);
}

void OR(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    u64 right_operand;

    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operand = cpu->fetched;
    }
    else {
        right_operand = *src_ptr;
    }

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = *dst_ptr | right_operand;

    set_flag(cpu, FLAG_ZERO, result == 0);

    write_value_to_register(cpu, dst_ptr, result, size);
}

void XOR(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    u64 right_operand;

    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operand = cpu->fetched;
    }
    else {
        right_operand = *src_ptr;
    }

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = *dst_ptr ^ right_operand;

    set_flag(cpu, FLAG_ZERO, result == 0);

    write_value_to_register(cpu, dst_ptr, result, size);
}

void AND(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    u64 right_operand;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operand = cpu->fetched;
    }
    else {
        right_operand = *src_ptr;
    }

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = *dst_ptr & right_operand;

    set_flag(cpu, FLAG_ZERO, result == 0);

    write_value_to_register(cpu, dst_ptr, result, size);
}

void NOT(CPU* cpu) {
    fetch_byte(cpu);

    u8 dst_id = cpu->fetched & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = ~(*dst_ptr);

    set_flag(cpu, FLAG_ZERO, result == 0);

    write_value_to_register(cpu, dst_ptr, result, size);
}

void NEG(CPU* cpu) {
    fetch_byte(cpu);

    u8 dst_id = cpu->fetched & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid destination register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = -(*dst_ptr);

    set_flag(cpu, FLAG_ZERO, result == 0);

    write_value_to_register(cpu, dst_ptr, result, size);
}

void CMP(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u8 dst_id = (cpu->fetched >> 3) & 0b111;
    u8 size = 1 << ((cpu->fetched >> 6) & 0b11);

    u64* src_ptr = get_reg_ptr(cpu, src_id);
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    u64 right_operator;
    if(src_ptr == NULL) {
        fetch_sized(cpu, size);
        right_operator = cpu->fetched;
    }
    else {
        right_operator = *src_ptr;
    }
    DEBUG_PRINT("Comparing %llu with %llu\n", *dst_ptr, right_operator);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 result = *dst_ptr - right_operator;

    u8 dst_sign_bit = *dst_ptr >> (size * 8 - 1) & 1;
    u8 src_sign_bit = right_operator >> (size * 8 - 1) & 1;
    u8 result_sign_bit = result >> (size * 8 - 1) & 1;

    set_flag(cpu, FLAG_ZERO, result == 0);
    set_flag(cpu, FLAG_CARRY, *dst_ptr < result);
    set_flag(cpu, FLAG_NEGATIVE, result_sign_bit & 1);
    set_flag(cpu, FLAG_OVERFLOW, dst_sign_bit != src_sign_bit && dst_sign_bit != result_sign_bit);
}

void PUSH(CPU* cpu) {
    fetch_byte(cpu);

    u8 src_id = cpu->fetched & 0b111;
    u64* src_ptr = get_reg_ptr(cpu, src_id);

    if(src_ptr == NULL) {
        DEBUG_PRINT("Invalid SRC register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    push_qword(cpu, *src_ptr);

    DEBUG_PRINT("Wrote %llu to address %llu from register %s\n", *src_ptr, cpu->sp, get_reg_name(src_id));
}

void POP(CPU* cpu) {
    fetch_byte(cpu);

    u8 dst_id = cpu->fetched & 0b111;
    u64* dst_ptr = get_reg_ptr(cpu, dst_id);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register!\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u64 value = pop_qword(cpu);
    *dst_ptr = value;

    DEBUG_PRINT("Read %llu into register %s from address %llu\n", *dst_ptr, get_reg_name(dst_id), cpu->sp - 8);
}

void PUSHF(CPU* cpu) {
    push_cpu_flags(cpu);
}

void POPF(CPU* cpu) {
    pop_cpu_flags(cpu);
}

void STR(CPU* cpu) {
    fetch_byte(cpu);

    u64* src_ptr = get_reg_ptr(cpu, cpu->fetched & 0b111);
    u8 size = 1 << (cpu->fetched >> 6 & 0b11);

    u64 effective_address = get_effective_address(cpu);

    if(src_ptr == NULL) {
        DEBUG_PRINT("Invalid SRC register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    cpu_write(cpu, src_ptr, effective_address, size);
}

void LDR(CPU* cpu) {
    fetch_byte(cpu);

    u64* dst_ptr = get_reg_ptr(cpu, cpu->fetched & 0b111);
    u8 size = 1 << (cpu->fetched >> 6 & 0b11);

    u64 effective_address = get_effective_address(cpu);

    u64 derefrenced = 0;
    cpu_read(cpu, &derefrenced, effective_address, size);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    write_value_to_register(cpu, dst_ptr, derefrenced, size);
}

void LEA(CPU* cpu) {
    fetch_byte(cpu);

    u64* dst_ptr = get_reg_ptr(cpu, cpu->fetched & 0b111);
    u8 size = 1 << (cpu->fetched >> 6 & 0b11);

    u64 effective_address = get_effective_address(cpu);

    if(dst_ptr == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    write_value_to_register(cpu, dst_ptr, effective_address, size);
}

void JMP(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    cpu->ip = address;
}

void JZ(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_ZERO) == true) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JNZ(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_ZERO) == false) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JO(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_OVERFLOW) == true) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JNO(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_OVERFLOW) == false) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JS(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_NEGATIVE) == true) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JNS(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_NEGATIVE) == false) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JC(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_CARRY) == true) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JNC(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_CARRY) == false) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JBE(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_CARRY) == true || get_flag(cpu, FLAG_ZERO) == true) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JA(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_CARRY) == false && get_flag(cpu, FLAG_ZERO) == false) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JL(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_NEGATIVE) != get_flag(cpu, FLAG_OVERFLOW)) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JGE(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_NEGATIVE) == get_flag(cpu, FLAG_OVERFLOW)) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}
void JLE(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_ZERO) == true || get_flag(cpu, FLAG_NEGATIVE) != get_flag(cpu, FLAG_OVERFLOW)) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void JG(CPU* cpu) {
    size_t address = get_effective_address(cpu);

    if(get_flag(cpu, FLAG_ZERO) == false && get_flag(cpu, FLAG_NEGATIVE) == get_flag(cpu, FLAG_OVERFLOW)) {
        DEBUG_PRINT("Jumping to address %#zx\n", address);
        cpu->ip = address;
    }
}

void CALL(CPU* cpu) {
    size_t effective_address = get_effective_address(cpu);

    push_qword(cpu, cpu->ip);
    DEBUG_PRINT("Pushing current instruction pointer(%#llx) to address %#llx\n", cpu->ip, cpu->sp);

    cpu->ip = effective_address;
}

void RET(CPU* cpu) {
    DEBUG_PRINT("Reading from address %#llx into instruction pointer\n", cpu->sp);

    u64 return_address = pop_qword(cpu);
    cpu->ip = return_address;
}

void LIDT(CPU* cpu) {
    size_t effective_address = get_effective_address(cpu);

    DEBUG_PRINT("Setting IDT to address %p\n", (void*)effective_address);
    cpu->idt = effective_address;
}

void RETI(CPU* cpu) {
    u64 return_address = pop_qword(cpu);
    pop_cpu_flags(cpu);
    cpu->ip = return_address;
}

void INT(CPU* cpu) {
    fetch_byte(cpu);
    interrupt_request(cpu, (u8)cpu->fetched);
}

void CLI(CPU* cpu) {
    set_flag(cpu, FLAG_INTERRUPT, false);
}

void STI(CPU* cpu) {
    set_flag(cpu, FLAG_INTERRUPT, true);
}

void IN(CPU* cpu) {
    fetch_byte(cpu);

    u64* dst_reg = get_reg_ptr(cpu, cpu->fetched & 0xff);

    if(dst_reg == NULL) {
        DEBUG_PRINT("Invalid DST register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u16 port = fetch_word(cpu);

    port_bus_read(cpu->port_bus, port, dst_reg);
}

void OUT(CPU* cpu) {
    fetch_byte(cpu);

    u64* src_reg = get_reg_ptr(cpu, cpu->fetched & 0xff);

    if(src_reg == NULL) {
        DEBUG_PRINT("Invalid SRC register\n");
        non_maskable_interrupt(cpu, INVALID_INSTRUCTION);
        return;
    }

    u16 port = fetch_word(cpu);

    port_bus_write(cpu->port_bus, port, *src_reg);
}

void NOP(CPU* cpu) {
    // Do nothing
}
