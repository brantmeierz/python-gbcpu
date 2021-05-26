# Green GB palette
palette = [(0, 63, 0), (46, 115, 32), (140, 191, 10), (160, 207, 10)]

waiting_on_button = False
interrupts_enabled = False

def enable_interrupts():
    global interrupts_enabled
    interrupts_enabled = True

def disable_interrupts():
    global interrupts_enabled
    interrupts_enabled = False

def err_msg(msg, fatal=False):
    """
    Reports a formatted error message.
    """
    if fatal:
        print("[!] FATAL ERROR:")
        print(msg)
        print("[!][!][!]")
    else:
        print("[!] Error:")
        print(msg)

def byte_string(size):
    """
    Converts a quantity of bytes into a properly formatted string with the right unit.
    """
    if size < 1024:
        return str(size) + " B"
    elif size < 1024 * 1024:
        return str(size / 1024) + " KB"
    elif size < 1024 * 1024 * 1024:
        return str(size / 1024 / 1024) + " MB"
    else:
        return "TOO BIG BYTE STRING"

def signed_8bit(val): #???
    if val & 0b10000000:
        return val
    else:
        return val

#==========
#= MEMORY =
#==========
memory = [0] * 0xFFFF

def mem_init():
    """
    Initializes memory. DONT USE YET
    """
    for i in range(0, 0xFFFF):
        memory[i] = 0

def mem_set(add, val):
    """
    Sets memory at the specified address to the given value.
    
    Takes into account the memory constraints of
    the processor and throws appropriate errors.
    """
    
    # Address sanity checking
    if add < 0x0000:
        err_msg("Attempting to set memory at a negative address")
        return
    elif add > 0xFFFF:
        err_msg("Attempting to set memory above available range")
        return
    elif add >= 0xE000 and add <= 0xFDFF:
        err_msg("Writing to upper half of echo RAM, prohibited but proceeding")

    # Value sanity checking
    if val < 0x00:
        err_msg("Attempting to set memory to a negative value, proceeding with mask")

    if val > 0xFF:
        err_msg("Attempting to set value of greater than 1 byte, proceeding with mask")

    # Perform write operation
    memory[add] = val & 0xFF

    # Handle echo block
    #if add >= 0xC000 and add <= 0xDFFF:
    #    print(add - (0xE000 - 0xC000))
    #    memory[add + (0xE000 - 0xC000)] = val & 0xFF
    #elif add >= 0xE000 and add <= 0xFDFF:
    #    print(add - (0xE000 - 0xC000))
    #    memory[add - (0xE000 - 0xC000)] = val & 0xFF

def mem_read(add):
    """
    Returns the value found at the specified memory address.
    """
    #Address sanity checking
    if add < 0x0000:
        err_msg("Attempting to read memory at a negative address")
        return
    elif add > 0xFFFF:
        err_msg("Attempting to read memory above available range")
        return

SP = 0x0000
def SP_set(val):
    """
    Sets the stack pointer to a value.
    """
    SP = val & 0xFFFF

def push_stack(val):
    pass

def pop_stack(val):
    pass

#=============
#= CARTRIDGE =
#=============
cart_data = []
cart_name = ""
def load_cart(filename, verbose=False):
    """
    Loads the game cartridge with the specified filename into memory.
    """
    global cart_data
    global cart_name
    with open(filename, 'rb') as f:
        cart_data = f.read()
        for i in range(0x0134, 0x0141):
            if cart_data[i] != 0:
                print(str(i) + ": " + str(cart_data[i]) + " " + chr(cart_data[i]))
                cart_name = cart_name + chr(cart_data[i])
    
    if verbose:
        print("=================")
        print("= Load complete =")
        print("Name: " + cart_name)
        print("Size: " + byte_string(len(cart_data)))
        print("=================")

#=============
#= REGISTERS =
#=============
A = 0x00
F = 0x00
B = 0x00
C = 0x00
D = 0x00
E = 0x00
H = 0x00
L = 0x00

def A_set(val):
    """
    Sets the A register to the value provided, masked to a byte.
    """
    global A
    A = val & 0xFF

def F_set(val):
    """
    Sets the F register to the value provided, masked to a byte. (not often used)
    """
    global F
    F = val & 0xFF

def B_set(val):
    """
    Sets the B register to the value provided, masked to a byte.
    """
    global B
    B = val & 0xFF

def C_set(val):
    """
    Sets the C register to the value provided, masked to a byte.
    """
    global C
    C = val & 0xFF

def D_set(val):
    """
    Sets the D register to the value provided, masked to a byte.
    """
    global D
    D = val & 0xFF

def E_set(val):
    """
    Sets the E register to the value provided, masked to a byte.
    """
    global E
    E = val & 0xFF

def H_set(val):
    """
    Sets the H register to the value provided, masked to a byte.
    """
    global H
    H = val & 0xFF

def L_set(val):
    """
    Sets the L register to the value provided, masked to a byte.
    """
    global L
    L = val & 0xFF

def compose(byte_high, byte_low):
    """
    Combines a high and low byte (often two registers) to form a single 2-byte value.
    """
    return ((byte_high << 8) | byte_low) & 0xFFFF

def swap(byte):
    """
    Swaps the upper and lower nibbles of the byte.
    """
    return (((byte & 0xF) << 4) | ((byte & 0xF0) >> 4)) & 0xFF

def BC():
    """
    Returns the value of the BC register.
    """
    return compose(B, C)

def BC_set(val):
    global B
    global C
    B = (val & 0xFF00) >> 8
    C = val & 0x00FF

def DE():
    """
    Returns the value of the DE register.
    """
    return compose(D, E)

def DE_set(val):
    global D
    global E
    D = (val & 0xFF00) >> 8
    E = val & 0x00FF

def HL():
    """
    Returns the value of the HL register.
    """
    return compose(H, L)

def HL_set(val):
    global H
    global L
    H = (val & 0xFF00) >> 8
    L = val & 0x00FF

BIT_0 = 0b00000001
BIT_1 = 0b00000010
BIT_2 = 0b00000100
BIT_3 = 0b00001000
BIT_4 = 0b00010000
BIT_5 = 0b00100000
BIT_6 = 0b01000000
BIT_7 = 0b10000000
BIT_11 = 0b000010000000000
BIT_15 = 0b100000000000000

def RL(val):
    """
    Returns left-rotated value.
    """
    bit = nonzero_to_one(val & BIT_7)
    return ((val << 1) & 0xFF) | bit

def RR(val):
    """
    Returns right-rotated value.
    """
    bit = nonzero_to_one(val & BIT_0)
    return ((val >> 1) & 0xFF) | (bit << 7)

#===================
#= PROGRAM COUNTER =
#===================
PC = 0x0000

def PC_set(val):
    """
    Sets the program counter to the specified value.
    """
    global PC
    
    #Sanity checking
    if val < 0x0000:
        err_msg("Attempting to set program counter to a negative address")
        return
    elif val > 0xFFFF:
        err_msg("Attempting to set program counter above available range")

    PC = val & 0xFFFF

def PC1():
    """
    Increment program counter by 1.
    """
    PC_set(PC + 1)

def PC2():
    """
    Increment program counter by 2.
    """
    PC_set(PC + 2)

def PC3():
    """
    Increment program counter by 3.
    """
    PC_set(PC + 3)

def rst(n):
    """
    """
    global PC
    push_stack(PC)
    PC = n

def ret():
    """
    Ret instruction, returns to the address on the stack.
    """
    global PC
    byte1 = pop_stack()
    byte2 = pop_stack()
    PC = compose(byte1, byte2)


#=========
#= FLAGS =
#=========
F_Z = 0b10000000
F_N = 0b01000000
F_H = 0b00100000
F_C = 0b00010000

def set_flag(FLAG):
    """
    Sets the specified flag (F_Z, F_N, F_H, or F_C) of the flag register (F) to one.
    """
    global F
    F = F | FLAG

def set_flag_to(FLAG, val):
    if val == 0:
        unset_flag(FLAG)
    elif val == 1:
        set_flag(FLAG)
    else:
        err_msg("Unexpected flag value: " + str(val))

def unset_flag(FLAG):
    """
    Sets the specified flag (F_Z, F_N, F_H, or F_C) of the flag register (F) to zero.
    """
    global F
    F = F & ~FLAG

def zero_flag(val):
    """
    Sets or unsets the zero flag according to the input value provided.
    """
    if val == 0x00:
        set_flag(F_Z)
    else:
        unset_flag(F_Z)

def halfcarry_flag(val1, val2, subtract=False):
    """
    Sets or unsets the half carry flag according to the two input values provided (initial and result).
    """
    return

def halfcarry_flag_16(val1, val2, subtract=False):
    """
    16-bit variation of halfcarry_flag.
    """
    return

def carry_flag(val):
    """
    Sets or unsets the carry flag according to the result value provided.
    """
    if (val & 0x100) != 0x00:
        set_flag(F_C)
    else:
        unset_flag(F_C)

def carry_flag_16(val):
    """
    16-bit variation of carry_flag.
    """
    if (val & 0x10000) != 0x00:
        set_flag(F_C)
    else:
        unset_flag(F_C)

def flag_is_set(FLAG):
    """
    Checks if a given flag is set.
    """
    return ((F & FLAG) != 0x00)

def nonzero_to_one(val):
    if val == 0:
        return 0
    else:
        return 1

#=============
#= OPERATION =
#=============
def parse_opcode(opcode):
    
    #NOP [1 4] [- - - -]
    if opcode == 0x00:
        PC1()
        return

    #LD BC, d16 [3 12] [- - - -]
    elif opcode == 0x01:
        B_set(memory[PC + 1])
        C_set(memory[PC + 2])
        PC3()
        return
    
    #LD (BC), A [1 8] [- - - -]
    elif opcode == 0x02:
        mem_set(BC(), A)
        PC1()
        return

    #INC BC [1 8] [- - - -]
    elif opcode == 0x03:
        BC_set(BC() + 1)
        PC1()
        return

    #INC B [1 4] [Z 0 H -]
    elif opcode == 0x04:
        halfcarry_flag(B, B + 1)
        B_set(B + 1)
        zero_flag(B)
        unset_flag(F_N)
        PC1()
        return

    #DEC B [1 4] [Z 1 H -]
    elif opcode == 0x05:
        halfcarry_flag(B, B - 1, True)
        B_set(B - 1)
        unset_flag(F_N)
        zero_flag(B)
        PC1()
        return

    #LD B, d8 [2 8] [- - - -]
    elif opcode == 0x06:
        B_set(mem_read(PC + 1))
        PC2()
        return

    #RLCA [1 4] [Z 0 0 C]
    elif opcode == 0x07:
        left_bit = nonzero_to_one(A & BIT_7)
        set_flag_to(F_C, left_bit)
        A_set(RL(A))
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        return

    #LD (a16), SP [3 20] [- - - -]
    elif opcode == 0x08:
        add = compose(mem_read(PC + 1), mem_read(PC + 2))
        mem_set(add, SP & 0xFF)
        mem_set(add + 1, (SP & 0xFF00) >> 8)
        PC3()
        return

    #ADD HL, BC [1 8] [- 0 H C]
    elif opcode == 0x09:
        unset_flag(F_N)
        halfcarry_flag_16(HL(), HL() + BC())
        carry_flag_16(HL(), HL() + BC())
        HL_set(HL() + BC())
        PC1()
        return

    #LD A, (BC) [1 8] [- - - -]
    elif opcode == 0x0A:
        A_set(mem_read(BC()))
        PC1()
        return

    #DEC BC [1 8] [- - - -]
    elif opcode == 0x0B:
        BC_set(BC() - 1)
        PC1()
        return

    #INC C [1 4] [Z 0 H -]
    elif opcode == 0x0C:
        halfcarry_flag(C, C + 1)
        C_set(C + 1)
        unset_flag(F_N)
        zero_flag(C)
        PC1()
        return

    #LD C, d8 [2 8] [- - - -]
    elif opcode == 0x0E:
        C_set(mem_read(PC + 1))
        PC2()
        return

    #RRCA [1 4] [0 0 0 C] ???
    elif opcode == 0x0F:

        PC1()
        return

    #STOP (+NOP) [1 4] [- - - -] ???
    elif opcode == 0x10:
        global waiting_on_button
        waiting_on_button = True
        PC1()
        return

    #LD DE, d16 [3 12] [- - - -]
    elif opcode == 0x11:
        D_set(memory[PC + 1])
        E_set(memory[PC + 2])
        PC3()
        return

    #LD (DE), A [1 8] [- - - -]
    elif opcode == 0x12:
        mem_set(DE(), A)
        PC1()
        return

    #INC DE [1 8] [- - - -]
    elif opcode == 0x13:
        DE_set(DE() + 1)
        PC1()
        return

    #INC D [1 4] [Z 0 H -]
    elif opcode == 0x14:
        halfcarry_flag(D, D + 1)
        D_set(D + 1)
        zero_flag(D)
        unset_flag(F_N)
        PC1()
        return

    #LD D, d8 [2 8] [- - - -]
    elif opcode == 0x16:
        D_set(mem_read(PC + 1))
        PC2()
        return

    #RLA [1 4] [Z 0 0 C]
    elif opcode == 0x17:
        left_bit = nonzero_to_one(A & BIT_7)
        carry_bit = nonzero_to_one(A & F_C)
        A_set(((A << 1) & 0xFF) | carry_bit)
        set_flag_to(F_C, left_bit)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        PC1()
        return

    #LD A, (DE) [1 8] [- - - -]
    elif opcode == 0x1A:
        A_set(mem_read(DE()))
        PC1()
        return

    #DEC DE [1 8] [- - - -]
    elif opcode == 0x1B:
        DE_set(DE() - 1)
        PC1()
        return

    #INC E [1 4] [Z 0 H -]
    elif opcode == 0x1C:
        halfcarry_flag(E, E + 1)
        E_set(E + 1)
        unset_flag(F_N)
        zero_flag(E)
        PC1()
        return

    #LD E, d8 [2 8] [- - - -]
    elif opcode == 0x1E:
        E_set(mem_read(PC + 1))
        PC2()
        return

    #JR NZ, r8 [2 12/8] [- - - -]
    elif opcode == 0x20:
        if not flag_is_set(F_Z):
            PC_set(PC + signed_8bit(mem_read(PC + 1)))
            return
        else:
            PC2()
            return

    #LD HL, d16 [3 12] [- - - -]
    elif opcode == 0x21:
        H_set(memory[PC + 1])
        L_set(memory[PC + 2])
        PC3()
        return

    #LD (HL+), A [1 8] [- - - -]
    elif opcode == 0x22:
        mem_set(HL(), A)
        HL_set(HL() + 1)
        PC1()
        return

    #INC HL [1 8] [- - - -]
    elif opcode == 0x23:
        HL_set(HL() + 1)
        PC1()
        return

    #INC H [1 4] [Z 0 H -]
    elif opcode == 0x24:
        halfcarry_flag(H, H + 1)
        H_set(H + 1)
        zero_flag(H)
        unset_flag(F_N)
        PC1()
        return

    #DEC HL [1 8] [- - - -]
    elif opcode == 0x2B:
        HL_set(HL() - 1)
        PC1()
        return

    #INC L [1 4] [Z 0 H -]
    elif opcode == 0x2C:
        halfcarry_flag(L, L + 1)
        L_set(L + 1)
        unset_flag(F_N)
        zero_flag(L)
        PC1()
        return

    #LD L, d8 [2 8] [- - - -]
    elif opcode == 0x2E:
        L_set(mem_read(PC + 1))
        PC2()
        return

    #CPL [1 4] [- 1 1 -]
    elif opcode == 0x2F:
        A_set(~A & 0xFF)
        set_flag(F_N)
        set_flag(F_H)
        PC1()
        return

    #JR NC, r8 [2 12/8] [- - - -]
    elif opcode == 0x30:
        if not flag_is_set(F_N):
            PC_set(PC + signed_8bit(mem_read(PC + 1)))
            return
        else:
            PC2()
            return

    #LD DE, d16 [3 12] [- - - -]
    elif opcode == 0x31:
        DE_set(compose(mem_read(PC + 1), mem_read(PC + 2)))
        PC3()
        return

    #LD (HL-), A [1 8] [- - - -]
    elif opcode == 0x32:
        mem_set(HL(), A)
        HL_set(HL() - 1)
        PC1()
        return

    #INC SP [1 8] [- - - -]
    elif opcode == 0x33:
        SP_set(SP + 1)
        PC1()
        return

    #INC (HL) [1 12] [Z 0 H -]
    elif opcode == 0x34:
        halfcarry_flag(mem_read(HL()), mem_read(HL()) + 1)
        mem_set(HL(), mem_read(HL()) + 1)
        zero_flag(mem_read(HL()))
        unset_flag(F_N)
        PC1()
        return

    #SCF [1 4] [- 0 0 1]
    elif opcode == 0x37:
        unset_flag(F_N)
        unset_flag(F_H)
        set_flag(C)
        PC1()
        return

    #DEC SP [1 8] [- - - -]
    elif opcode == 0x3B:
        SP_set(SP - 1)
        PC1()
        return

    #INC A [1 4] [Z 0 H -]
    elif opcode == 0x3C:
        halfcarry_flag(A, A + 1)
        A_set(A + 1)
        unset_flag(F_N)
        zero_flag(A)
        PC1()
        return

    #LD A, d8 [2 8] [- - - -]
    elif opcode == 0x3E:
        A_set(mem_read(PC + 1))
        PC2()
        return

    #CCF [1 4] [- 0 0 ~C]
    elif opcode == 0x3F:
        unset_flag(F_N)
        unset_flag(F_H)
        if flag_is_set(F_C):
            unset_flag(F_C)
        else:
            set_flag(F_C)
        PC1()
        return

    #LD B, B [1 4] [- - - -]
    elif opcode == 0x40:
        PC1()
        return

    #LD B, C [1 4] [- - - -]
    elif opcode == 0x41:
        B_set(C)
        PC1()
        return

    #LD B, D [1 4] [- - - -]
    elif opcode == 0x42:
        B_set(D)
        PC1()
        return

    #LD B, E [1 4] [- - - -]
    elif opcode == 0x43:
        B_set(E)
        PC1()
        return

    #LD B, H [1 4] [- - - -]
    elif opcode == 0x44:
        B_set(H)
        PC1()
        return

    #LD B, L [1 4] [- - - -]
    elif opcode == 0x45:
        B_set(L)
        PC1()
        return

    #LD B, (HL) [1 8] [- - - -]
    elif opcode == 0x46:
        B_set(mem_read(HL()))
        PC1()
        return

    #LD B, A [1 4] [- - - -]
    elif opcode == 0x47:
        B_set(A)
        PC1()
        return

    #LD C, B [1 4] [- - - -]
    elif opcode == 0x48:
        C_set(B)
        PC1()
        return

    #LD C, C [1 4] [- - - -]
    elif opcode == 0x49:
        PC1()
        return

    #LD C, D [1 4] [- - - -]
    elif opcode == 0x4A:
        C_set(D)
        PC1()
        return

    #LD C, E [1 4] [- - - -]
    elif opcode == 0x4B:
        C_set(E)
        PC1()
        return

    #LD C, H [1 4] [- - - -]
    elif opcode == 0x4C:
        C_set(H)
        PC1()
        return
    
    #LD C, L [1 4] [- - - -]
    elif opcode == 0x4D:
        C_set(L)
        PC1()
        return

    #LD C, (HL) [1 8] [- - - -]
    elif opcode == 0x4E:
        C_set(mem_read(HL()))
        PC1()
        return

    #LD C, A [1 4] [- - - -]
    elif opcode == 0x4F:
        C_set(A)
        PC1()
        return

    #LD D, B [1 4] [- - - -]
    elif opcode == 0x50:
        D_set(B)
        PC1()
        return
    
    #LD D, C [1 4] [- - - -]
    elif opcode == 0x51:
        D_set(C)
        PC1()
        return

    #LD D, D [1 4] [- - - -]
    elif opcode == 0x52:
        D_set(D)
        PC1()
        return

    #LD D, E [1 4] [- - - -]
    elif opcode == 0x53:
        D_set(E)
        PC1()
        return

    #LD D, H [1 4] [- - - -]
    elif opcode == 0x54:
        D_set(H)
        PC1()
        return

    #LD D, L [1 4] [- - - -]
    elif opcode == 0x55:
        D_set(L)
        PC1()
        return

    #LD D, (HL) [1 8] [- - - -]
    elif opcode == 0x56:
        D_set(mem_read(HL()))
        PC1()
        return

    #LD D, A [1 4] [- - - -]
    elif opcode == 0x57:
        D_set(A)
        PC1()
        return

    #LD E, B [1 4] [- - - -]
    elif opcode == 0x58:
        E_set(B)
        PC1()
        return

    #LD E, C [1 4] [- - - -]
    elif opcode == 0x59:
        E_set(C)
        PC1()
        return

    #LD E, D [1 4] [- - - -]
    elif opcode == 0x5A:
        E_set(D)
        PC1()
        return

    #LD E, E [1 4] [- - - -]
    elif opcode == 0x5B:
        PC1()
        return

    #LD E, H [1 4] [- - - -]
    elif opcode == 0x5C:
        E_set(H)
        PC1()
        return

    #LD E, L [1 4] [- - - -]
    elif opcode == 0x5D:
        E_set(L)
        PC1()
        return

    #LD E, (HL) [1 8] [- - - -]
    elif opcode == 0x5E:
        E_set(mem_read(HL()))
        PC1()
        return

    #LD E, A [1 4] [- - - -]
    elif opcode == 0x5F:
        E_set(A)
        PC1()
        return

    #LD H, B [1 4] [- - - -]
    elif opcode == 0x60:
        H_set(B)
        PC1()
        return

    #LD H, C [1 4] [- - - -]
    elif opcode == 0x61:
        H_set(C)
        PC1()
        return

    #LD H, D [1 4] [- - - -]
    elif opcode == 0x62:
        H_set(D)
        PC1()
        return

    #LD H, E [1 4] [- - - -]
    elif opcode == 0x63:
        H_set(E)
        PC1()
        return

    #LD H, H [1 4] [- - - -]
    elif opcode == 0x64:
        H_set(H)
        PC1()
        return

    #LD H, L [1 4] [- - - -]
    elif opcode == 0x65:
        H_set(L)
        PC1()
        return

    #LD H, (HL) [1 4] [- - - -]
    elif opcode == 0x66:
        H_set(mem_read(HL()))
        PC1()
        return

    #LD H, A [1 4] [- - - -]
    elif opcode == 0x67:
        H_set(A)
        PC1()
        return

    #LD L, B [1 4] [- - - -]
    elif opcode == 0x68:
        L_set(B)
        PC1()
        return

    #LD L, C [1 4] [- - - -]
    elif opcode == 0x69:
        L_set(C)
        PC1()
        return

    #LD L, D [1 4] [- - - -]
    elif opcode == 0x6A:
        L_set(D)
        PC1()
        return

    #LD L, E [1 4] [- - - -]
    elif opcode == 0x6B:
        L_set(E)
        PC1()
        return

    #LD L, H [1 4] [- - - -]
    elif opcode == 0x6C:
        L_set(H)
        PC1()
        return

    #LD L, L [1 4] [- - - -]
    elif opcode == 0x6D:
        PC1()
        return

    #LD L, (HL) [1 8] [- - - -]
    elif opcode == 0x6E:
        L_set(mem_read(HL()))
        PC1()
        return

    #LD L, A [1 4] [- - - -]
    elif opcode == 0x6F:
        L_set(A)
        PC1()
        return

    #LD (HL), B [1 8] [- - - -]
    elif opcode == 0x70:
        mem_set(HL(), B)
        PC1()
        return

    #LD (HL), C [1 8] [- - - -]
    elif opcode == 0x71:
        mem_set(HL(), C)
        PC1()
        return

    #LD (HL), D [1 8] [- - - -]
    elif opcode == 0x72:
        mem_set(HL(), D)
        PC1()
        return

    #LD (HL), E [1 8] [- - - -]
    elif opcode == 0x73:
        mem_set(HL(), E)
        PC1()
        return

    #LD (HL), H [1 8] [- - - -]
    elif opcode == 0x74:
        mem_set(HL(), H)
        PC1()
        return

    #LD (HL), L [1 8] [- - - -]
    elif opcode == 0x75:
        mem_set(HL(), L)
        PC1()
        return

    #HALT [1 4] [- - - -] ???
    elif opcode == 0x76:
        if not interrupts_enabled:
            PC2() #skip next instruction
        else:
            PC1()
        return

    #LD (HL), A [1 8] [- - - -]
    elif opcode == 0x77:
        mem_set(HL(), A)
        PC1()
        return

    #LD A, B [1 4] [- - - -]
    elif opcode == 0x78:
        A_set(B)
        PC1()
        return

    #LD A, C [1 4] [- - - -]
    elif opcode == 0x79:
        A_set(C)
        PC1()
        return
    
    #LD A, D [1 4] [- - - -]
    elif opcode == 0x7A:
        A_set(D)
        PC1()
        return
    
    #LD A, E [1 4] [- - - -]
    elif opcode == 0x7B:
        A_set(E)
        PC1()
        return

    #LD A, H [1 4] [- - - -]
    elif opcode == 0x7C:
        A_set(H)
        PC1()
        return

    #LD A, L [1 4] [- - - -]
    elif opcode == 0x7D:
        A_set(L)
        PC1()
        return

    #LD A, (HL) [1 8] [- - - -]
    elif opcode == 0x7E:
        A_set(mem_read(HL()))
        PC1()
        return

    #LD A, A [1 4] [- - - -]
    elif opcode == 0x7F:
        PC1()
        return

    #ADD A, B [1 4] [Z 0 H C]
    elif opcode == 0x80:
        halfcarry_flag(A, A + B)
        carry_flag(A + B)
        A_set(A + B)
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADD A, C [1 4] [Z 0 H C]
    elif opcode == 0x81:
        halfcarry_flag(A, A + C)
        carry_flag(A + C)
        A_set(A + C)
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADD A, D [1 4] [Z 0 H C]
    elif opcode == 0x82:
        halfcarry_flag(A, A + D)
        carry_flag(A + D)
        A_set(A + D)
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADD A, E [1 4] [Z 0 H C]
    elif opcode == 0x83:
        halfcarry_flag(A, A + E)
        carry_flag(A + E)
        A_set(A + E)
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADD A, H [1 4] [Z 0 H C]
    elif opcode == 0x84:
        halfcarry_flag(A, A + H)
        carry_flag(A + H)
        A_set(A + H)
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADD A, L [1 4] [Z 0 H C]
    elif opcode == 0x85:
        halfcarry_flag(A, A + L)
        carry_flag(A + L)
        A_set(A + L)
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADD A, (HL) [1 8] [Z 0 H C]
    elif opcode == 0x86:
        halfcarry_flag(A, A + mem_read(HL()))
        carry_flag(A + mem_read(HL()))
        A_set(A + mem_read(HL()))
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADD A, A [1 4] [Z 0 H C]
    elif opcode == 0x87:
        halfcarry_flag(A, A + A)
        carry_flag(A + A)
        A_set(A + A)
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADC A, B [1 4] [Z 0 H C]
    elif opcode == 0x88:
        halfcarry_flag(A, A + B + nonzero_to_one(F & F_C))
        carry_flag(A + B + nonzero_to_one(F & F_C))
        A_set(A + B + nonzero_to_one(F & F_C))
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADC A, C [1 4] [Z 0 H C]
    elif opcode == 0x89:
        halfcarry_flag(A, A + C + nonzero_to_one(F & F_C))
        carry_flag(A + C + nonzero_to_one(F & F_C))
        A_set(A + C + nonzero_to_one(F & F_C))
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADC A, D [1 4] [Z 0 H C]
    elif opcode == 0x8A:
        halfcarry_flag(A, A + D + nonzero_to_one(F & F_C))
        carry_flag(A + D + nonzero_to_one(F & F_C))
        A_set(A + D + nonzero_to_one(F & F_C))
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADC A, E [1 4] [Z 0 H C]
    elif opcode == 0x8B:
        halfcarry_flag(A, A + E + nonzero_to_one(F & F_C))
        carry_flag(A + E + nonzero_to_one(F & F_C))
        A_set(A + E + nonzero_to_one(F & F_C))
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADC A, H [1 4] [Z 0 H C]
    elif opcode == 0x8C:
        halfcarry_flag(A, A + H + nonzero_to_one(F & F_C))
        carry_flag(A + H + nonzero_to_one(F & F_C))
        A_set(A + H + nonzero_to_one(F & F_C))
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADC A, L [1 4] [Z 0 H C]
    elif opcode == 0x8D:
        halfcarry_flag(A, A + L + nonzero_to_one(F & F_C))
        carry_flag(A + L + nonzero_to_one(F & F_C))
        A_set(A + L + nonzero_to_one(F & F_C))
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADC A, (HL) [1 8] [Z 0 H C]
    elif opcode == 0x8E:
        halfcarry_flag(A, A + mem_read(HL()) + nonzero_to_one(F & F_C))
        carry_flag(A + mem_read(HL()) + nonzero_to_one(F & F_C))
        A_set(A + mem_read(HL()) + nonzero_to_one(F & F_C))
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #ADC A, A [1 4] [Z 0 H C]
    elif opcode == 0x8F:
        halfcarry_flag(A, A + A + nonzero_to_one(F & F_C))
        carry_flag(A + A + nonzero_to_one(F & F_C))
        A_set(A + A + nonzero_to_one(F & F_C))
        zero_flag(A)
        unset_flag(F_N)
        PC1()
        return

    #SUB B [1 4] [Z 1 H C]
    elif opcode == 0x90:
        halfcarry_flag(A, A - B, True)
        if B > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - B)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SUB C [1 4] [Z 1 H C]
    elif opcode == 0x91:
        halfcarry_flag(A, A - C, True)
        if C > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - C)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SUB D [1 4] [Z 1 H C]
    elif opcode == 0x92:
        halfcarry_flag(A, A - D, True)
        if D > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - D)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SUB E [1 4] [Z 1 H C]
    elif opcode == 0x93:
        halfcarry_flag(A, A - E, True)
        if E > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - E)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SUB H [1 4] [Z 1 H C]
    elif opcode == 0x94:
        halfcarry_flag(A, A - H, True)
        if H > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - H)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SUB L [1 4] [Z 1 H C]
    elif opcode == 0x95:
        halfcarry_flag(A, A - L, True)
        if L > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - L)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SUB (HL) [1 8] [Z 1 H C]
    elif opcode == 0x96:
        halfcarry_flag(A, A - mem_read(HL()), True)
        if mem_read(HL()) > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - mem_read(HL()))
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SUB A [1 4] [Z 1 H C]
    elif opcode == 0x97:
        halfcarry_flag(A, A - A, True)
        unset_flag(F_C)
        A_set(A - A)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SBC A, B [1 4] [Z 1 H C]
    elif opcode == 0x98:
        sub = B + nonzero_to_one(F & F_C)
        halfcarry_flag(A, A - sub, True)
        if sub > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - sub)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SBC A, C [1 4] [Z 1 H C]
    elif opcode == 0x99:
        sub = C + nonzero_to_one(F & F_C)
        halfcarry_flag(A, A - sub, True)
        if sub > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - sub)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SBC A, D [1 4] [Z 1 H C]
    elif opcode == 0x9A:
        sub = D + nonzero_to_one(F & F_C)
        halfcarry_flag(A, A - sub, True)
        if sub > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - sub)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SBC A, E [1 4] [Z 1 H C]
    elif opcode == 0x9B:
        sub = E + nonzero_to_one(F & F_C)
        halfcarry_flag(A, A - sub, True)
        if sub > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - sub)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SBC A, H [1 4] [Z 1 H C]
    elif opcode == 0x9C:
        sub = H + nonzero_to_one(F & F_C)
        halfcarry_flag(A, A - sub, True)
        if sub > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - sub)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SBC A, L [1 4] [Z 1 H C]
    elif opcode == 0x9D:
        sub = L + nonzero_to_one(F & F_C)
        halfcarry_flag(A, A - sub, True)
        if sub > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - sub)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SBC A, (HL) [1 8] [Z 1 H C]
    elif opcode == 0x9E:
        sub = mem_read(HL()) + nonzero_to_one(F & F_C)
        halfcarry_flag(A, A - sub, True)
        if sub > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - sub)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #SBC A, A [1 4] [Z 1 H C]
    elif opcode == 0x9F:
        sub = A + nonzero_to_one(F & F_C)
        halfcarry_flag(A, A - sub, True)
        if sub > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - sub)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #AND B [1 4] [Z 0 1 0]
    elif opcode == 0xA0:
        A_set(A & B)
        zero_flag(A)
        unset_flag(F_N)
        set_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #AND C [1 4] [Z 0 1 0]
    elif opcode == 0xA1:
        A_set(A & C)
        zero_flag(A)
        unset_flag(F_N)
        set_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #AND D [1 4] [Z 0 1 0]
    elif opcode == 0xA2:
        A_set(A & D)
        zero_flag(A)
        unset_flag(F_N)
        set_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #AND E [1 4] [Z 0 1 0]
    elif opcode == 0xA3:
        A_set(A & E)
        zero_flag(A)
        unset_flag(F_N)
        set_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #AND H [1 4] [Z 0 1 0]
    elif opcode == 0xA4:
        A_set(A & H)
        zero_flag(A)
        unset_flag(F_N)
        set_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #AND L [1 4] [Z 0 1 0]
    elif opcode == 0xA5:
        A_set(A & L)
        zero_flag(A)
        unset_flag(F_N)
        set_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #AND (HL) [1 8] [Z 0 1 0]
    elif opcode == 0xA6:
        A_set(A & mem_read(HL()))
        zero_flag(A)
        unset_flag(F_N)
        set_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #AND A [1 8] [Z 0 1 0]
    elif opcode == 0xA7:
        zero_flag(A)
        unset_flag(F_N)
        set_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #XOR B [1 4] [Z 0 0 0]
    elif opcode == 0xA8:
        A_set(A ^ B)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #XOR C [1 4] [Z 0 0 0]
    elif opcode == 0xA9:
        A_set(A ^ C)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #XOR D [1 4] [Z 0 0 0]
    elif opcode == 0xAA:
        A_set(A ^ D)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #XOR E [1 4] [Z 0 0 0]
    elif opcode == 0xAB:
        A_set(A ^ E)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #XOR H [1 4] [Z 0 0 0]
    elif opcode == 0xAC:
        A_set(A ^ H)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #XOR L [1 4] [Z 0 0 0]
    elif opcode == 0xAD:
        A_set(A ^ L)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #XOR (HL) [1 8] [Z 0 0 0]
    elif opcode == 0xAE:
        A_set(A ^ mem_read(HL()))
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #XOR A [1 8] [Z 0 0 0]
    elif opcode == 0xAF:
        A_set(A ^ A)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #OR B [1 4] [Z 0 0 0]
    elif opcode == 0xB0:
        A_set(A | B)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #OR C [1 4] [Z 0 0 0]
    elif opcode == 0xB1:
        A_set(A | C)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #OR D [1 4] [Z 0 0 0]
    elif opcode == 0xB2:
        A_set(A | D)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #OR E [1 4] [Z 0 0 0]
    elif opcode == 0xB3:
        A_set(A | E)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #OR H [1 4] [Z 0 0 0]
    elif opcode == 0xB4:
        A_set(A | H)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #OR L [1 4] [Z 0 0 0]
    elif opcode == 0xB5:
        A_set(A | L)
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #OR (HL) [1 8] [Z 0 0 0]
    elif opcode == 0xB6:
        A_set(A | mem_read(HL()))
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #OR A [1 4] [Z 0 0 0]
    elif opcode == 0xB7:
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC1()
        return

    #CP B [1 4] [Z 1 H C]
    elif opcode == 0xB8:
        halfcarry_flag(A, A - B, True)
        if B > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        zero_flag(A - B)
        set_flag(F_N)
        PC1()
        return

    #CP C [1 4] [Z 1 H C]
    elif opcode == 0xB9:
        halfcarry_flag(A, A - C, True)
        if C > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        zero_flag(A - C)
        set_flag(F_N)
        PC1()
        return

    #CP D [1 4] [Z 1 H C]
    elif opcode == 0xBA:
        halfcarry_flag(A, A - D, True)
        if D > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        zero_flag(A - D)
        set_flag(F_N)
        PC1()
        return

    #CP E [1 4] [Z 1 H C]
    elif opcode == 0xBB:
        halfcarry_flag(A, A - E, True)
        if E > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        zero_flag(A - E)
        set_flag(F_N)
        PC1()
        return

    #CP H [1 4] [Z 1 H C]
    elif opcode == 0xBC:
        halfcarry_flag(A, A - H, True)
        if H > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        zero_flag(A - H)
        set_flag(F_N)
        PC1()
        return

    #CP L [1 4] [Z 1 H C]
    elif opcode == 0xBD:
        halfcarry_flag(A, A - L, True)
        if L > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        zero_flag(A - L)
        set_flag(F_N)
        PC1()
        return

    #CP (HL) [1 8] [Z 1 H C]
    elif opcode == 0xBE:
        halfcarry_flag(A, A - mem_read(HL()), True)
        if mem_read(HL()) > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        zero_flag(A - mem_read(HL()))
        set_flag(F_N)
        PC1()
        return

    #CP A [1 4] [Z 1 H C]
    elif opcode == 0xBF:
        halfcarry_flag(A, A - A, True)
        if A > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        zero_flag(A - A)
        set_flag(F_N)
        PC1()
        return

    #RET NZ [1 20/8] [- - - -]
    elif opcode == 0xC0:
        if not flag_is_set(F_Z):
            ret()
            return
        PC1()
        return

    #POP BC [1 12] [- - - -]
    elif opcode == 0xC1:
        C_set(pop_stack())
        B_set(pop_stack())
        PC1()
        return

    #JP NZ, a16 [3 16/12] [- - - -]
    elif opcode == 0xC2:
        if not flag_is_set(F_Z):
            PC_set(compose(mem_read(PC + 2), mem_read(PC + 1)))
            return
        else:
            PC3()
            return

    #PUSH BC [1 16] [- - - -]
    elif opcode == 0xC5:
        push_stack(B)
        push_stack(C)
        PC1()
        return

    #ADD A, d8 [2 8] [Z 0 H C]
    elif opcode == 0xC6:
        halfcarry_flag(A, A + mem_read(PC + 1))
        carry_flag(A + mem_read(PC + 1))
        A_set(A + mem_read(PC + 1))
        zero_flag(A)
        unset_flag(F_N)
        PC2()
        return

    #RST 00H [1 16] [- - - -]
    elif opcode == 0xC7:
        rst(0x00)
        PC1()
        return

    #RET Z [1 20/8] [- - - -]
    elif opcode == 0xC8:
        if flag_is_set(F_Z):
            ret()
            return
        PC1()
        return

    #RET [1 16] [- - - -]
    elif opcode == 0xC9:
        ret()
        return

    #JP Z, a16 [3 16/12] [- - - -]
    elif opcode == 0xCA:
        if flag_is_set(F_Z):
            PC_set(compose(mem_read(PC + 2), mem_read(PC + 1)))
            return
        else:
            PC3()
            return

    #PREFIX CB [1 4] [- - - -] ???
    elif opcode == 0xCB:
        PC1()
        opcode = mem_read(PC)

        #RLC B [2 8] [Z 0 0 C] ???
        if opcode == 0x00:
            PC1()
            return

        #SET 7, A [2 8] [- - - -]
        if opcode == 0xFF:
            A_set(A | BIT_7)
            PC1()
            return

    #ADC A, d8 [2 8] [Z 0 H C]
    elif opcode == 0xCE:
        halfcarry_flag(A, A + mem_read(PC + 1) + nonzero_to_one(F & F_C))
        carry_flag(A + mem_read(PC + 1) + nonzero_to_one(F & F_C))
        A_set(A + mem_read(PC + 1) + nonzero_to_one(F & F_C))
        zero_flag(A)
        unset_flag(F_N)
        PC2()
        return

    #RST 08H [1 16] [- - - -]
    elif opcode == 0xCF:
        rst(0x08)
        PC1()
        return

    #RET NC [1 20/8] [- - - -]
    elif opcode == 0xD0:
        if not flag_is_set(F_C):
            ret()
            return
        PC1()
        return

    #POP DE [1 12] [- - - -]
    elif opcode == 0xD1:
        E_set(pop_stack())
        D_set(pop_stack())
        PC1()
        return

    #JP NC, a16 [3 16/12] [- - - -]
    elif opcode == 0xD2:
        if not flag_is_set(F_C):
            PC_set(compose(mem_read(PC + 2), mem_read(PC + 1)))
            return
        else:
            PC3()
            return

    #Empty opcode
    elif opcode == 0xD3:
        err_msg("Empty opcode (D3)")
        PC1()
        return

    #PUSH DE [1 16] [- - - -]
    elif opcode == 0xD5:
        push_stack(D)
        push_stack(E)
        PC1()
        return

    #RST 10H [1 16] [- - - -]
    elif opcode == 0xD7:
        rst(0x10)
        PC1()
        return

    #RET C [1 20/8] [- - - -]
    elif opcode == 0xD8:
        if flag_is_set(F_C):
            ret()
            return
        PC1()
        return

    #RETI [1 16] [- - - -]
    elif opcode == 0xD9:
        ret()
        enable_interrupts()
        return

    #JP C, a16 [3 16/12] [- - - -]
    elif opcode == 0xDA:
        if not flag_is_set(F_C):
            PC_set(compose(mem_read(PC + 2), mem_read(PC + 1)))
            return
        else:
            PC3()
            return
    
    #Empty opcode
    elif opcode == 0xDB:
        err_msg("Empty opcode (DB)")
        PC1()
        return

    #Empty opcode
    elif opcode == 0xDD:
        err_msg("Empty opcode (DD)")
        PC1()
        return

    #SBC A, d8 [2 8] [Z 1 H C]
    elif opcode == 0xDE:
        sub = mem_read(PC + 1) + nonzero_to_one(F & F_C)
        halfcarry_flag(A, A - sub, True)
        if sub > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        A_set(A - sub)
        zero_flag(A)
        set_flag(F_N)
        PC1()
        return

    #RST 18H [1 16] [- - - -]
    elif opcode == 0xDF:
        rst(0x18)
        PC1()
        return

    #LDH (a8), A [2 12] [- - - -]
    elif opcode == 0xE0:
        mem_set(0xFF00 + mem_read(PC + 1), A)
        PC2()
        return

    #POP HL [1 12] [- - - -]
    elif opcode == 0xE1:
        L_set(pop_stack())
        H_set(pop_stack())
        PC1()
        return

    #LDH (C), A [1 ?] [- - - -]
    elif opcode == 0xE2:
        mem_set(0xFF00 + C, A)
        PC1()
        return

    #Empty opcode
    elif opcode == 0xE3:
        err_msg("Empty opcode (E3)")
        PC1()
        return

    #Empty opcode
    elif opcode == 0xE4:
        err_msg("Empty opcode (E4)")
        PC1()
        return

    #PUSH HL [1 16] [- - - -]
    elif opcode == 0xE5:
        push_stack(H)
        push_stack(L)
        PC1()
        return

    #AND d8 [2 8] [Z 0 1 0]
    elif opcode == 0xE6:
        A_set(A & mem_read(PC + 1))
        zero_flag(A)
        unset_flag(F_N)
        set_flag(F_H)
        unset_flag(F_C)
        PC2()
        return

    #RST 20H [1 16] [- - - -]
    elif opcode == 0xE7:
        rst(0x20)
        PC1()
        return

    #JP (HL) [1 4] [- - - -]
    elif opcode == 0xE9:
        PC_set(HL())
        return

    #LD (a16), A
    elif opcode == 0xEA:
        mem_set(compose(mem_read(PC + 2), mem_read(PC + 1)), A)
        PC3()
        return

    #Empty opcode
    elif opcode == 0xEB:
        err_msg("Empty opcode (EB)")
        PC1()
        return

    #Empty opcode
    elif opcode == 0xEC:
        err_msg("Empty opcode (EC)")
        PC1()
        return

    #Empty opcode
    elif opcode == 0xED:
        err_msg("Empty opcode (ED)")
        PC1()
        return

    #XOR d8 [2 8] [Z 0 0 0]
    elif opcode == 0xEE:
        A_set(A ^ mem_read(PC + 1))
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC2()
        return

    #RST 28H [1 16] [- - - -]
    elif opcode == 0xEF:
        rst(0x28)
        PC1()
        return

    #LDH A, (a8) [2 12] [- - - -]
    elif opcode == 0xF0:
        A_set(mem_read(0xFF00 + mem_read(PC + 1)))
        PC2()
        return

    #POP AF [1 12] [- - - -]
    elif opcode == 0xF1:
        F_set(pop_stack())
        A_set(pop_stack())
        PC1()
        return

    #LDH A, (C) [1 ?] [- - - -]
    elif opcode == 0xF2:
        A_set(mem_read(0xFF00 + C))
        PC1()
        return

    #DI [1 4] [- - - -]
    elif opcode == 0xF3:
        disable_interrupts()
        PC1()
        return

    #Empty opcode
    elif opcode == 0xF4:
        err_msg("Empty opcode (F4)")
        PC1()
        return

    #PUSH AF [1 16] [- - - -]
    elif opcode == 0xF5:
        push_stack(A)
        push_stack(F)
        PC1()
        return

    #OR d8 [2 8] [Z 0 0 0]
    elif opcode == 0xF6:
        A_set(A | mem_read(PC + 1))
        zero_flag(A)
        unset_flag(F_N)
        unset_flag(F_H)
        unset_flag(F_C)
        PC2()
        return

    #RST 30H [1 16] [- - - -]
    elif opcode == 0xF7:
        rst(0x30)
        PC1()
        return

    #LD SP, HL [1 8] [- - - -]
    elif opcode == 0xF9:
        SP_set(HL())
        return

    #LD (a16), A
    elif opcode == 0xFA:
        A_set(mem_read(compose(mem_read(PC + 2), mem_read(PC + 1))))
        PC3()
        return

    #EI [1 4] [- - - -]
    elif opcode == 0xFB:
        enable_interrupts()
        PC1()
        return

    #Empty opcode
    elif opcode == 0xFC:
        err_msg("Empty opcode (FC)")
        PC1()
        return

    #Empty opcode
    elif opcode == 0xFD:
        err_msg("Empty opcode (FD)")
        PC1()
        return

    #CP d8 [2 8] [Z 1 H C]
    elif opcode == 0xB8:
        halfcarry_flag(A, A - mem_read(PC + 1), True)
        if mem_read(PC + 1) > A:
            set_flag(F_C)
        else:
            unset_flag(F_C)
        zero_flag(A - mem_read(PC + 1))
        set_flag(F_N)
        PC1()
        return

    #RST 38H [1 16] [- - - -]
    elif opcode == 0xFF:
        rst(0x38)
        PC1()
        return

    else:
        err_msg("Invalid opcode (" + hex(opcode) + ")", True)

def main():
    #mem_init()
    #print(len(memory))
    load_cart('blue.gb', True)

if __name__ == '__main__':
    main()