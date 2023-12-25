# Green GB palette
import time


palette = [(0, 63, 0), (46, 115, 32), (140, 191, 10), (160, 207, 10)]

waiting_on_button = False
interrupts_enabled = False
print_assembly = True

def enable_interrupts():
	global interrupts_enabled
	interrupts_enabled = True

def disable_interrupts():
	global interrupts_enabled
	interrupts_enabled = False

def print_asm(message: str) -> None:
	if print_assembly:
		print(message)

def print_registers() -> None:
	print(f"A: {A:02X}")
	print(f"B: {B:02X} C: {C:02X}")
	print(f"D: {D:02X} E: {E:02X}")
	print(f"H: {H:02X} L: {L:02X}")
	print(f"F: {F:02X} (Z: {flag_is_set(F_Z)} N: {flag_is_set(F_N)} H: {flag_is_set(F_H)} C: {flag_is_set(F_C)})")
	print(f"PC: {PC:04X} SP: {SP:04X}")

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
		return -((val ^ 0xFF) + 1)
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
	global memory
	for i in range(0, 0xFFFF):
		memory[i] = 0

def mem_set(add, val):
	"""
	Sets memory at the specified address to the given value.

	Takes into account the memory constraints of
	the processor and throws appropriate errors.
	"""
	global memory
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
	else:
		return memory[add]

SP = 0x0000
def SP_set(val):
	"""
	Sets the stack pointer to a value.
	"""
	global SP
	SP = val & 0xFFFF

def push_stack(val):
	global SP
	SP -= 1
	mem_set(SP, val & 0xFF)

def pop_stack(val):
	global SP
	SP += 1
	return mem_read(SP)

#=============
#= CARTRIDGE =
#=============
cart_data = []
cart_name = ""

def load_bootstrap(filename: str, verbose: bool=False) -> None:
	"""
	Loads the gameboy bootstrap ROM with the specified filename into memory.
	"""
	with open(filename, 'rb') as f:
		bootstrap = f.read()
		for i in range(0, len(bootstrap)):
			memory[i] = int(bootstrap[i])
	if verbose:
		print("=================")
		print("= Bootstrap load complete =")
		print(f"Name: {filename}")
		print("Size: " + byte_string(len(bootstrap)))
		print("=================")

def load_cart(filename: str, verbose: bool=False) -> None:
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

# def run_program(filename:str) -> None:
# 	"""
# 	Runs the program in the specified file.
# 	"""
# 	#load_cart(filename)
# 	program_data = []
# 	with open(filename, 'rb') as f:
# 		program_data = f.read()

# 	PC_set(0x0100)
# 	while True:
# 		print(f"PC: {PC:04X} {PC} mem[PC] = {memory[PC]:02X}")
# 		parse_opcode(mem_read(PC))
# 		time.sleep(5)

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

def call(nn):
	"""
	"""
	global PC
	push_stack(PC)
	PC = nn

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
		print_asm(f"[00] NOP")
		PC1()
		return

	#LD BC, d16 [3 12] [- - - -]
	elif opcode == 0x01:
		print_asm(f"[01] LD BC, d16 => LD BC, {memory[PC + 2]:02X}{memory[PC + 1]:02X}")

		B_set(memory[PC + 2])
		C_set(memory[PC + 1])
		PC3()
		return

	#LD (BC), A [1 8] [- - - -]
	elif opcode == 0x02:
		print_asm(f"[02] LD (BC), A => LD {BC()}, {A}")
		mem_set(BC(), A)
		PC1()
		return

	#INC BC [1 8] [- - - -]
	elif opcode == 0x03:
		print_asm(f"[03] INC BC")
		BC_set(BC() + 1)
		PC1()
		return

	#INC B [1 4] [Z 0 H -]
	elif opcode == 0x04:
		print_asm(f"[04] INC B")
		halfcarry_flag(B, B + 1)
		B_set(B + 1)
		zero_flag(B)
		unset_flag(F_N)
		PC1()
		return

	#DEC B [1 4] [Z 1 H -]
	elif opcode == 0x05:
		print_asm(f"[05] DEC B")
		halfcarry_flag(B, B - 1, True)
		B_set(B - 1)
		unset_flag(F_N)
		zero_flag(B)
		PC1()
		return

	#LD B, d8 [2 8] [- - - -]
	elif opcode == 0x06:
		print_asm(f"[06] LD B, d8 => LD B, {memory[PC + 1]:02X}")
		B_set(mem_read(PC + 1))
		PC2()
		return

	#RLCA [1 4] [Z 0 0 C]
	elif opcode == 0x07:
		print_asm(f"[07] RLCA")
		left_bit = nonzero_to_one(A & BIT_7)
		set_flag_to(F_C, left_bit)
		A_set(RL(A))
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		return

	#LD (a16), SP [3 20] [- - - -]
	elif opcode == 0x08:
		print_asm(f"[08] LD (a16), SP => LD {compose(memory[PC + 2], memory[PC + 1]):04X}, SP")
		add = compose(mem_read(PC + 2), mem_read(PC + 1))
		mem_set(add, SP & 0xFF)
		mem_set(add + 1, (SP & 0xFF00) >> 8)
		PC3()
		return

	#ADD HL, BC [1 8] [- 0 H C]
	elif opcode == 0x09:
		print_asm(f"[09] ADD HL, BC")
		unset_flag(F_N)
		halfcarry_flag_16(HL(), HL() + BC())
		carry_flag_16(HL(), HL() + BC())
		HL_set(HL() + BC())
		PC1()
		return

	#LD A, (BC) [1 8] [- - - -]
	elif opcode == 0x0A:
		print_asm(f"[0A] LD A, (BC) => LD A, {BC()}")
		A_set(mem_read(BC()))
		PC1()
		return

	#DEC BC [1 8] [- - - -]
	elif opcode == 0x0B:
		print_asm(f"[0B] DEC BC")
		BC_set(BC() - 1)
		PC1()
		return

	#INC C [1 4] [Z 0 H -]
	elif opcode == 0x0C:
		print_asm(f"[0C] INC C")
		halfcarry_flag(C, C + 1)
		C_set(C + 1)
		unset_flag(F_N)
		zero_flag(C)
		PC1()
		return

	#LD C, d8 [2 8] [- - - -]
	elif opcode == 0x0E:
		print_asm(f"[0E] LD C, d8 => LD C, {memory[PC + 1]:02X}")
		C_set(mem_read(PC + 1))
		PC2()
		return

	#RRCA [1 4] [0 0 0 C] ???
	elif opcode == 0x0F:
		print_asm(f"[0F] RRCA")
		# TODO
		PC1()
		return

	#STOP (+NOP) [1 4] [- - - -] ???
	elif opcode == 0x10:
		print_asm(f"[10] STOP")
		global waiting_on_button
		waiting_on_button = True
		PC1()
		return

	#LD DE, d16 [3 12] [- - - -]
	elif opcode == 0x11:
		print_asm(f"[11] LD DE, d16 => LD DE, {memory[PC + 2]:02X}{memory[PC + 1]:02X}")
		D_set(memory[PC + 2])
		E_set(memory[PC + 1])
		PC3()
		return

	#LD (DE), A [1 8] [- - - -]
	elif opcode == 0x12:
		print_asm(f"[12] LD (DE), A => LD {DE()}, {A}")
		mem_set(DE(), A)
		PC1()
		return

	#INC DE [1 8] [- - - -]
	elif opcode == 0x13:
		print_asm(f"[13] INC DE")
		DE_set(DE() + 1)
		PC1()
		return

	#INC D [1 4] [Z 0 H -]
	elif opcode == 0x14:
		print_asm(f"[14] INC D")
		halfcarry_flag(D, D + 1)
		D_set(D + 1)
		zero_flag(D)
		unset_flag(F_N)
		PC1()
		return

	#LD D, d8 [2 8] [- - - -]
	elif opcode == 0x16:
		print_asm(f"[16] LD D, d8 => LD D, {memory[PC + 1]:02X}")
		D_set(mem_read(PC + 1))
		PC2()
		return

	#RLA [1 4] [Z 0 0 C]
	elif opcode == 0x17:
		print_asm(f"[17] RLA")
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
		print_asm(f"[1A] LD A, (DE) => LD A, mem[{DE()}] => LD A, {mem_read(DE())}")
		A_set(mem_read(DE()))
		PC1()
		return

	#DEC DE [1 8] [- - - -]
	elif opcode == 0x1B:
		print_asm(f"[1B] DEC DE")
		DE_set(DE() - 1)
		PC1()
		return

	#INC E [1 4] [Z 0 H -]
	elif opcode == 0x1C:
		print_asm(f"[1C] INC E")
		halfcarry_flag(E, E + 1)
		E_set(E + 1)
		unset_flag(F_N)
		zero_flag(E)
		PC1()
		return

	#LD E, d8 [2 8] [- - - -]
	elif opcode == 0x1E:
		print_asm(f"[1E] LD E, d8 => LD E, {memory[PC + 1]:02X}")
		E_set(mem_read(PC + 1))
		PC2()
		return

	#JR NZ, r8 [2 12/8] [- - - -]
	elif opcode == 0x20:
		print_asm(f"[20] JR NZ, r8 => JR NZ, {signed_8bit(mem_read(PC + 1)):02X}")
		if not flag_is_set(F_Z):
			PC_set((PC + 2) + signed_8bit(mem_read(PC + 1)))
			return
		else:
			PC2()
			return

	#LD HL, d16 [3 12] [- - - -]
	elif opcode == 0x21:
		print_asm(f"[21] LD HL, d16 => LD HL, {compose(mem_read(PC + 2), mem_read(PC + 1)):04X}")
		H_set(memory[PC + 2])
		L_set(memory[PC + 1])
		PC3()
		return

	#LD (HL+), A [1 8] [- - - -]
	elif opcode == 0x22:
		print_asm(f"[22] LD (HL+), A => LD mem[{HL():04X}], {A:02X}")
		mem_set(HL(), A)
		HL_set(HL() + 1)
		PC1()
		return

	#INC HL [1 8] [- - - -]
	elif opcode == 0x23:
		print_asm(f"[23] INC HL => INC {HL():04X}")
		HL_set(HL() + 1)
		PC1()
		return

	#INC H [1 4] [Z 0 H -]
	elif opcode == 0x24:
		print_asm(f"[24] INC H => INC {H:02X}")
		halfcarry_flag(H, H + 1)
		H_set(H + 1)
		zero_flag(H)
		unset_flag(F_N)
		PC1()
		return

	#DEC HL [1 8] [- - - -]
	elif opcode == 0x2B:
		print_asm(f"[2B] DEC HL => DEC {HL():04X}")
		HL_set(HL() - 1)
		PC1()
		return

	#INC L [1 4] [Z 0 H -]
	elif opcode == 0x2C:
		print_asm(f"[2C] INC L => INC {L:02X}")
		halfcarry_flag(L, L + 1)
		L_set(L + 1)
		unset_flag(F_N)
		zero_flag(L)
		PC1()
		return

	#LD L, d8 [2 8] [- - - -]
	elif opcode == 0x2E:
		print_asm(f"[2E] LD L, d8 => LD L, {mem_read(PC + 1):02X}")
		L_set(mem_read(PC + 1))
		PC2()
		return

	#CPL [1 4] [- 1 1 -]
	elif opcode == 0x2F:
		print_asm(f"[2F] CPL")
		A_set(~A & 0xFF)
		set_flag(F_N)
		set_flag(F_H)
		PC1()
		return

	#JR NC, r8 [2 12/8] [- - - -]
	elif opcode == 0x30:
		print_asm(f"[30] JR NC, r8 => JR NC, {signed_8bit(mem_read(PC + 1)):02X}")
		if not flag_is_set(F_N):
			PC_set(PC + signed_8bit(mem_read(PC + 1)))
			return
		else:
			PC2()
			return

	#LD SP, d16 [3 12] [- - - -]
	elif opcode == 0x31:
		print_asm(f"[31] LD SP, d16 => LD SP, {compose(mem_read(PC + 2), mem_read(PC + 1)):04X}")
		SP_set(compose(mem_read(PC + 2), mem_read(PC + 1)))
		PC3()
		return

	#LD (HL-), A [1 8] [- - - -]
	elif opcode == 0x32:
		print_asm(f"[32] LD (HL-), A => LD mem[{HL():04X}], {A}")
		mem_set(HL(), A)
		HL_set(HL() - 1)
		PC1()
		return

	#INC SP [1 8] [- - - -]
	elif opcode == 0x33:
		print_asm(f"[33] INC SP")
		SP_set(SP + 1)
		PC1()
		return

	#INC (HL) [1 12] [Z 0 H -]
	elif opcode == 0x34:
		print_asm(f"[34] INC (HL) => INC mem[{HL():04X}] => INC {mem_read(HL()):02X}")
		halfcarry_flag(mem_read(HL()), mem_read(HL()) + 1)
		mem_set(HL(), mem_read(HL()) + 1)
		zero_flag(mem_read(HL()))
		unset_flag(F_N)
		PC1()
		return

	#SCF [1 4] [- 0 0 1]
	elif opcode == 0x37:
		print_asm(f"[37] SCF")
		unset_flag(F_N)
		unset_flag(F_H)
		set_flag(C)
		PC1()
		return

	#DEC SP [1 8] [- - - -]
	elif opcode == 0x3B:
		print_asm(f"[3B] DEC SP")
		SP_set(SP - 1)
		PC1()
		return

	#INC A [1 4] [Z 0 H -]
	elif opcode == 0x3C:
		print_asm(f"[3C] INC A")
		halfcarry_flag(A, A + 1)
		A_set(A + 1)
		unset_flag(F_N)
		zero_flag(A)
		PC1()
		return

	#LD A, d8 [2 8] [- - - -]
	elif opcode == 0x3E:
		print_asm(f"[3E] LD A, d8 => LD A, {mem_read(PC + 1):02X}")
		A_set(mem_read(PC + 1))
		PC2()
		return

	#CCF [1 4] [- 0 0 ~C]
	elif opcode == 0x3F:
		print_asm(f"[3F] CCF")
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
		print_asm(f"[40] LD B, B => LD B, {B:02X}")
		PC1()
		return

	#LD B, C [1 4] [- - - -]
	elif opcode == 0x41:
		print_asm(f"[41] LD B, C => LD B, {C:02X}")
		B_set(C)
		PC1()
		return

	#LD B, D [1 4] [- - - -]
	elif opcode == 0x42:
		print_asm(f"[42] LD B, D => LD B, {D:02X}")
		B_set(D)
		PC1()
		return

	#LD B, E [1 4] [- - - -]
	elif opcode == 0x43:
		print_asm(f"[43] LD B, E => LD B, {E:02X}")
		B_set(E)
		PC1()
		return

	#LD B, H [1 4] [- - - -]
	elif opcode == 0x44:
		print_asm(f"[44] LD B, H => LD B, {H:02X}")
		B_set(H)
		PC1()
		return

	#LD B, L [1 4] [- - - -]
	elif opcode == 0x45:
		print_asm(f"[45] LD B, L => LD B, {L:02X}")
		B_set(L)
		PC1()
		return

	#LD B, (HL) [1 8] [- - - -]
	elif opcode == 0x46:
		print_asm(f"[46] LD B, (HL) => LD B, mem[{HL():04X}] => LD B, {mem_read(HL()):02X}")
		B_set(mem_read(HL()))
		PC1()
		return

	#LD B, A [1 4] [- - - -]
	elif opcode == 0x47:
		print_asm(f"[47] LD B, A => LD B, {A:02X}")
		B_set(A)
		PC1()
		return

	#LD C, B [1 4] [- - - -]
	elif opcode == 0x48:
		print_asm(f"[48] LD C, B => LD C, {B:02X}")
		C_set(B)
		PC1()
		return

	#LD C, C [1 4] [- - - -]
	elif opcode == 0x49:
		print_asm(f"[49] LD C, C => LD C, {C:02X}")
		PC1()
		return

	#LD C, D [1 4] [- - - -]
	elif opcode == 0x4A:
		print_asm(f"[4A] LD C, D => LD C, {D:02X}")
		C_set(D)
		PC1()
		return

	#LD C, E [1 4] [- - - -]
	elif opcode == 0x4B:
		print_asm(f"[4B] LD C, E => LD C, {E:02X}")
		C_set(E)
		PC1()
		return

	#LD C, H [1 4] [- - - -]
	elif opcode == 0x4C:
		print_asm(f"[4C] LD C, H => LD C, {H:02X}")
		C_set(H)
		PC1()
		return
	
	#LD C, L [1 4] [- - - -]
	elif opcode == 0x4D:
		print_asm(f"[4D] LD C, L => LD C, {L:02X}")
		C_set(L)
		PC1()
		return

	#LD C, (HL) [1 8] [- - - -]
	elif opcode == 0x4E:
		print_asm(f"[4E] LD C, (HL) => LD C, mem[{HL():04X}] => LD C, {mem_read(HL()):02X}")
		C_set(mem_read(HL()))
		PC1()
		return

	#LD C, A [1 4] [- - - -]
	elif opcode == 0x4F:
		print_asm(f"[4F] LD C, A => LD C, {A:02X}")
		C_set(A)
		PC1()
		return

	#LD D, B [1 4] [- - - -]
	elif opcode == 0x50:
		print_asm(f"[50] LD D, B => LD D, {B:02X}")
		D_set(B)
		PC1()
		return
	
	#LD D, C [1 4] [- - - -]
	elif opcode == 0x51:
		print_asm(f"[51] LD D, C => LD D, {C:02X}")
		D_set(C)
		PC1()
		return

	#LD D, D [1 4] [- - - -]
	elif opcode == 0x52:
		print_asm(f"[52] LD D, D => LD D, {D:02X}")
		D_set(D)
		PC1()
		return

	#LD D, E [1 4] [- - - -]
	elif opcode == 0x53:
		print_asm(f"[53] LD D, E => LD D, {E:02X}")
		D_set(E)
		PC1()
		return

	#LD D, H [1 4] [- - - -]
	elif opcode == 0x54:
		print_asm(f"[54] LD D, H => LD D, {H:02X}")
		D_set(H)
		PC1()
		return

	#LD D, L [1 4] [- - - -]
	elif opcode == 0x55:
		print_asm(f"[55] LD D, L => LD D, {L:02X}")
		D_set(L)
		PC1()
		return

	#LD D, (HL) [1 8] [- - - -]
	elif opcode == 0x56:
		print_asm(f"[56] LD D, (HL) => LD D, mem[{HL():04X}] => LD D, {mem_read(HL()):02X}")
		D_set(mem_read(HL()))
		PC1()
		return

	#LD D, A [1 4] [- - - -]
	elif opcode == 0x57:
		print_asm(f"[57] LD D, A => LD D, {A:02X}")
		D_set(A)
		PC1()
		return

	#LD E, B [1 4] [- - - -]
	elif opcode == 0x58:
		print_asm(f"[58] LD E, B => LD E, {B:02X}")
		E_set(B)
		PC1()
		return

	#LD E, C [1 4] [- - - -]
	elif opcode == 0x59:
		print_asm(f"[59] LD E, C => LD E, {C:02X}")
		E_set(C)
		PC1()
		return

	#LD E, D [1 4] [- - - -]
	elif opcode == 0x5A:
		print_asm(f"[5A] LD E, D => LD E, {D:02X}")
		E_set(D)
		PC1()
		return

	#LD E, E [1 4] [- - - -]
	elif opcode == 0x5B:
		print_asm(f"[5B] LD E, E => LD E, {E:02X}")
		PC1()
		return

	#LD E, H [1 4] [- - - -]
	elif opcode == 0x5C:
		print_asm(f"[5C] LD E, H => LD E, {H:02X}")
		E_set(H)
		PC1()
		return

	#LD E, L [1 4] [- - - -]
	elif opcode == 0x5D:
		print_asm(f"[5D] LD E, L => LD E, {L:02X}")
		E_set(L)
		PC1()
		return

	#LD E, (HL) [1 8] [- - - -]
	elif opcode == 0x5E:
		print_asm(f"[5E] LD E, (HL) => LD E, mem[{HL():04X}] => LD E, {mem_read(HL()):02X}")
		E_set(mem_read(HL()))
		PC1()
		return

	#LD E, A [1 4] [- - - -]
	elif opcode == 0x5F:
		print_asm(f"[5F] LD E, A => LD E, {A:02X}")
		E_set(A)
		PC1()
		return

	#LD H, B [1 4] [- - - -]
	elif opcode == 0x60:
		print_asm(f"[60] LD H, B => LD H, {B:02X}")
		H_set(B)
		PC1()
		return

	#LD H, C [1 4] [- - - -]
	elif opcode == 0x61:
		print_asm(f"[61] LD H, C => LD H, {C:02X}")
		H_set(C)
		PC1()
		return

	#LD H, D [1 4] [- - - -]
	elif opcode == 0x62:
		print_asm(f"[62] LD H, D => LD H, {D:02X}")
		H_set(D)
		PC1()
		return

	#LD H, E [1 4] [- - - -]
	elif opcode == 0x63:
		print_asm(f"[63] LD H, E => LD H, {E:02X}")
		H_set(E)
		PC1()
		return

	#LD H, H [1 4] [- - - -]
	elif opcode == 0x64:
		print_asm(f"[64] LD H, H => LD H, {H:02X}")
		H_set(H)
		PC1()
		return

	#LD H, L [1 4] [- - - -]
	elif opcode == 0x65:
		print_asm(f"[65] LD H, L => LD H, {L:02X}")
		H_set(L)
		PC1()
		return

	#LD H, (HL) [1 4] [- - - -]
	elif opcode == 0x66:
		print_asm(f"[66] LD H, (HL) => LD H, mem[{HL():04X}] => LD H, {mem_read(HL()):02X}")
		H_set(mem_read(HL()))
		PC1()
		return

	#LD H, A [1 4] [- - - -]
	elif opcode == 0x67:
		print_asm(f"[67] LD H, A => LD H, {A:02X}")
		H_set(A)
		PC1()
		return

	#LD L, B [1 4] [- - - -]
	elif opcode == 0x68:
		print_asm(f"[68] LD L, B => LD L, {B:02X}")
		L_set(B)
		PC1()
		return

	#LD L, C [1 4] [- - - -]
	elif opcode == 0x69:
		print_asm(f"[69] LD L, C => LD L, {C:02X}")
		L_set(C)
		PC1()
		return

	#LD L, D [1 4] [- - - -]
	elif opcode == 0x6A:
		print_asm(f"[6A] LD L, D => LD L, {D:02X}")
		L_set(D)
		PC1()
		return

	#LD L, E [1 4] [- - - -]
	elif opcode == 0x6B:
		print_asm(f"[6B] LD L, E => LD L, {E:02X}")
		L_set(E)
		PC1()
		return

	#LD L, H [1 4] [- - - -]
	elif opcode == 0x6C:
		print_asm(f"[6C] LD L, H => LD L, {H:02X}")
		L_set(H)
		PC1()
		return

	#LD L, L [1 4] [- - - -]
	elif opcode == 0x6D:
		print_asm(f"[6D] LD L, L => LD L, {L:02X}")
		PC1()
		return

	#LD L, (HL) [1 8] [- - - -]
	elif opcode == 0x6E:
		print_asm(f"[6E] LD L, (HL) => LD L, mem[{HL():04X}] => LD L, {mem_read(HL()):02X}")
		L_set(mem_read(HL()))
		PC1()
		return

	#LD L, A [1 4] [- - - -]
	elif opcode == 0x6F:
		print_asm(f"[6F] LD L, A => LD L, {A:02X}")
		L_set(A)
		PC1()
		return

	#LD (HL), B [1 8] [- - - -]
	elif opcode == 0x70:
		print_asm(f"[70] LD (HL), B => LD mem[{HL():04X}], {B:02X}")
		mem_set(HL(), B)
		PC1()
		return

	#LD (HL), C [1 8] [- - - -]
	elif opcode == 0x71:
		print_asm(f"[71] LD (HL), C => LD mem[{HL():04X}], {C:02X}")
		mem_set(HL(), C)
		PC1()
		return

	#LD (HL), D [1 8] [- - - -]
	elif opcode == 0x72:
		print_asm(f"[72] LD (HL), D => LD mem[{HL():04X}], {D:02X}")
		mem_set(HL(), D)
		PC1()
		return

	#LD (HL), E [1 8] [- - - -]
	elif opcode == 0x73:
		print_asm(f"[73] LD (HL), E => LD mem[{HL():04X}], {E:02X}")
		mem_set(HL(), E)
		PC1()
		return

	#LD (HL), H [1 8] [- - - -]
	elif opcode == 0x74:
		print_asm(f"[74] LD (HL), H => LD mem[{HL():04X}], {H:02X}")
		mem_set(HL(), H)
		PC1()
		return

	#LD (HL), L [1 8] [- - - -]
	elif opcode == 0x75:
		print_asm(f"[75] LD (HL), L => LD mem[{HL():04X}], {L:02X}")
		mem_set(HL(), L)
		PC1()
		return

	#HALT [1 4] [- - - -] ???
	elif opcode == 0x76:
		print_asm(f"[76] HALT")
		if not interrupts_enabled:
			PC2() #skip next instruction
		else:
			PC1()
		return

	#LD (HL), A [1 8] [- - - -]
	elif opcode == 0x77:
		print_asm(f"[77] LD (HL), A => LD mem[{HL():04X}], {A:02X}")
		mem_set(HL(), A)
		PC1()
		return

	#LD A, B [1 4] [- - - -]
	elif opcode == 0x78:
		print_asm(f"[78] LD A, B => LD A, {B:02X}")
		A_set(B)
		PC1()
		return

	#LD A, C [1 4] [- - - -]
	elif opcode == 0x79:
		print_asm(f"[79] LD A, C => LD A, {C:02X}")
		A_set(C)
		PC1()
		return
	
	#LD A, D [1 4] [- - - -]
	elif opcode == 0x7A:
		print_asm(f"[7A] LD A, D => LD A, {D:02X}")
		A_set(D)
		PC1()
		return
	
	#LD A, E [1 4] [- - - -]
	elif opcode == 0x7B:
		print_asm(f"[7B] LD A, E => LD A, {E:02X}")
		A_set(E)
		PC1()
		return

	#LD A, H [1 4] [- - - -]
	elif opcode == 0x7C:
		print_asm(f"[7C] LD A, H => LD A, {H:02X}")
		A_set(H)
		PC1()
		return

	#LD A, L [1 4] [- - - -]
	elif opcode == 0x7D:
		print_asm(f"[7D] LD A, L => LD A, {L:02X}")
		A_set(L)
		PC1()
		return

	#LD A, (HL) [1 8] [- - - -]
	elif opcode == 0x7E:
		print_asm(f"[7E] LD A, (HL) => LD A, mem[{HL():04X}] => LD A, {mem_read(HL()):02X}")
		A_set(mem_read(HL()))
		PC1()
		return

	#LD A, A [1 4] [- - - -]
	elif opcode == 0x7F:
		print_asm(f"[7F] LD A, A => LD A, {A:02X}")
		PC1()
		return

	#ADD A, B [1 4] [Z 0 H C]
	elif opcode == 0x80:
		print_asm(f"[80] ADD A, B => ADD {A:02X}, {B:02X}")
		halfcarry_flag(A, A + B)
		carry_flag(A + B)
		A_set(A + B)
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADD A, C [1 4] [Z 0 H C]
	elif opcode == 0x81:
		print_asm(f"[81] ADD A, C => ADD {A:02X}, {C:02X}")
		halfcarry_flag(A, A + C)
		carry_flag(A + C)
		A_set(A + C)
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADD A, D [1 4] [Z 0 H C]
	elif opcode == 0x82:
		print_asm(f"[82] ADD A, D => ADD {A:02X}, {D:02X}")
		halfcarry_flag(A, A + D)
		carry_flag(A + D)
		A_set(A + D)
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADD A, E [1 4] [Z 0 H C]
	elif opcode == 0x83:
		print_asm(f"[83] ADD A, E => ADD {A:02X}, {E:02X}")
		halfcarry_flag(A, A + E)
		carry_flag(A + E)
		A_set(A + E)
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADD A, H [1 4] [Z 0 H C]
	elif opcode == 0x84:
		print_asm(f"[84] ADD A, H => ADD {A:02X}, {H:02X}")
		halfcarry_flag(A, A + H)
		carry_flag(A + H)
		A_set(A + H)
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADD A, L [1 4] [Z 0 H C]
	elif opcode == 0x85:
		print_asm(f"[85] ADD A, L => ADD {A:02X}, {L:02X}")
		halfcarry_flag(A, A + L)
		carry_flag(A + L)
		A_set(A + L)
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADD A, (HL) [1 8] [Z 0 H C]
	elif opcode == 0x86:
		print_asm(f"[86] ADD A, (HL) => ADD {A:02X}, mem[{HL():04X}] => ADD {A:02X}, {mem_read(HL()):02X}")
		halfcarry_flag(A, A + mem_read(HL()))
		carry_flag(A + mem_read(HL()))
		A_set(A + mem_read(HL()))
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADD A, A [1 4] [Z 0 H C]
	elif opcode == 0x87:
		print_asm(f"[87] ADD A, A => ADD {A}, {A}")
		halfcarry_flag(A, A + A)
		carry_flag(A + A)
		A_set(A + A)
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADC A, B [1 4] [Z 0 H C]
	elif opcode == 0x88:
		print_asm(f"[88] ADC A, B => ADC {A:02X}, {B:02X}")
		halfcarry_flag(A, A + B + nonzero_to_one(F & F_C))
		carry_flag(A + B + nonzero_to_one(F & F_C))
		A_set(A + B + nonzero_to_one(F & F_C))
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADC A, C [1 4] [Z 0 H C]
	elif opcode == 0x89:
		print_asm(f"[89] ADC A, C => ADC {A:02X}, {C:02X}")
		halfcarry_flag(A, A + C + nonzero_to_one(F & F_C))
		carry_flag(A + C + nonzero_to_one(F & F_C))
		A_set(A + C + nonzero_to_one(F & F_C))
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADC A, D [1 4] [Z 0 H C]
	elif opcode == 0x8A:
		print_asm(f"[8A] ADC A, D => ADC {A:02X}, {D:02X}")
		halfcarry_flag(A, A + D + nonzero_to_one(F & F_C))
		carry_flag(A + D + nonzero_to_one(F & F_C))
		A_set(A + D + nonzero_to_one(F & F_C))
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADC A, E [1 4] [Z 0 H C]
	elif opcode == 0x8B:
		print_asm(f"[8B] ADC A, E => ADC {A:02X}, {E:02X}")
		halfcarry_flag(A, A + E + nonzero_to_one(F & F_C))
		carry_flag(A + E + nonzero_to_one(F & F_C))
		A_set(A + E + nonzero_to_one(F & F_C))
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADC A, H [1 4] [Z 0 H C]
	elif opcode == 0x8C:
		print_asm(f"[8C] ADC A, H => ADC {A:02X}, {H:02X}")
		halfcarry_flag(A, A + H + nonzero_to_one(F & F_C))
		carry_flag(A + H + nonzero_to_one(F & F_C))
		A_set(A + H + nonzero_to_one(F & F_C))
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADC A, L [1 4] [Z 0 H C]
	elif opcode == 0x8D:
		print_asm(f"[8D] ADC A, L => ADC {A:02X}, {L:02X}")
		halfcarry_flag(A, A + L + nonzero_to_one(F & F_C))
		carry_flag(A + L + nonzero_to_one(F & F_C))
		A_set(A + L + nonzero_to_one(F & F_C))
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADC A, (HL) [1 8] [Z 0 H C]
	elif opcode == 0x8E:
		print_asm(f"[8E] ADC A, (HL) => ADC {A:02X}, mem[{HL():04X}] => ADC {A:02X}, {mem_read(HL()):02X}")
		halfcarry_flag(A, A + mem_read(HL()) + nonzero_to_one(F & F_C))
		carry_flag(A + mem_read(HL()) + nonzero_to_one(F & F_C))
		A_set(A + mem_read(HL()) + nonzero_to_one(F & F_C))
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#ADC A, A [1 4] [Z 0 H C]
	elif opcode == 0x8F:
		print_asm(f"[8F] ADC A, A => ADC {A}, {A}")
		halfcarry_flag(A, A + A + nonzero_to_one(F & F_C))
		carry_flag(A + A + nonzero_to_one(F & F_C))
		A_set(A + A + nonzero_to_one(F & F_C))
		zero_flag(A)
		unset_flag(F_N)
		PC1()
		return

	#SUB B [1 4] [Z 1 H C]
	elif opcode == 0x90:
		print_asm(f"[90] SUB B => SUB {B:02X}")
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
		print_asm(f"[91] SUB C => SUB {C:02X}")
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
		print_asm(f"[92] SUB D => SUB {D:02X}")
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
		print_asm(f"[93] SUB E => SUB {E:02X}")
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
		print_asm(f"[94] SUB H => SUB {H:02X}")
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
		print_asm(f"[95] SUB L => SUB {L:02X}")
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
		print_asm(f"[96] SUB (HL) => SUB mem[{HL():04X}] => SUB {mem_read(HL()):02X}")
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
		print_asm(f"[97] SUB A => SUB A")
		halfcarry_flag(A, A - A, True)
		unset_flag(F_C)
		A_set(A - A)
		zero_flag(A)
		set_flag(F_N)
		PC1()
		return

	#SBC A, B [1 4] [Z 1 H C]
	elif opcode == 0x98:
		print_asm(f"[98] SBC A, B => SBC {A:02X}, {B:02X}")
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
		print_asm(f"[99] SBC A, C => SBC {A:02X}, {C:02X}")
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
		print_asm(f"[9A] SBC A, D => SBC {A:02X}, {D:02X}")
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
		print_asm(f"[9B] SBC A, E => SBC {A:02X}, {E:02X}")
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
		print_asm(f"[9C] SBC A, H => SBC {A:02X}, {H:02X}")
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
		print_asm(f"[9D] SBC A, L => SBC {A:02X}, {L:02X}")
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
		print_asm(f"[9E] SBC A, (HL) => SBC {A:02X}, mem[{HL():04X}] => SBC {A:02X}, {mem_read(HL()):02X}")
		print_asm(f"[9E] SBC A, (HL) => SBC A, mem[{HL():02X}] => SBC A, {mem_read(HL()):02X}]")
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
		print_asm(f"[9F] SBC A, A => SBC {A}, {A}")
		print_asm(f"[9F] SBC A, A")
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
		print_asm(f"[A0] AND B => AND {B:02X}")
		A_set(A & B)
		zero_flag(A)
		unset_flag(F_N)
		set_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#AND C [1 4] [Z 0 1 0]
	elif opcode == 0xA1:
		print_asm(f"[A1] AND C => AND {C:02X}")
		A_set(A & C)
		zero_flag(A)
		unset_flag(F_N)
		set_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#AND D [1 4] [Z 0 1 0]
	elif opcode == 0xA2:
		print_asm(f"[A2] AND D => AND {D:02X}")
		A_set(A & D)
		zero_flag(A)
		unset_flag(F_N)
		set_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#AND E [1 4] [Z 0 1 0]
	elif opcode == 0xA3:
		print_asm(f"[A3] AND E => AND {E:02X}")
		A_set(A & E)
		zero_flag(A)
		unset_flag(F_N)
		set_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#AND H [1 4] [Z 0 1 0]
	elif opcode == 0xA4:
		print_asm(f"[A4] AND H => AND {H:02X}")
		A_set(A & H)
		zero_flag(A)
		unset_flag(F_N)
		set_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#AND L [1 4] [Z 0 1 0]
	elif opcode == 0xA5:
		print_asm(f"[A5] AND L => AND {L:02X}")
		A_set(A & L)
		zero_flag(A)
		unset_flag(F_N)
		set_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#AND (HL) [1 8] [Z 0 1 0]
	elif opcode == 0xA6:
		print_asm(f"[A6] AND (HL) => AND mem[{HL():02X}] => AND {mem_read(HL()):02X}]")
		A_set(A & mem_read(HL()))
		zero_flag(A)
		unset_flag(F_N)
		set_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#AND A [1 8] [Z 0 1 0]
	elif opcode == 0xA7:
		print_asm(f"[A7] AND A => AND {A:02X}")
		zero_flag(A)
		unset_flag(F_N)
		set_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#XOR B [1 4] [Z 0 0 0]
	elif opcode == 0xA8:
		print_asm(f"[A8] XOR B => XOR {B:02X}")
		A_set(A ^ B)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#XOR C [1 4] [Z 0 0 0]
	elif opcode == 0xA9:
		print_asm(f"[A9] XOR C => XOR {C:02X}")
		A_set(A ^ C)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#XOR D [1 4] [Z 0 0 0]
	elif opcode == 0xAA:
		print_asm(f"[AA] XOR D => XOR {D:02X}")
		A_set(A ^ D)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#XOR E [1 4] [Z 0 0 0]
	elif opcode == 0xAB:
		print_asm(f"[AB] XOR E => XOR {E:02X}")
		A_set(A ^ E)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#XOR H [1 4] [Z 0 0 0]
	elif opcode == 0xAC:
		print_asm(f"[AC] XOR H => XOR {H:02X}")
		A_set(A ^ H)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#XOR L [1 4] [Z 0 0 0]
	elif opcode == 0xAD:
		print_asm(f"[AD] XOR L => XOR {L:02X}")
		A_set(A ^ L)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#XOR (HL) [1 8] [Z 0 0 0]
	elif opcode == 0xAE:
		print_asm(f"[AE] XOR (HL) => XOR mem[{HL():04X}] => XOR {mem_read(HL()):02X}]")
		A_set(A ^ mem_read(HL()))
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#XOR A [1 8] [Z 0 0 0]
	elif opcode == 0xAF:
		print_asm(f"[AF] XOR A => XOR {A:02X}")
		A_set(A ^ A)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#OR B [1 4] [Z 0 0 0]
	elif opcode == 0xB0:
		print_asm(f"[B0] OR B => OR {B:02X}")
		A_set(A | B)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#OR C [1 4] [Z 0 0 0]
	elif opcode == 0xB1:
		print_asm(f"[B1] OR C => OR {C:02X}")
		A_set(A | C)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#OR D [1 4] [Z 0 0 0]
	elif opcode == 0xB2:
		print_asm(f"[B2] OR D => OR {D:02X}")
		A_set(A | D)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#OR E [1 4] [Z 0 0 0]
	elif opcode == 0xB3:
		print_asm(f"[B3] OR E => OR {E:02X}")
		A_set(A | E)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#OR H [1 4] [Z 0 0 0]
	elif opcode == 0xB4:
		print_asm(f"[B4] OR H => OR {H:02X}")
		A_set(A | H)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#OR L [1 4] [Z 0 0 0]
	elif opcode == 0xB5:
		print_asm(f"[B5] OR L => OR {L:02X}")
		A_set(A | L)
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#OR (HL) [1 8] [Z 0 0 0]
	elif opcode == 0xB6:
		print_asm(f"[B6] OR (HL) => OR mem[{HL():04X}] => OR {mem_read(HL()):02X}]")
		A_set(A | mem_read(HL()))
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#OR A [1 4] [Z 0 0 0]
	elif opcode == 0xB7:
		print_asm(f"[B7] OR A => OR {A:02X}")
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC1()
		return

	#CP B [1 4] [Z 1 H C]
	elif opcode == 0xB8:
		print_asm(f"[B8] CP B => CP {B:02X}")
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
		print_asm(f"[B9] CP C => CP {C:02X}")
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
		print_asm(f"[BA] CP D => CP {D:02X}")
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
		print_asm(f"[BB] CP E => CP {E:02X}")
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
		print_asm(f"[BC] CP H => CP {H:02X}")
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
		print(f"[BD] CP L => CP {L:02X}")
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
		print(f"[BE] CP (HL) => CP {mem_read(HL()):02X}")
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
		print_asm(f"[BF] CP A => CP {A:02X}")
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
		print_asm(f"[C0] RET NZ")
		if not flag_is_set(F_Z):
			ret()
			return
		PC1()
		return

	#POP BC [1 12] [- - - -]
	elif opcode == 0xC1:
		print_asm(f"[C1] POP BC")
		C_set(pop_stack())
		B_set(pop_stack())
		PC1()
		return

	#JP NZ, a16 [3 16/12] [- - - -]
	elif opcode == 0xC2:
		print_asm(f"[C2] JP NZ, a16 => JP NZ, {compose(mem_read(PC + 2), mem_read(PC + 1))}")
		if not flag_is_set(F_Z):
			PC_set(compose(mem_read(PC + 2), mem_read(PC + 1)))
			return
		else:
			PC3()
			return

	#PUSH BC [1 16] [- - - -]
	elif opcode == 0xC5:
		print_asm(f"[C5] PUSH BC => PUSH {B:02X}{C:02X}")
		push_stack(B)
		push_stack(C)
		PC1()
		return

	#ADD A, d8 [2 8] [Z 0 H C]
	elif opcode == 0xC6:
		print_asm(f"[C6] ADD A, d8 => ADD A, {mem_read(PC + 1):02X}")
		halfcarry_flag(A, A + mem_read(PC + 1))
		carry_flag(A + mem_read(PC + 1))
		A_set(A + mem_read(PC + 1))
		zero_flag(A)
		unset_flag(F_N)
		PC2()
		return

	#RST 00H [1 16] [- - - -]
	elif opcode == 0xC7:
		print_asm(f"[C7] RST 00H")
		rst(0x00)
		PC1()
		return

	#RET Z [1 20/8] [- - - -]
	elif opcode == 0xC8:
		print_asm(f"[C8] RET Z")
		if flag_is_set(F_Z):
			ret()
			return
		PC1()
		return

	#RET [1 16] [- - - -]
	elif opcode == 0xC9:
		print_asm(f"[C9] RET")
		ret()
		return

	#JP Z, a16 [3 16/12] [- - - -]
	elif opcode == 0xCA:
		print_asm(f"[CA] JP Z, a16 => JP Z, {compose(mem_read(PC + 2), mem_read(PC + 1))}")
		if flag_is_set(F_Z):
			PC_set(compose(mem_read(PC + 2), mem_read(PC + 1)))
			return
		else:
			PC3()
			return

	#PREFIX CB [1 4] [- - - -] ???
	elif opcode == 0xCB:
		print_asm(f"[CB] PREFIX CB")
		PC1()
		opcode = mem_read(PC)

		#RLC B [2 8] [Z 0 0 C] ???
		if opcode == 0x00:
			print_asm(f"[CB][00] RLC B")
			# TODO
			PC1()
			return

		#RL B [2 8] [Z 0 0 C]
		elif opcode == 0x10:
			print_asm(f"[CB][10] RL B")
			carry_flag(B & BIT_7)
			B_set(RL(B))
			zero_flag(B)
			unset_flag(F_N)
			unset_flag(F_H)
			PC1()
			return

		#RL C [2 8] [Z 0 0 C]
		elif opcode == 0x11:
			print_asm(f"[CB][11] RL C")
			carry_flag(C & BIT_7)
			C_set(RL(C))
			zero_flag(C)
			unset_flag(F_N)
			unset_flag(F_H)
			PC1()
			return

		#BIT 7, H [2 8] [Z 0 1 -] ???
		elif opcode == 0x7C:
			print_asm(f"[CB][7C] BIT 7, H")
			zero_flag(H & BIT_7)
			unset_flag(F_N)
			set_flag(F_H)
			PC1()
			return

		#SET 7, A [2 8] [- - - -]
		elif opcode == 0xFF:
			print_asm(f"[CB][FF] SET 7, A")
			A_set(A | BIT_7)
			PC1()
			return

		else:
			print_asm(f"[CB][{opcode:02X}] Unknown CB opcode")
			err_msg(f"Unknown opcode (CB {opcode:02X})")
			PC1()
			return

	#CALL d16 [3 24/12] [- - - -]
	elif opcode == 0xCD:
		print_asm(f"[CD] CALL d16 => CALL {compose(mem_read(PC + 2), mem_read(PC + 1))}")
		call(compose(mem_read(PC + 2), mem_read(PC + 1)))
		return

	#ADC A, d8 [2 8] [Z 0 H C]
	elif opcode == 0xCE:
		print_asm(f"[CE] ADC A, d8 => ADC A, {mem_read(PC + 1)}")
		halfcarry_flag(A, A + mem_read(PC + 1) + nonzero_to_one(F & F_C))
		carry_flag(A + mem_read(PC + 1) + nonzero_to_one(F & F_C))
		A_set(A + mem_read(PC + 1) + nonzero_to_one(F & F_C))
		zero_flag(A)
		unset_flag(F_N)
		PC2()
		return

	#RST 08H [1 16] [- - - -]
	elif opcode == 0xCF:
		print_asm(f"[CF] RST 08H")
		rst(0x08)
		PC1()
		return

	#RET NC [1 20/8] [- - - -]
	elif opcode == 0xD0:
		print_asm(f"[D0] RET NC")
		if not flag_is_set(F_C):
			ret()
			return
		PC1()
		return

	#POP DE [1 12] [- - - -]
	elif opcode == 0xD1:
		print_asm(f"[D1] POP DE")
		E_set(pop_stack())
		D_set(pop_stack())
		PC1()
		return

	#JP NC, a16 [3 16/12] [- - - -]
	elif opcode == 0xD2:
		print_asm(f"[D2] JP NC, a16 => JP NC, {compose(mem_read(PC + 2), mem_read(PC + 1))}")
		if not flag_is_set(F_C):
			PC_set(compose(mem_read(PC + 2), mem_read(PC + 1)))
			return
		else:
			PC3()
			return

	#Empty opcode
	elif opcode == 0xD3:
		print_asm(f"[D3] Empty")
		err_msg("Empty opcode (D3)")
		PC1()
		return

	#PUSH DE [1 16] [- - - -]
	elif opcode == 0xD5:
		print_asm(f"[D5] PUSH DE => PUSH {D:02X}{E:02X}")
		push_stack(D)
		push_stack(E)
		PC1()
		return

	#RST 10H [1 16] [- - - -]
	elif opcode == 0xD7:
		print_asm(f"[D7] RST 10H")
		rst(0x10)
		PC1()
		return

	#RET C [1 20/8] [- - - -]
	elif opcode == 0xD8:
		print_asm(f"[D8] RET C")
		if flag_is_set(F_C):
			ret()
			return
		PC1()
		return

	#RETI [1 16] [- - - -]
	elif opcode == 0xD9:
		print_asm(f"[D9] RETI")
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
		print_asm(f"[DB] Empty")
		err_msg("Empty opcode (DB)")
		PC1()
		return

	#Empty opcode
	elif opcode == 0xDD:
		print_asm(f"[DD] Empty")
		err_msg("Empty opcode (DD)")
		PC1()
		return

	#SBC A, d8 [2 8] [Z 1 H C]
	elif opcode == 0xDE:
		print_asm(f"[DE] SBC A, d8 => SBC A, {mem_read(PC + 1):02X}")
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
		print_asm(f"[DF] RST 18H")
		rst(0x18)
		PC1()
		return

	#LDH (a8), A [2 12] [- - - -]
	elif opcode == 0xE0:
		print_asm(f"[E0] LDH (a8), A => LDH mem[{0xFF00 + mem_read(PC + 1):04X}], {A:02X}")
		mem_set(0xFF00 + mem_read(PC + 1), A)
		PC2()
		return

	#POP HL [1 12] [- - - -]
	elif opcode == 0xE1:
		print_asm(f"[E1] POP HL")
		L_set(pop_stack())
		H_set(pop_stack())
		PC1()
		return

	#LDH (C), A [1 ?] [- - - -]
	elif opcode == 0xE2:
		print_asm(f"[E2] LDH (C), A => LDH mem[{0xFF00 + C:04X}], {A:02X}")
		mem_set(0xFF00 + C, A)
		PC1()
		return

	#Empty opcode
	elif opcode == 0xE3:
		err_msg("Empty opcode (E3)")
		print_asm(f"[E3] Empty")
		PC1()
		return

	#Empty opcode
	elif opcode == 0xE4:
		print_asm(f"[E4] Empty")
		err_msg("Empty opcode (E4)")
		PC1()
		return

	#PUSH HL [1 16] [- - - -]
	elif opcode == 0xE5:
		print_asm(f"[E5] PUSH HL => PUSH {H:02X}{L:02X}")
		push_stack(H)
		push_stack(L)
		PC1()
		return

	#AND d8 [2 8] [Z 0 1 0]
	elif opcode == 0xE6:
		print_asm(f"[E6] AND d8 => AND {mem_read(PC + 1):02X}")
		A_set(A & mem_read(PC + 1))
		zero_flag(A)
		unset_flag(F_N)
		set_flag(F_H)
		unset_flag(F_C)
		PC2()
		return

	#RST 20H [1 16] [- - - -]
	elif opcode == 0xE7:
		print_asm(f"[E7] RST 20H")
		rst(0x20)
		PC1()
		return

	#JP (HL) [1 4] [- - - -]
	elif opcode == 0xE9:
		print_asm(f"[E9] JP (HL) => JP {HL():04X}")
		PC_set(HL())
		return

	#LD (a16), A
	elif opcode == 0xEA:
		print_asm(f"[EA] LD (a16), A => LD mem[{compose(mem_read(PC + 2), mem_read(PC + 1)):04X}], {A:02X}")
		mem_set(compose(mem_read(PC + 2), mem_read(PC + 1)), A)
		PC3()
		return

	#Empty opcode
	elif opcode == 0xEB:
		print_asm(f"[EB] Empty")
		err_msg("Empty opcode (EB)")
		PC1()
		return

	#Empty opcode
	elif opcode == 0xEC:
		print_asm(f"[EC] Empty")
		err_msg("Empty opcode (EC)")
		PC1()
		return

	#Empty opcode
	elif opcode == 0xED:
		print_asm(f"[ED] Empty")
		err_msg("Empty opcode (ED)")
		PC1()
		return

	#XOR d8 [2 8] [Z 0 0 0]
	elif opcode == 0xEE:
		print_asm(f"[EE] XOR d8 => XOR {mem_read(PC + 1):02X}")
		A_set(A ^ mem_read(PC + 1))
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC2()
		return

	#RST 28H [1 16] [- - - -]
	elif opcode == 0xEF:
		print_asm(f"[EF] RST 28H")
		rst(0x28)
		PC1()
		return

	#LDH A, (a8) [2 12] [- - - -]
	elif opcode == 0xF0:
		print_asm(f"[F0] LDH A, (a8) => LDH A, mem[{0xFF00 + mem_read(PC + 1):04X}] => LDH A, {mem_read(0xFF00 + mem_read(PC + 1)):02X}")
		A_set(mem_read(0xFF00 + mem_read(PC + 1)))
		PC2()
		return

	#POP AF [1 12] [- - - -]
	elif opcode == 0xF1:
		print_asm(f"[F1] POP AF")
		F_set(pop_stack())
		A_set(pop_stack())
		PC1()
		return

	#LDH A, (C) [1 ?] [- - - -]
	elif opcode == 0xF2:
		print_asm(f"[F2] LDH A, (C) => LDH A, mem[{0xFF00 + C:04X}] => LDH, A {mem_read(0xFF00 + C):02X}")
		A_set(mem_read(0xFF00 + C))
		PC1()
		return

	#DI [1 4] [- - - -]
	elif opcode == 0xF3:
		print_asm(f"[F3] DI")
		disable_interrupts()
		PC1()
		return

	#Empty opcode
	elif opcode == 0xF4:
		print_asm(f"[F4] Empty")
		err_msg("Empty opcode (F4)")
		PC1()
		return

	#PUSH AF [1 16] [- - - -]
	elif opcode == 0xF5:
		print_asm(f"[F5] PUSH AF => PUSH {A:02X}{F:02X}")
		push_stack(A)
		push_stack(F)
		PC1()
		return

	#OR d8 [2 8] [Z 0 0 0]
	elif opcode == 0xF6:
		print_asm(f"[F6] OR d8 => OR {mem_read(PC + 1):02X}")
		A_set(A | mem_read(PC + 1))
		zero_flag(A)
		unset_flag(F_N)
		unset_flag(F_H)
		unset_flag(F_C)
		PC2()
		return

	#RST 30H [1 16] [- - - -]
	elif opcode == 0xF7:
		print_asm(f"[F7] RST 30H")
		rst(0x30)
		PC1()
		return

	#LD SP, HL [1 8] [- - - -]
	elif opcode == 0xF9:
		print_asm(f"[F9] LD SP, HL => LD SP, {HL():04X}")
		SP_set(HL())
		return

	#LD (a16), A
	elif opcode == 0xFA:
		print_asm(f"[FA] LD (a16), A => LD  mem[({compose(mem_read(PC + 2), mem_read(PC + 1)):04X})], {A:02X}")
		#mem_set(compose(mem_read(PC + 2), mem_read(PC + 1)), A)
		A_set(mem_read(compose(mem_read(PC + 2), mem_read(PC + 1))))
		PC3()
		return

	#EI [1 4] [- - - -]
	elif opcode == 0xFB:
		print_asm(f"[FB] EI")
		enable_interrupts()
		PC1()
		return

	#Empty opcode
	elif opcode == 0xFC:
		print_asm(f"[FC] Empty")
		err_msg("Empty opcode (FC)")
		PC1()
		return

	#Empty opcode
	elif opcode == 0xFD:
		print_asm(f"[FD] Empty")
		err_msg("Empty opcode (FD)")
		PC1()
		return

	#CP d8 [2 8] [Z 1 H C]
	elif opcode == 0xFE:
		print_asm(f"[FE] CP d8 => CP {mem_read(PC + 1):02X}")
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
		print_asm(f"[FF] RST 38H")
		rst(0x38)
		PC1()
		return

	else:
		err_msg("Invalid opcode (" + hex(opcode) + ")", True)

def main():
	global print_assembly
	#mem_init()
	#print(len(memory))
	#load_cart('blue.gb', True)
	#run_program("python-gbcpu/DMG_ROM.bin")
	load_bootstrap("python-gbcpu/DMG_ROM.bin", True)
	load_cart('python-gbcpu/blue.gb', True)
	PC_set(0x0000)
	running = True
	print_assembly = True
	while running:
		opcode = mem_read(PC)
		#print(f"PC: {PC:04X} | Opcode: {opcode:02X}")
		parse_opcode(opcode)
		#print_registers()
		print("----")

		if HL() == 0x7FFA:
			print("FINISHED BLOCK")
			running = False

		#if PC > 10:
		#	print("FINISHED ZEROING BLOCK")
		#	running = False
		#else:
		#	time.sleep(3)
		#input()v

if __name__ == '__main__':
	main()