/*
 * Name: RV32IASU Emulator in C
 * Author: Michael T. Kloos
 * 
 * Copyright: 
 * (C) Copyright 2022 Michael T. Kloos (http://www.michaelkloos.com/).
 * All Rights Reserved.
 */

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

typedef int32_t sint32_t;

#define OPCODE(value) (value & 0x7F)

#define R_rd(value) ((value >> 7) & 0x1F)
#define R_funct3(value) ((value >> 12) & 0x07)
#define R_rs1(value) ((value >> 15) & 0x1F)
#define R_rs2(value) ((value >> 20) & 0x1F)
#define R_funct7(value) ((value >> 25) & 0x7F)

#define I_rd(value) ((value >> 7) & 0x1F)
#define I_funct3(value) ((value >> 12) & 0x07)
#define I_rs1(value) ((value >> 15) & 0x1F)
#define I_imm(value) ((value >> 20) & 0xFFF)

#define S_funct3(value) ((value >> 12) & 0x07)
#define S_rs1(value) ((value >> 15) & 0x1F)
#define S_rs2(value) ((value >> 20) & 0x1F)
#define S_imm(value) \
	(((value & 0xFE000000) >> 20) | ((value & 0x00000F80) >> 7))

#define B_funct3(value) ((value >> 12) & 0x07)
#define B_rs1(value) ((value >> 15) & 0x1F)
#define B_rs2(value) ((value >> 20) & 0x1F)
#define B_imm(value)                                             \
	(((value & 0x80000000) >> 19) | ((value & 0x7E000000) >> 20) | \
	 ((value & 0x00000F00) >> 7) | ((value & 0x00000080) << 4))

#define U_rd(value) ((value >> 7) & 0x1F)
#define U_imm(value) ((value >> 0) & 0xFFFFF000)

#define J_rd(value) ((value >> 7) & 0x1F)
#define J_imm(value)                                             \
	(((value & 0x80000000) >> 11) | ((value & 0x7FE00000) >> 20) | \
	 ((value & 0x00100000) >> 9) | ((value & 0x000FF000) << 0))

#define RegVal (context->xr)

#define STDIN  0
#define STDOUT 1
#define STDERR 2

//#define DEBUG

// Exception Causes
#define INSTRUCTION_ADDRESS_MISALIGNED     0
#define INSTRUCTION_ACCESS_FAULT           1
#define ILLEGAL_INSTRUCTION                2
#define BREAKPOINT                         3
#define LOAD_ADDRESS_MISALIGNED            4
#define LOAD_ACCESS_FAULT                  5
#define STORE_AMO_ADDRESS_MISALIGNED       6
#define STORE_AMO_ACCESS_FAULT             7
#define ENVIRONMENT_CALL_FROM_U_MODE       8
#define ENVIRONMENT_CALL_FROM_S_MODE       9
// Reserved                               10
#define ENVIRONMENT_CALL_FROM_M_MODE      11
#define INSTRUCTION_PAGE_FAULT            12
#define LOAD_PAGE_FAULT                   13
// Reserved                               14
#define STORE_AMO_PAGE_FAULT              15
// --
#define CUSTOM_INTERNAL_WFI_SLEEP         24
#define CUSTOM_INTERNAL_EXECUTION_SUCCESS 25

// CSRs
// -- 0xF1? MRO
#define CSR_MVENDORID 0 // 0xF11
#define CSR_MARCHID 1   // 0xF12
#define CSR_MIMPID 2    // 0xF13
#define CSR_MHARTID 3   // 0xF14
// -- 0x30? MRW
#define CSR_MSTATUS 4   // 0x300
#define CSR_MISA 5      // 0x301
// Placeholder: MEDELEG // 0x302
#define CSR_MIDELEG 6   // 0x303
#define CSR_MIE 7       // 0x304
#define CSR_MTVEC 8     // 0x305
// -- 0x34? MRW
#define CSR_MSCRATCH 9  // 0x340
#define CSR_MEPC 10     // 0x341
#define CSR_MCAUSE 11   // 0x342
#define CSR_MTVAL 12    // 0x343
#define CSR_MIP 13      // 0x344
// -- 0x10? SRW
// Placeholder: SSTATUS // 0x100
// Placeholder: SIE     // 0x104
#define CSR_STVEC 14    // 0x105
// -- 0x14? SRW
#define CSR_SSCRATCH 15 // 0x140
#define CSR_SEPC 16     // 0x141
#define CSR_SCAUSE 17   // 0x142
#define CSR_STVAL 18    // 0x143
// Placeholder: SIP     // 0x144
// -- 0x18? SRW
#define CSR_SATP 19     // 0x180
// -- 0xC?? URO
// Placeholder: TIME    // 0xC01
// Placeholder: TIMEH   // 0xC81

struct retvals {
	uint32_t error;
	uint32_t value;
};

struct cpu_context {
	uint32_t xr[32];
	uint32_t pc;
	uint32_t mode;
	uint32_t csr[20];
};

// Physical Memory-Map Of Emulator:
// 0x1000_0000: UART
// 0x2000_0000: Memory-Mapped Data (Filesystem CPIO Image)
// 0x3000_0000: mtimecmp
// 0x3200_0000: CLINT
// 0x4000_0000: PLIC
// 0x8000_0000: RAM

void* mmdata; // 0x2000_0000
void* memory; // 0x8000_0000

uint32_t mmdata_length; // Should be in 4 byte increments

// PLIC Regs
uint32_t plic_source_priorities; //       Offset 0x0000_0000
uint32_t plic_pending_array;     //       Offset 0x0000_1000
uint32_t plic_h0_m_inter_en;     // Start Offset 0x0000_2000, Inc: 0x0000_0080
uint32_t plic_h0_s_inter_en;
uint32_t plic_h0_m_pri_thres;    // Start Offset 0x0020_0000, Inc: 0x0000_1000
uint32_t plic_h0_m_claim_compl;  // Start Offset 0x0020_0004, Inc: 0x0000_1000
uint32_t plic_h0_s_pri_thres;
uint32_t plic_h0_s_claim_compl;

// CLINT Regs
uint32_t clint_mtimecmp;
uint32_t clint_mtimecmph;

// UART0 Regs
uint32_t uart0_txdata;
uint32_t uart0_rxdata;
uint32_t uart0_txctrl;
uint32_t uart0_rxctrl;
uint32_t uart0_ie;
uint32_t uart0_ip;
uint32_t uart0_div;
// Internal
uint32_t uart0_rxcue;
uint32_t uart0_rxcuecount;

uint32_t ReadPhysMemory(uint32_t addr, unsigned int bitwidth, struct cpu_context *context) {
	if        (addr >= 0x80000000 && addr < 0x88000000) {
		addr -= 0x80000000;
		if        (bitwidth == 32) {
			uint32_t* ptr = memory + addr;
			return (uint32_t)(*ptr);
		} else if (bitwidth == 16) {
			uint16_t* ptr = memory + addr;
			return (uint32_t)(*ptr);
		} else {
			uint8_t*  ptr = memory + addr;
			return (uint32_t)(*ptr);
		}
	} else if (addr >= 0x40000000 && addr < 0x44000000) {
		addr -= 0x40000000;
		if (bitwidth == 32) {
			if        (addr == 0x000004) {
				return plic_source_priorities;
			} else if (addr == 0x001000) {
				return plic_pending_array;
			} else if (addr == 0x002000) {
				return plic_h0_m_inter_en;
			} else if (addr == 0x002080) {
				return plic_h0_s_inter_en;
			} else if (addr == 0x200000) {
				return plic_h0_m_pri_thres;
			} else if (addr == 0x200004) {
				return plic_h0_m_claim_compl;
			} else if (addr == 0x201000) {
				return plic_h0_s_pri_thres;
			} else if (addr == 0x201004) {
				return plic_h0_s_claim_compl;
			}
		}
	} else if (addr >= 0x32000000 && addr < 0x32010000) {
		addr -= 0x32000000;
		if (bitwidth == 32) {
			if        (addr == 0x00000) {
				return (context->csr[CSR_MIP] & 0x8) >> 3;
			} else if (addr == 0x04000) {
				return clint_mtimecmp;
			} else if (addr == 0x04004) {
				return clint_mtimecmph;
			}
		}
	} else if (addr >= 0x20000000 && addr < 0x30000000) {
		addr -= 0x20000000;
		if (addr < mmdata_length) {
			if        (bitwidth == 32) {
				uint32_t* ptr = mmdata + addr;
				return (uint32_t)(*ptr);
			} else if (bitwidth == 16) {
				uint16_t* ptr = mmdata + addr;
				return (uint32_t)(*ptr);
			} else {
				uint8_t*  ptr = mmdata + addr;
				return (uint32_t)(*ptr);
			}
		}
	} else if (addr >= 0x10000000 && addr < 0x10001000) {
		addr -= 0x10000000;
		if (bitwidth == 32) {
			if        (addr == 0x00) {
				// txdata
				return 0;
			} else if (addr == 0x04) {
				// rxdata
				return 0;
			} else if (addr == 0x08) {
				// txctrl
				return uart0_txctrl & 0x00070003;
			} else if (addr == 0x0C) {
				// rxctrl
				return uart0_rxctrl & 0x00070001;
			} else if (addr == 0x10) {
				// ie
				return uart0_ie     & 0x00000003;
			} else if (addr == 0x14) {
				// ip
				return uart0_ip     & 0x00000003;
			} else if (addr == 0x18) {
				// div
				return uart0_div    & 0x0000FFFF;
			}
		}
	}
	return 0;
}

void SavePhysMemory(uint32_t addr, unsigned int bitwidth, struct cpu_context *context, uint32_t value) {
	if        (addr >= 0x80000000 && addr < 0x88000000) {
		addr -= 0x80000000;
		if        (bitwidth == 32) {
			uint32_t* ptr = memory + addr;
			*ptr = (uint32_t)value;
		} else if (bitwidth == 16) {
			uint16_t* ptr = memory + addr;
			*ptr = (uint16_t)value;
		} else {
			uint8_t*  ptr = memory + addr;
			*ptr =  (uint8_t)value;
		}
	} else if (addr >= 0x40000000 && addr < 0x44000000) {
		addr -= 0x40000000;
		if (bitwidth == 32) {
			if        (addr == 0x000004) {
				plic_source_priorities = value & 0x7;
			} else if (addr == 0x001000) {
				// pending_array - Do Nothing
			} else if (addr == 0x002000) {
				plic_h0_m_inter_en = value & 0x2;
			} else if (addr == 0x002080) {
				plic_h0_s_inter_en = value & 0x2;
			} else if (addr == 0x200000) {
				plic_h0_m_pri_thres = value & 0x7;
			} else if (addr == 0x200004) {
				// h0_m_claim_compl
				if (value == 1) {
					if (plic_h0_m_inter_en & (1 << value)) {
						plic_pending_array &= ~(1 << value);
					}
				}
			} else if (addr == 0x201000) {
				plic_h0_s_pri_thres = value & 0x7;
			} else if (addr == 0x201004) {
				// h0_s_claim_compl
				if (value == 1) {
					if (plic_h0_s_inter_en & (1 << value)) {
						plic_pending_array &= ~(1 << value);
					}
				}
			}
		}
	} else if (addr >= 0x32000000 && addr < 0x32010000) {
		addr -= 0x32000000;
		if (bitwidth == 32) {
			if        (addr == 0x00000) {
				if (value & 0x1) {
					context->csr[CSR_MIP] |= 0x8;
				} else {
					context->csr[CSR_MIP] &= ~((uint32_t)0x8);
				}
			} else if (addr == 0x04000) {
				clint_mtimecmp = value;
			} else if (addr == 0x04004) {
				clint_mtimecmph = value;
			}
		}
	} else if (addr >= 0x10000000 && addr < 0x10001000) {
		addr -= 0x10000000;
		if (bitwidth == 32) {
			if        (addr == 0x00) {
				// txdata
				dprintf(STDOUT, "%c", value);
			} else if (addr == 0x04) {
				// rxdata
				// Do Nothing
			} else if (addr == 0x08) {
				// txctrl
				uart0_txctrl = value & 0x00070003;
			} else if (addr == 0x0C) {
				// rxctrl
				uart0_rxctrl = value & 0x00070001;
			} else if (addr == 0x10) {
				// ie
				uart0_ie = value     & 0x00000003;
			} else if (addr == 0x14) {
				// ip
				// Do Nothing
			} else if (addr == 0x18) {
				// div
				uart0_div = value    & 0x0000FFFF;
			}
		}
	}
	return;
}

struct retvals WalkPTs(uint32_t location, uint32_t csr_satp, uint32_t access_type, struct cpu_context *context) {
	uint32_t mem_addr;
	
	// Get the inital value of the satp CSR
	sint32_t page_walk = csr_satp;
	
	// Is Virtual Memory Active?
	if (page_walk < 0) {
		// Shift to match PTE entry offset to ready for entry to the PT Walk loop
		page_walk <<= 10;
		
		// Walk the PTs
		sint32_t shift_ammount = 10 + 12 - 2;
		do {
			// Correct Offset: Left Shift and then Right Arithmetic Shift for Sign Extension
			page_walk <<= 2;
			page_walk >>= 0;
			
			// Find the location of the next entry from the next page
			uint32_t entry_next_pt_location;
			entry_next_pt_location  =                  page_walk  & ~((uint32_t)0xFFF);
			entry_next_pt_location |= (location >> shift_ammount) &             0xFFC;
			
			// Update page_walk with the next
			page_walk = ReadPhysMemory(entry_next_pt_location, 32, context);
			
			shift_ammount -= 10;
		} while ((page_walk & 0xF) == 1 && shift_ammount >= (12 - 2));
		
		// Not a leaf PTE?
		if ((page_walk & 0xF) == 0x1) {
			struct retvals rtn;
			rtn.value = 0;
			rtn.error = 1; // Page Fault
			return rtn;
		}
		
		// if: pte.v == 0 || (pte.r == 0 && pte.w == 1)
		if ((page_walk & 0x1) == 0x0 || (page_walk & 0x6) == 0x4) {
			struct retvals rtn;
			rtn.value = 0;
			rtn.error = 1; // Page Fault
			return rtn;
		}
		
		// Access Allowed?
		if (context->csr[CSR_MSTATUS] & 0x00080000) {
			// mstatus.MXR == 1
			
			if (page_walk & 0x8) {
				// pte.x == 1
				
				page_walk |= 0x2; // Set pte.r = 1
			}
		}
		if (context->mode == 0) {
			// U-Mode
			
			// Does U-Mode not have access?
			if ((page_walk & 0x10) == 0x00) {
				// pte.u == 0
				
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = 1; // Page Fault
				return rtn;
			}
		} else {
			// S-Mode
			
			// Does S-Mode not have access?
			if ((page_walk & 0x10) != 0x00) {
				// pte.u == 1
				
				if (context->csr[CSR_MSTATUS] & 0x00040000) {
					// mstatus.SUM == 1
					
					page_walk &= ~((uint32_t)0x8); // Clear pte.x
				} else {
					// mstatus.SUM == 0
					
					struct retvals rtn;
					rtn.value = 0;
					rtn.error = 1; // Page Fault
					return rtn;
				}
			}
		}
		if        (access_type == 0) {
			// Read
			
			if ((page_walk & 0x2) == 0) {
				// pte.r == 0
				
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = 1; // Page Fault
				return rtn;
			}
		} else if (access_type == 1) {
			// Write
			
			if ((page_walk & 0x4) == 0) {
				// pte.w == 0
				
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = 1; // Page Fault
				return rtn;
			}
		} else {
			// Execute
			
			if ((page_walk & 0x8) == 0) {
				// pte.x == 0
				
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = 1; // Page Fault
				return rtn;
			}
		}
		
		mem_addr = page_walk << 2;
		mem_addr &= ~((uint32_t)0xFFF);
		
		// Is this a Superpage?
		// Append the additional lower bits if so.
		shift_ammount += 2;
		while (shift_ammount >= 12) {
			uint32_t mask = 0x3FF << shift_ammount;
			if (mem_addr & mask) {
				// Misaligned Superpage
				
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = 1; // Page Fault
				return rtn;
			}
			mem_addr |= location & mask;
			shift_ammount -= 10;
		}
		
		// Check Page Accessed and Page Dirty BitFlags
		if ((page_walk & 0x40) == 0) {
			// pte.a == 0
			
			struct retvals rtn;
			rtn.value = 0;
			rtn.error = 1; // Page Fault
			return rtn;
		}
		if ((page_walk & 0x80) == 0) {
			// pte.d == 0
			
			if (access_type == 1) {
				// Write
				
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = 1; // Page Fault
				return rtn;
			}
		}
		
		// Binary OR the last 12 bits of the address to the computed physical memory page.
		mem_addr |= location & 0xFFF;
	} else {
		mem_addr = location;
	}
	
	struct retvals rtn;
	rtn.value = mem_addr;
	rtn.error = 0;
	return rtn;
}

struct retvals ExecMemory(uint32_t addr, struct cpu_context *context) {
	if (addr & 0x3) {
		struct retvals rtn;
		rtn.error = INSTRUCTION_ADDRESS_MISALIGNED;
		rtn.value = 0;
		return rtn;
	}
	
	if (context->mode < 3) {
		struct retvals rtn;
		rtn = WalkPTs(addr, context->csr[CSR_SATP], 2, context);
		if (rtn.error) {
			rtn.error = INSTRUCTION_PAGE_FAULT;
			rtn.value = 0;
			return rtn;
		}
		addr = rtn.value;
	}
	
	uint32_t value = ReadPhysMemory(addr, 32, context);
	
	struct retvals rtn;
	rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
	rtn.value = value;
	return rtn;
}

struct retvals ReadMemory(uint32_t addr, unsigned int bitwidth, struct cpu_context *context) {
	if        (bitwidth == 32) {
		if (addr & 0x3) {
			struct retvals rtn;
			rtn.error = LOAD_ADDRESS_MISALIGNED;
			rtn.value = 0;
			return rtn;
		}
	} else if (bitwidth == 16) {
		if (addr & 0x1) {
			struct retvals rtn;
			rtn.error = LOAD_ADDRESS_MISALIGNED;
			rtn.value = 0;
			return rtn;
		}
	}
	
	if (context->mode < 3) {
		struct retvals rtn;
		rtn = WalkPTs(addr, context->csr[CSR_SATP], 0, context);
		if (rtn.error) {
			rtn.error = LOAD_PAGE_FAULT;
			rtn.value = 0;
			return rtn;
		}
		addr = rtn.value;
	}
	
	uint32_t value = ReadPhysMemory(addr, bitwidth, context);
	
	struct retvals rtn;
	rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
	rtn.value = value;
	return rtn;
}

struct retvals SaveMemory(uint32_t addr, unsigned int bitwidth, struct cpu_context *context, uint32_t value) {
	if        (bitwidth == 32) {
		if (addr & 0x3) {
			struct retvals rtn;
			rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
			rtn.value = 0;
			return rtn;
		}
	} else if (bitwidth == 16) {
		if (addr & 0x1) {
			struct retvals rtn;
			rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
			rtn.value = 0;
			return rtn;
		}
	}
	
	if (context->mode < 3) {
		struct retvals rtn;
		rtn = WalkPTs(addr, context->csr[CSR_SATP], 1, context);
		if (rtn.error) {
			rtn.error = STORE_AMO_PAGE_FAULT;
			rtn.value = 0;
			return rtn;
		}
		addr = rtn.value;
	}
	
	SavePhysMemory(addr, bitwidth, context, value);
	
	struct retvals rtn;
	rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
	rtn.value = 0;
	return rtn;
}

struct retvals CSR_Read(uint32_t csr_addr, struct cpu_context* context, uint32_t perm) {
	uint32_t csr_prefix = csr_addr & 0xF00;
	
	if        (csr_prefix == 0xF00) {
		if (context->mode < 3 || perm == 1) {
			struct retvals rtn;
			rtn.error = 1;
			rtn.value = 0;
			return rtn;
		}
	} else if (csr_prefix == 0x300) {
		if (context->mode < 3) {
			struct retvals rtn;
			rtn.error = 1;
			rtn.value = 0;
			return rtn;
		}
	} else if (csr_prefix == 0x100) {
		if (context->mode < 1) {
			struct retvals rtn;
			rtn.error = 1;
			rtn.value = 0;
			return rtn;
		}
	} else if (csr_prefix == 0xC00) {
		if (perm == 1) {
			struct retvals rtn;
			rtn.error = 1;
			rtn.value = 0;
			return rtn;
		}
	}
	
	// If we reach this point, we are allowed access and, 
	// if applicable, are allowed writes to the requested CSR.
	// However, the CSR may still not exist.  We will check that 
	// in the switch/if below.  The CSR not existing is the last 
	// way for this to fail in error.
	struct retvals rtn;
	rtn.error = 0;
	
	if        (csr_addr == 0xF11) {
		rtn.value = context->csr[CSR_MVENDORID];
	} else if (csr_addr == 0xF12) {
		rtn.value = context->csr[CSR_MARCHID];
	} else if (csr_addr == 0xF13) {
		rtn.value = context->csr[CSR_MIMPID];
	} else if (csr_addr == 0xF14) {
		rtn.value = context->csr[CSR_MHARTID];
	} else if (csr_addr == 0x300) {
		rtn.value = context->csr[CSR_MSTATUS];
	} else if (csr_addr == 0x301) {
		rtn.value = context->csr[CSR_MISA];
	} else if (csr_addr == 0x302) {
		rtn.value = 0; // MEDELEG
	} else if (csr_addr == 0x303) {
		rtn.value = context->csr[CSR_MIDELEG];
	} else if (csr_addr == 0x304) {
		rtn.value = context->csr[CSR_MIE];
	} else if (csr_addr == 0x305) {
		rtn.value = context->csr[CSR_MTVEC];
	} else if (csr_addr == 0x340) {
		rtn.value = context->csr[CSR_MSCRATCH];
	} else if (csr_addr == 0x341) {
		rtn.value = context->csr[CSR_MEPC];
	} else if (csr_addr == 0x342) {
		rtn.value = context->csr[CSR_MCAUSE];
	} else if (csr_addr == 0x343) {
		rtn.value = context->csr[CSR_MTVAL];
	} else if (csr_addr == 0x344) {
		rtn.value = context->csr[CSR_MIP];
	} else if (csr_addr == 0x100) {
		rtn.value = context->csr[CSR_MSTATUS] & 0x000C0122; // SSTATUS
	} else if (csr_addr == 0x104) {
		rtn.value = context->csr[CSR_MIE] & context->csr[CSR_MIDELEG]; // SIE
	} else if (csr_addr == 0x105) {
		rtn.value = context->csr[CSR_STVEC];
	} else if (csr_addr == 0x140) {
		rtn.value = context->csr[CSR_SSCRATCH];
	} else if (csr_addr == 0x141) {
		rtn.value = context->csr[CSR_SEPC];
	} else if (csr_addr == 0x142) {
		rtn.value = context->csr[CSR_SCAUSE];
	} else if (csr_addr == 0x143) {
		rtn.value = context->csr[CSR_STVAL];
	} else if (csr_addr == 0x144) {
		rtn.value = context->csr[CSR_MIP] & context->csr[CSR_MIDELEG]; // SIP
	} else if (csr_addr == 0x180) {
		rtn.value = context->csr[CSR_SATP];
	} else if (csr_addr == 0xC01) {
		rtn.value = 0; // TODO - TIME
	} else if (csr_addr == 0xC02) {
		rtn.value = 0; // TODO - TIMEH
	} else {
		// CSR Does Not Exist
		rtn.error = 1;
		rtn.value = 0;
	}
	
	return rtn;
}

void CSR_Write(uint32_t csr_addr, struct cpu_context* context, uint32_t value) {
	// This function doesn't check for errors.  It will always 
	// try to preform the requested action and may result in 
	// unexpected and incorrect behaviour if an error should 
	// have occurred.  Callers to this function should have 
	// previously called CSR_Read([csr_addr], [context], 1); 
	// to check for error conditions before calling this function.
	// Do not call this function if the call to CSR_Read() returned 
	// with an error.
	
	if        (csr_addr == 0x300) {
		context->csr[CSR_MSTATUS] = value & 0x000C19AA;
	} else if (csr_addr == 0x303) {
		context->csr[CSR_MIDELEG] = value & 0x00000222;
	} else if (csr_addr == 0x304) {
		context->csr[CSR_MIE] = value & 0x00000AAA;
	} else if (csr_addr == 0x305) {
		context->csr[CSR_MTVEC] = value;
	} else if (csr_addr == 0x340) {
		context->csr[CSR_MSCRATCH] = value;
	} else if (csr_addr == 0x341) {
		context->csr[CSR_MEPC] = value & 0xFFFFFFFC;
	} else if (csr_addr == 0x342) {
		context->csr[CSR_MCAUSE] = value;
	} else if (csr_addr == 0x343) {
		context->csr[CSR_MTVAL] = value;
	} else if (csr_addr == 0x344) {
		context->csr[CSR_MIP] = value & 0x00000222;
	} else if (csr_addr == 0x100) {
		context->csr[CSR_MSTATUS] = (context->csr[CSR_MSTATUS] & ~(0x000C0122)) | (value & 0x000C0122); // SSTATUS
	} else if (csr_addr == 0x104) {
		context->csr[CSR_MIE] = (context->csr[CSR_MIE] & ~(context->csr[CSR_MIDELEG])) | (value & context->csr[CSR_MIDELEG]); // SIE
	} else if (csr_addr == 0x105) {
		context->csr[CSR_STVEC] = value;
	} else if (csr_addr == 0x140) {
		context->csr[CSR_SSCRATCH] = value;
	} else if (csr_addr == 0x141) {
		context->csr[CSR_SEPC] = value & 0xFFFFFFFC;
	} else if (csr_addr == 0x142) {
		context->csr[CSR_SCAUSE] = value;
	} else if (csr_addr == 0x143) {
		context->csr[CSR_STVAL] = value;
	} else if (csr_addr == 0x144) {
		context->csr[CSR_MIP] = (context->csr[CSR_MIP] & ~(context->csr[CSR_MIDELEG] & 0x00000002)) | (value & context->csr[CSR_MIDELEG] & 0x00000002); // SIP
	} else if (csr_addr == 0x180) {
		context->csr[CSR_SATP] = value;
	}
	
	return;
}

uint32_t ExecuteInstruction(uint32_t inst, struct cpu_context* context) {
	// Execute Instruction
	
#ifdef DEBUG
	dprintf(STDERR, "PC: 0x%08X, DATA: 0x%08X, OPCODE: 0x%08X\n", context.pc, inst, OPCODE(inst));
#endif
	
	if (OPCODE(inst) == 0x37) {
		// Instruction: LUI
		// U-type
#ifdef DEBUG
		dprintf(STDERR, "\tLUI x%d, 0x%08X\n", U_rd(inst), U_imm(inst));
#endif
		if (U_rd(inst) != 0) {
			RegVal[U_rd(inst)] = U_imm(inst);
		}
	} else if (OPCODE(inst) == 0x17) {
		// Instruction: AUIPC
		// U-type
		
#ifdef DEBUG
		dprintf(STDERR, "\tAUIPC x%d, 0x%08X\n", U_rd(inst), U_imm(inst));
#endif
		
		if (U_rd(inst) != 0) {
			RegVal[U_rd(inst)] = U_imm(inst) + context->pc;
		}
		
	} else if (OPCODE(inst) == 0x6F) {
		// Instruction: JAL
		// J-type
		
#ifdef DEBUG
		dprintf(STDERR, "\tJAL x%d, 0x%08X\n", J_rd(inst), J_imm(inst));
#endif
		
		uint32_t offset = J_imm(inst);
		// Sign Extend
		if (offset & 0x00100000) {
			offset |= 0xFFE00000;
		}
		offset += context->pc;
		if (offset & 0x3) {
			// Instruction address misaligned
			return INSTRUCTION_ADDRESS_MISALIGNED;
		}
		if (J_rd(inst) != 0) {
			RegVal[J_rd(inst)] = context->pc + 4;
		}
		context->pc = offset;
		return CUSTOM_INTERNAL_EXECUTION_SUCCESS;
		
	} else if (OPCODE(inst) == 0x67) {
		// Possible: JALR
		// I-type
		
		if (I_funct3(inst) == 0) {
			// Instruction: JALR
			
#ifdef DEBUG
			dprintf(STDERR, "\tJALR x%d, 0x%03X(x%d)\n", I_rd(inst), I_imm(inst), I_rs1(inst));
#endif
			
			uint32_t offset = I_imm(inst);
			// Sign Extend
			if (offset & 0x00000800) {
				offset |= 0xFFFFF000;
			}
			offset += RegVal[I_rs1(inst)];
			offset &= 0xFFFFFFFE;
			if (offset & 0x3) {
				// Misaligned Exception
				return INSTRUCTION_ADDRESS_MISALIGNED;
			}
			if (I_rd(inst) != 0) {
				RegVal[I_rd(inst)] = context->pc + 4;
			}
			context->pc = offset;
			return CUSTOM_INTERNAL_EXECUTION_SUCCESS;
		} else {
			// Invalid Op-code
			return ILLEGAL_INSTRUCTION;
		}
		
	} else if (OPCODE(inst) == 0x63) {
		// Possible: BEQ, BNE, BLT, BLTU, BGE, BGEU
		// B-type
		
		uint32_t offset = B_imm(inst);
		// Sign Extend
		if (offset & 0x00001000) {
			offset |= 0xFFFFE000;
		}
		
		offset += context->pc;
		unsigned int take_branch = 0;
		
		if (B_funct3(inst) == 0x0) {
			// Instruction: BEQ
			
#ifdef DEBUG
			dprintf(STDERR, "\tBEQ x%d, x%d, 0x%08X\n", B_rs1(inst), B_rs2(inst), B_imm(inst));
#endif
			
			uint32_t rs1b = RegVal[B_rs1(inst)];
			uint32_t rs2b = RegVal[B_rs1(inst)];
			if (rs1b == rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x1) {
			// Instruction: BNE
			
#ifdef DEBUG
			dprintf(STDERR, "\tBNE x%d, x%d, 0x%08X\n", B_rs1(inst), B_rs2(inst), B_imm(inst));
#endif
			
			uint32_t rs1b = RegVal[B_rs1(inst)];
			uint32_t rs2b = RegVal[B_rs1(inst)];
			if (rs1b != rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x4) {
			// Instruction: BLT
			
#ifdef DEBUG
			dprintf(STDERR, "\tBLT x%d, x%d, 0x%08X\n", B_rs1(inst), B_rs2(inst), B_imm(inst));
#endif
			
			sint32_t rs1b = RegVal[B_rs1(inst)];
			sint32_t rs2b = RegVal[B_rs1(inst)];
			if (rs1b < rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x5) {
			// Instruction: BGE
			
#ifdef DEBUG
			dprintf(STDERR, "\tBGE x%d, x%d, 0x%08X\n", B_rs1(inst), B_rs2(inst), B_imm(inst));
#endif
			
			sint32_t rs1b = RegVal[B_rs1(inst)];
			sint32_t rs2b = RegVal[B_rs1(inst)];
			if (rs1b >= rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x6) {
			// Instruction: BLTU
			
#ifdef DEBUG
			dprintf(STDERR, "\tBLTU x%d, x%d, 0x%08X\n", B_rs1(inst), B_rs2(inst), B_imm(inst));
#endif
			
			uint32_t rs1b = RegVal[B_rs1(inst)];
			uint32_t rs2b = RegVal[B_rs1(inst)];
			if (rs1b < rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x7) {
			// Instruction: BGEU
			
#ifdef DEBUG
			dprintf(STDERR, "\tBGEU x%d, x%d, 0x%08X\n", B_rs1(inst), B_rs2(inst), B_imm(inst));
#endif
			
			uint32_t rs1b = RegVal[B_rs1(inst)];
			uint32_t rs2b = RegVal[B_rs1(inst)];
			if (rs1b >= rs2b) {
				take_branch = 1;
			}
			
		} else {
			// Invalid Op-code
			return ILLEGAL_INSTRUCTION;
		}
		
		if (take_branch) {
			if (offset & 0x3) {
				// Misaligned Exception
				return INSTRUCTION_ADDRESS_MISALIGNED;
			}
			context->pc = offset;
			return CUSTOM_INTERNAL_EXECUTION_SUCCESS;
		}
		
	} else if (OPCODE(inst) == 0x03) {
		// Possible: LW, LH, LHU, LB, LBU
		// I-type
		
		uint32_t offset = I_imm(inst);
		// Sign Extend
		if (offset & 0x00000800) {
			offset |= 0xFFFFF000;
		}
		
		offset += RegVal[I_rs1(inst)];
		uint32_t rd = 0;
		
		if ((I_funct3(inst) & 0x3) == 0x0) {
			// Instruction: LB
			
#ifdef DEBUG
			if (I_funct3(inst) & 0x4) {
				dprintf(STDERR, "\tLBU");
			} else {
				dprintf(STDERR, "\tLB");
			}
			dprintf(STDERR, " x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst), I_imm(inst));
#endif
			
			struct retvals rtn;
			rtn = ReadMemory(offset, 8, context);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				return rtn.error;
			}
			rd = rtn.value;
			
			// Sign Extend
			if ((I_funct3(inst) & 0x4) == 0 && (rd & 0x00000080)) {
				rd |= 0xFFFFFF00;
			}
			
		} else if ((I_funct3(inst) & 0x3) == 0x1) {
			// Instruction: LH
			
#ifdef DEBUG
			if (I_funct3(inst) & 0x4) {
				dprintf(STDERR, "\tLHU");
			} else {
				dprintf(STDERR, "\tLH");
			}
			dprintf(STDERR, " x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst), I_imm(inst));
#endif
			
			struct retvals rtn;
			rtn = ReadMemory(offset, 16, context);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				return rtn.error;
			}
			rd = rtn.value;
			
			// Sign Extend
			if ((I_funct3(inst) & 0x4) == 0 && (rd & 0x00008000)) {
				rd |= 0xFFFF0000;
			}
			
		} else if ((I_funct3(inst) & 0x3) == 0x2) {
			// Instruction: LW
			
#ifdef DEBUG
			dprintf(STDERR, "\tLW x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst), I_imm(inst));
#endif
			
			struct retvals rtn;
			rtn = ReadMemory(offset, 32, context);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				return rtn.error;
			}
			rd = rtn.value;
			
		} else {
			// Invalid Op-code
			return ILLEGAL_INSTRUCTION;
		}
		
		if (I_rd(inst) != 0) {
			RegVal[I_rd(inst)] = rd;
		}
		
	} else if (OPCODE(inst) == 0x23) {
		// Possible: SW, SH, SB
		// S-type
		
		uint32_t offset = S_imm(inst);
		// Sign Extend
		if (offset & 0x00000800) {
			offset |= 0xFFFFF000;
		}
		
		offset += RegVal[S_rs1(inst)];
		
		if (S_funct3(inst) == 0x0) {
			// Instruction: SB
			
#ifdef DEBUG
			dprintf(STDERR, "\tSB x%d, x%d, 0x%08X\n", S_rs1(inst), S_rs2(inst), S_imm(inst));
#endif
			
			struct retvals rtn;
			rtn = SaveMemory(offset, 8, context, RegVal[S_rs2(inst)] & 0xFF);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				return rtn.error;
			}
			
		} else if (S_funct3(inst) == 0x1) {
			// Instruction: SH
			
#ifdef DEBUG
			dprintf(STDERR, "\tSH x%d, x%d, 0x%08X\n", S_rs1(inst), S_rs2(inst), S_imm(inst));
#endif
			
			struct retvals rtn;
			rtn = SaveMemory(offset, 16, context, RegVal[S_rs2(inst)] & 0xFFFF);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				return rtn.error;
			}
			
		} else if (S_funct3(inst) == 0x2) {
			// Instruction: SW
			
#ifdef DEBUG
			dprintf(STDERR, "\tSW x%d, x%d, 0x%08X\n", S_rs1(inst), S_rs2(inst), S_imm(inst));
#endif
			
			struct retvals rtn;
			rtn = SaveMemory(offset, 32, context, RegVal[S_rs2(inst)]);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				return rtn.error;
			}
			
		} else {
			// Invalid Op-code
			return ILLEGAL_INSTRUCTION;
		}
		
	} else if (OPCODE(inst) == 0x13) {
		// Possible: ADDI, SLTI, SLTIU, ANDI, ORI, XORI, SLLI, SRLI, SRAI
		// I-type
		
		uint32_t imm = I_imm(inst);
		// Sign Extend
		if (imm & 0x00000800) {
			imm |= 0xFFFFF000;
		}
		
		uint32_t rd;
		
		if (I_funct3(inst) == 0x0) {
			// Instruction: ADDI
			
#ifdef DEBUG
			dprintf(STDERR, "\tADDI x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst), I_imm(inst));
#endif
			
			uint32_t rs1b = RegVal[I_rs1(inst)];
			uint32_t immb = imm;
			rd = rs1b + immb;
			
		} else if (I_funct3(inst) == 0x2) {
			// Instruction: SLTI
			
#ifdef DEBUG
			dprintf(STDERR, "\tSLTI x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst), I_imm(inst));
#endif
			
			sint32_t rs1b = RegVal[I_rs1(inst)];
			sint32_t immb = imm;
			if (immb > rs1b) {
				rd = 1;
			} else {
				rd = 0;
			}
			
		} else if (I_funct3(inst) == 0x3) {
			// Instruction: SLTIU
			
#ifdef DEBUG
			dprintf(STDERR, "\tSLTIU x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst), I_imm(inst));
#endif
			
			uint32_t rs1b = RegVal[I_rs1(inst)];
			uint32_t immb = imm;
			if (immb > rs1b) {
				rd = 1;
			} else {
				rd = 0;
			}
			
		} else if (I_funct3(inst) == 0x4) {
			// Instruction: XORI
			
#ifdef DEBUG
			dprintf(STDERR, "\tXORI x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst),
							I_imm(inst));
#endif
			
			uint32_t rs1b = RegVal[I_rs1(inst)];
			uint32_t immb = imm;
			rd = immb ^ rs1b;
			
		} else if (I_funct3(inst) == 0x6) {
			// Instruction: ORI
#ifdef DEBUG
			dprintf(STDERR, "\tORI x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst),
							I_imm(inst));
#endif
			
			uint32_t rs1b = RegVal[I_rs1(inst)];
			uint32_t immb = imm;
			rd = immb | rs1b;
			
		} else if (I_funct3(inst) == 0x7) {
			// Instruction: ANDI
#ifdef DEBUG
			dprintf(STDERR, "\tANDI x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst),
							I_imm(inst));
#endif
			
			uint32_t rs1b = RegVal[I_rs1(inst)];
			uint32_t immb = imm;
			rd = immb & rs1b;
			
		} else if (I_funct3(inst) == 0x1) {
			// Possible: SLLI
			
			if ((imm & 0xFE0) == 0x000) {
				// Instruction: SLLI
				
#ifdef DEBUG
				dprintf(STDERR, "\tSLLI x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst), I_imm(inst));
#endif
				
				uint32_t rs1b = RegVal[I_rs1(inst)];
				uint32_t immb = imm & 0x1F;
				rd = rs1b << immb;
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		} else if (I_funct3(inst) == 0x5) {
			// Possible: SRLI, SRAI
			
			if ((imm & 0xFE0) == 0x000) {
				// Instruction: SRLI
				
#ifdef DEBUG
				dprintf(STDERR, "\tSRLI x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst), I_imm(inst));
#endif
				
				uint32_t rs1b = RegVal[I_rs1(inst)];
				uint32_t immb = imm & 0x1F;
				rd = rs1b >> immb;
				
			} else if ((imm & 0xFE0) == 0x400) {
				// Instruction: SRAI
				
#ifdef DEBUG
				dprintf(STDERR, "\tSRAI x%d, x%d, 0x%08X\n", I_rd(inst), I_rs1(inst), I_imm(inst));
#endif
				
				sint32_t rs1b = RegVal[I_rs1(inst)];
				sint32_t immb = imm & 0x1F;
				rd = rs1b >> immb;
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
		}
		if (I_rd(inst) != 0) {
			RegVal[I_rd(inst)] = rd;
		}
		
	} else if (OPCODE(inst) == 0x33) {
		// Possible: ADD, SUB, SLL, SRL, SRA, SLT, SLTU, AND, OR, XOR
		// R-type
		
		uint32_t rd;
		
		if (R_funct3(inst) == 0x0) {
			// Possible: ADD, SUB
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: ADD
				
#ifdef DEBUG
				dprintf(STDERR, "\tADD x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst),
								R_rs2(inst));
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b + rs2b;
				
			} else if (R_funct7(inst) == 0x20) {
				// Instruction: SUB
				
#ifdef DEBUG
				dprintf(STDERR, "\tSUB x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst),
								R_rs2(inst));
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b - rs2b;
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		} else if (R_funct3(inst) == 0x1) {
			// Possible: SLL
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: SLL
				
#ifdef DEBUG
				dprintf(STDERR, "\tSLL x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst), R_rs2(inst));
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b << (rs2b & 0x1F);
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		} else if (R_funct3(inst) == 0x2) {
			// Possible: SLT
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: SLT
				
#ifdef DEBUG
				dprintf(STDERR, "\tSLT x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst), R_rs2(inst));
#endif
				
				sint32_t rs1b = RegVal[R_rs1(inst)];
				sint32_t rs2b = RegVal[R_rs2(inst)];
				if (rs1b < rs2b) {
					rd = 1;
				} else {
					rd = 0;
				}
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		} else if (R_funct3(inst) == 0x3) {
			// Possible: SLTU
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: SLTU
				
#ifdef DEBUG
				dprintf(STDERR, "\tSLTU x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst), R_rs2(inst));
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				if (rs1b < rs2b) {
					rd = 1;
				} else {
					rd = 0;
				}
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		} else if (R_funct3(inst) == 0x4) {
			// Possible: XOR
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: XOR
				
#ifdef DEBUG
				dprintf(STDERR, "\tXOR x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst), R_rs2(inst));
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b ^ rs2b;
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		} else if (R_funct3(inst) == 0x5) {
			// Possible: SRL, SRA
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: SRL
				
#ifdef DEBUG
				dprintf(STDERR, "\tSRL x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst), R_rs2(inst));
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b >> (rs2b & 0x1F);
				
			} else if (R_funct7(inst) == 0x20) {
				// Instruction: SRA
				
#ifdef DEBUG
				dprintf(STDERR, "\tSRA x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst), R_rs2(inst));
#endif
				
				sint32_t rs1b = RegVal[R_rs1(inst)];
				sint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b >> (rs2b & 0x1F);
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		} else if (R_funct3(inst) == 0x6) {
			// Possible: OR
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: OR
				
#ifdef DEBUG
				dprintf(STDERR, "\tOR x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst), R_rs2(inst));
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b | rs2b;
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		} else if (R_funct3(inst) == 0x7) {
			// Possible: AND
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: AND
				
#ifdef DEBUG
				dprintf(STDERR, "\tAND x%d, x%d, x%d\n", R_rd(inst), R_rs1(inst), R_rs2(inst));
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b & rs2b;
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		}
		if (R_rd(inst) != 0) {
			RegVal[R_rd(inst)] = rd;
		}
		
	} else if (OPCODE(inst) == 0x0F) {
		// Possible: FENCE
		// I-type
		
		if (I_funct3(inst) == 0x0) {
			// Instruction: FENCE
			
#ifdef DEBUG
			dprintf(STDERR, "\tFENCE\n");
#endif
			
			// Do nothing, implemented as a NO-OP
			
		} else {
			// Invalid Op-code
			return ILLEGAL_INSTRUCTION;
		}
		
	} else if (OPCODE(inst) == 0x2F) {
		// Possible: LR.W, SC.W
		// Possible: AMOSWAP.W, AMOADD.W, AMOXOR.W, AMOAND.W, AMOOR.W, AMOMIN.W, AMOMAX.W, AMOMINU.W, AMOMAXU.W
		// R-type
		
		if (R_funct3(inst) == 0x2) {
			// Possible: LR.W, SC.W
			// Possible: AMOSWAP.W, AMOADD.W, AMOXOR.W, AMOAND.W, AMOOR.W, AMOMIN.W, AMOMAX.W, AMOMINU.W, AMOMAXU.W
			
			uint32_t funct7_prefix = R_funct7(inst) & 0x7C;
			if        (funct7_prefix == 0x08) {
				// Instruction: LR.W
				
				if (R_rs2(inst) != 0) {
					// Invalid Op-code
					return ILLEGAL_INSTRUCTION;
				}
				
#ifdef DEBUG
				dprintf(STDERR, "\tLR.W\n");
#endif
				
				// TODO
				
			} else if (funct7_prefix == 0x0C) {
				// Instruction: SC.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tSC.W\n");
#endif
				
				// TODO
				
			} else if (funct7_prefix == 0x04) {
				// Instruction: AMOSWAP.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tAMOSWAP.W\n");
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						return STORE_AMO_ADDRESS_MISALIGNED;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						return STORE_AMO_PAGE_FAULT;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						return STORE_AMO_ACCESS_FAULT;
					}
					return rtn.error;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working  = RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					return rtn.error;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x00) {
				// Instruction: AMOADD.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tAMOADD.W\n");
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						return STORE_AMO_ADDRESS_MISALIGNED;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						return STORE_AMO_PAGE_FAULT;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						return STORE_AMO_ACCESS_FAULT;
					}
					return rtn.error;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working += RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					return rtn.error;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x10) {
				// Instruction: AMOXOR.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tAMOXOR.W\n");
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						return STORE_AMO_ADDRESS_MISALIGNED;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						return STORE_AMO_PAGE_FAULT;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						return STORE_AMO_ACCESS_FAULT;
					}
					return rtn.error;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working ^= RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					return rtn.error;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x30) {
				// Instruction: AMOAND.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tAMOAND.W\n");
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						return STORE_AMO_ADDRESS_MISALIGNED;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						return STORE_AMO_PAGE_FAULT;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						return STORE_AMO_ACCESS_FAULT;
					}
					return rtn.error;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working &= RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					return rtn.error;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x20) {
				// Instruction: AMOOR.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tAMOOR.W\n");
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						return STORE_AMO_ADDRESS_MISALIGNED;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						return STORE_AMO_PAGE_FAULT;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						return STORE_AMO_ACCESS_FAULT;
					}
					return rtn.error;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working |= RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					return rtn.error;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x40) {
				// Instruction: AMOMIN.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tAMOMIN.W\n");
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						return STORE_AMO_ADDRESS_MISALIGNED;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						return STORE_AMO_PAGE_FAULT;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						return STORE_AMO_ACCESS_FAULT;
					}
					return rtn.error;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				sint32_t v1 = working;
				sint32_t v2 = RegVal[R_rs2(inst)];
				if (v1 < v2) {
					working = v1;
				} else {
					working = v2;
				}
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					return rtn.error;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x50) {
				// Instruction: AMOMAX.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tAMOMAX.W\n");
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						return STORE_AMO_ADDRESS_MISALIGNED;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						return STORE_AMO_PAGE_FAULT;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						return STORE_AMO_ACCESS_FAULT;
					}
					return rtn.error;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				sint32_t v1 = working;
				sint32_t v2 = RegVal[R_rs2(inst)];
				if (v1 > v2) {
					working = v1;
				} else {
					working = v2;
				}
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					return rtn.error;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x60) {
				// Instruction: AMOMINU.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tAMOMINU.W\n");
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						return STORE_AMO_ADDRESS_MISALIGNED;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						return STORE_AMO_PAGE_FAULT;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						return STORE_AMO_ACCESS_FAULT;
					}
					return rtn.error;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				uint32_t v1 = working;
				uint32_t v2 = RegVal[R_rs2(inst)];
				if (v1 < v2) {
					working = v1;
				} else {
					working = v2;
				}
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					return rtn.error;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x70) {
				// Instruction: AMOMAXU.W
				
#ifdef DEBUG
				dprintf(STDERR, "\tAMOMAXU.W\n");
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						return STORE_AMO_ADDRESS_MISALIGNED;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						return STORE_AMO_PAGE_FAULT;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						return STORE_AMO_ACCESS_FAULT;
					}
					return rtn.error;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				uint32_t v1 = working;
				uint32_t v2 = RegVal[R_rs2(inst)];
				if (v1 > v2) {
					working = v1;
				} else {
					working = v2;
				}
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					return rtn.error;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
		} else {
			// Invalid Op-code
			return ILLEGAL_INSTRUCTION;
		}
		
	} else if (OPCODE(inst) == 0x73) {
		// Possible: ECALL, EBREAK
		// Possible: CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI
		// Possible: MRET, SRET
		// Possible: WFI, SFENCE.VMA
		// Possible (But unsupported and illegal on this emulator): HFENCE.BVMA, HFENCE.GVMA, URET
		// I-type or R-type
		
		if (I_funct3(inst) == 0) {
			// Possible: ECALL, EBREAK
			// Possible: MRET, SRET
			// Possible: WFI, SFENCE.VMA
			// Possible (But unsupported and illegal on this emulator): HFENCE.BVMA, HFENCE.GVMA, URET
			
			if (I_rd(inst) != 0) {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
			
			if        (I_imm(inst) == 0) {
				// Instruction: ECALL
				
				if (I_rs1(inst) != 0) {
					// Invalid Op-code
					return ILLEGAL_INSTRUCTION;
				}
				
#ifdef DEBUG
				dprintf(STDERR, "\tECALL\n");
#endif
				
				if        (context->mode == 0) {
					// U-Mode
					return ENVIRONMENT_CALL_FROM_U_MODE;
				} else if (context->mode == 1) {
					// S-Mode
					return ENVIRONMENT_CALL_FROM_S_MODE;
				} else {// context.mode == 3
					// M-Mode
					return ENVIRONMENT_CALL_FROM_M_MODE;
				}
				
			} else if (I_imm(inst) == 1) {
				// Instruction: EBREAK
				
				if (I_rs1(inst) != 0) {
					// Invalid Op-code
					return ILLEGAL_INSTRUCTION;
				}
				
#ifdef DEBUG
				dprintf(STDERR, "\tEBREAK\n");
#endif
				
				return BREAKPOINT;
				
			} else if (I_imm(inst) == 0x102) {
				// Instruction: SRET
				
				if (I_rs1(inst) != 0) {
					// Invalid Op-code
					return ILLEGAL_INSTRUCTION;
				}
				
#ifdef DEBUG
				dprintf(STDERR, "\tSRET\n");
#endif
				
				// TODO
				
			} else if (I_imm(inst) == 0x302) {
				// Instruction: MRET
				
				if (I_rs1(inst) != 0) {
					// Invalid Op-code
					return ILLEGAL_INSTRUCTION;
				}
				
#ifdef DEBUG
				dprintf(STDERR, "\tMRET\n");
#endif
				
				// TODO
				
			} else if (I_imm(inst) == 0x105) {
				// Instruction: WFI
				
				if (I_rs1(inst) != 0) {
					// Invalid Op-code
					return ILLEGAL_INSTRUCTION;
				}
				
#ifdef DEBUG
				dprintf(STDERR, "\tWFI\n");
#endif
				
				context->pc += 4;
				return CUSTOM_INTERNAL_WFI_SLEEP;
				
			} else if (R_funct7(inst) == 0x09) {
				// Instruction: SFENCE.VMA
				
#ifdef DEBUG
				dprintf(STDERR, "\tSFENCE.VMA\n");
#endif
				
				// Do nothing, implemented as a NO-OP
				
			} else {
				// Invalid Op-code
				return ILLEGAL_INSTRUCTION;
			}
		} else if (I_funct3(inst) == 0x1) {
				// Instruction: CSRRW
				
				uint32_t rs1b = RegVal[I_rs1(inst)];
				struct retvals rtn = CSR_Read(I_imm(inst), context, 1);
				if (rtn.error) {
					// CSR Read/Write Failed
					return ILLEGAL_INSTRUCTION;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				CSR_Write(I_imm(inst), context, rs1b);
				
		} else if (I_funct3(inst) == 0x2) {
				// Instruction: CSRRS
				
				uint32_t rs1b = RegVal[I_rs1(inst)];
				uint32_t perm;
				if (rs1b != 0) {
					perm = 1;
				} else {
					perm = 0;
				}
				struct retvals rtn = CSR_Read(I_imm(inst), context, perm);
				if (rtn.error) {
					// CSR Read/Write Failed
					return ILLEGAL_INSTRUCTION;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				CSR_Write(I_imm(inst), context, rs1b | rtn.value);
				
		} else if (I_funct3(inst) == 0x3) {
				// Instruction: CSRRC
				
				uint32_t rs1b = RegVal[I_rs1(inst)];
				uint32_t perm;
				if (rs1b != 0) {
					perm = 1;
				} else {
					perm = 0;
				}
				struct retvals rtn = CSR_Read(I_imm(inst), context, perm);
				if (rtn.error) {
					// CSR Read/Write Failed
					return ILLEGAL_INSTRUCTION;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				CSR_Write(I_imm(inst), context, rs1b & ~(rtn.value));
				
		} else if (I_funct3(inst) == 0x5) {
				// Instruction: CSRRWI
				
				uint32_t rs1b = I_rs1(inst);
				struct retvals rtn = CSR_Read(I_imm(inst), context, 1);
				if (rtn.error) {
					// CSR Read/Write Failed
					return ILLEGAL_INSTRUCTION;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				CSR_Write(I_imm(inst), context, rs1b);
				
		} else if (I_funct3(inst) == 0x6) {
				// Instruction: CSRRSI
				
				uint32_t rs1b = I_rs1(inst);
				uint32_t perm;
				if (rs1b != 0) {
					perm = 1;
				} else {
					perm = 0;
				}
				struct retvals rtn = CSR_Read(I_imm(inst), context, perm);
				if (rtn.error) {
					// CSR Read/Write Failed
					return ILLEGAL_INSTRUCTION;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				CSR_Write(I_imm(inst), context, rs1b | rtn.value);
				
		} else if (I_funct3(inst) == 0x7) {
				// Instruction: CSRRCI
				
				uint32_t rs1b = I_rs1(inst);
				uint32_t perm;
				if (rs1b != 0) {
					perm = 1;
				} else {
					perm = 0;
				}
				struct retvals rtn = CSR_Read(I_imm(inst), context, perm);
				if (rtn.error) {
					// CSR Read/Write Failed
					return ILLEGAL_INSTRUCTION;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				CSR_Write(I_imm(inst), context, rs1b & ~(rtn.value));
				
		} else {
			// Invalid Op-code
			return ILLEGAL_INSTRUCTION;
		}
		
	} else {
		// Invalid Op-code
		return ILLEGAL_INSTRUCTION;
	}
	
	context->pc += 0x4;
	return CUSTOM_INTERNAL_EXECUTION_SUCCESS;
}

void RunLoop(struct cpu_context* context) {
	for (unsigned int i = 0; i < 200000; i++) {
	}
}

void InitEmu(struct cpu_context* context) {
	memset(context, 0, sizeof(struct cpu_context));
	context->pc = 0x80000000;
	context->mode = 3;
	context->csr[CSR_MISA] = 0x40000000 | (1 << 0) | (1 << 8) | (1 << 18) | (1 << 20);
	
	// PLIC Regs
	plic_source_priorities = 0; //       Offset 0x0000_0000
	plic_pending_array = 0;     //       Offset 0x0000_1000
	plic_h0_m_inter_en = 0;     // Start Offset 0x0000_2000, Inc: 0x0000_0080
	plic_h0_s_inter_en = 0;
	plic_h0_m_pri_thres = 0;    // Start Offset 0x0020_0000, Inc: 0x0000_1000
	plic_h0_m_claim_compl = 0;  // Start Offset 0x0020_0004, Inc: 0x0000_1000
	plic_h0_s_pri_thres = 0;
	plic_h0_s_claim_compl = 0;
	
	// CLINT Regs
	clint_mtimecmp = 0;
	clint_mtimecmph = 0;
	
	// UART Regs
	uart0_txdata = 0;
	uart0_rxdata = 0;
	uart0_txctrl = 0;
	uart0_rxctrl = 0;
	uart0_ie = 0;
	uart0_ip = 0;
	uart0_div = 0;
	// Internal
	uart0_rxcue = 0;
	uart0_rxcuecount = 0;
	
	return;
}

void RunEmulator(struct cpu_context* context) {
	InitEmu(context);
	while (1) {
		RunLoop(context);
	}
	return;
}

signed int main(unsigned int argc, char *argv[], char *envp[]) {
	if (argc <= 2) {
		return 1;
	}
	
	unsigned int fd;
	fd = open(argv[1], O_RDONLY);
	if (fd == -1) {
		return 2;
	}
	
	signed int ret_val;
	struct stat statbuf;
	ret_val = fstat(fd, &statbuf);
	if (ret_val == -1) {
		return 3;
	}
	if (!(statbuf.st_mode & S_IFREG)) {
		return 4;
	}
	if (statbuf.st_size == 0) {
		return 5;
	}
	memory = malloc(0x08000000);
	ret_val = read(fd, memory, 0x08000000);
	close(fd);
	if (ret_val != statbuf.st_size) {
		free(memory);
		return 6;
	}
	
	fd = open(argv[2], O_RDONLY);
	if (fd == -1) {
		return 7;
	}
	
	ret_val = fstat(fd, &statbuf);
	if (ret_val == -1) {
		return 8;
	}
	if (!(statbuf.st_mode & S_IFREG)) {
		return 9;
	}
	if (statbuf.st_size == 0) {
		return 10;
	}
	off_t fsize = statbuf.st_size;
	if (fsize & 0x3) {
		fsize &= ~((off_t)0x3);
		fsize += 0x4;
	}
	mmdata_length = fsize;
	mmdata = malloc(fsize);
	ret_val = read(fd, mmdata, statbuf.st_size);
	close(fd);
	if (ret_val != statbuf.st_size) {
		free(mmdata);
		free(memory);
		return 11;
	}
	
	struct cpu_context context;
	memset(&context, 0, sizeof(struct cpu_context));
	
	while (1) {
	}
	
	free(mmdata);
	free(memory);
	return 0;
}
