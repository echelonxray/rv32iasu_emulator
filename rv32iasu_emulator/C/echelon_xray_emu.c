/*
 * Name: RV32IASU Emulator Core Layer C-Half in C
 * Author: Michael T. Kloos
 *
 * Copyright:
 * (C) Copyright 2022 Michael T. Kloos (http://www.michaelkloos.com/).
 * All Rights Reserved.
 */

/*
 * Configuration Definitions:
 *   DEBUG
 *   WASM_BUILD
 */

#ifndef WASM_BUILD

#define _POSIX_C_SOURCE 200809L

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <poll.h>
#include <errno.h>
#include <semaphore.h>
#include <signal.h>

#endif

#ifdef WASM_BUILD

typedef unsigned long long uint64_t;
typedef   signed long long sint64_t;
typedef   signed long long  int64_t;

typedef unsigned int       uint32_t;
typedef   signed int       sint32_t;
typedef   signed int        int32_t;

typedef unsigned short     uint16_t;
typedef   signed short     sint16_t;
typedef   signed short      int16_t;

typedef unsigned char       uint8_t;
typedef   signed char       sint8_t;
typedef   signed char        int8_t;

typedef uint32_t  size_t;
typedef sint32_t ssize_t;

#define malloc cust_malloc
#define memcpy cust_memcpy
#define memset cust_memset

extern void __heap_base;

#else

typedef  int8_t  sint8_t;
typedef int16_t sint16_t;
typedef int32_t sint32_t;
typedef int64_t sint64_t;

#define thread_lock sem_wait
#define thread_unlock sem_post

#define EXTERN_INT_SIG (SIGRTMIN + 0)

#endif

// Physical Memory-Map Of Emulator:
// 0x1000_0000: UART
// 0x2000_0000: Memory-Mapped Data (Filesystem CPIO Image)
//// 0x3200_0000: CLINT
//// 0x4000_0000: PLIC
// 0x0200_0000: CLINT
// 0x0C00_0000: PLIC
// 0x8000_0000: RAM
#define ADDR_RAM_START  0x80000000
#define ADDR_RAM_LENGTH 0x08000000
#define ADDR_PLIC_START  0x0C000000
#define ADDR_PLIC_LENGTH 0x00210000
#define ADDR_CLINT_START  0x02000000
#define ADDR_CLINT_LENGTH 0x00010000
#define ADDR_UART_START  0x10000000
#define ADDR_UART_LENGTH 0x00001000
#define ADDR_DATAIMAGE_START  0x20000000
#define ADDR_DATAIMAGE_LENGTH 0x10000000

#define PLIC_SOURCENUM_UART 10

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

#define UART_RX_FIFO_SIZE 16

struct retvals {
	uint32_t error;
	uint32_t value;
};

struct cpu_context {
	uint32_t xr[32];
	uint32_t pc;
	uint32_t mode;
	uint32_t lr_reserve_set;
	uint32_t csr[20];
};

struct cpu_context cpu_cntxt;
volatile uint32_t running;
#ifdef WASM_BUILD
volatile uint32_t spinlk;
#else
sem_t spinlk;
#endif

void* mmdata; // 0x2000_0000
void* memory; // 0x8000_0000

uint32_t mmdata_length; // Should be in 4 byte increments

// PLIC Regs
uint32_t plic_source_priority; //       Offset 0x0000_0000
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
// Internal
uint32_t clint_time;
uint32_t clint_timeh;
uint64_t clint_start_time64;

// UART0 Regs
//uint32_t uart0_rhr;
//uint32_t uart0_thr;
uint32_t uart0_ier;
uint32_t uart0_isr;
uint32_t uart0_fcr;
uint32_t uart0_lcr;
uint32_t uart0_mcr;
uint32_t uart0_lsr;
uint32_t uart0_msr;
uint32_t uart0_spr;
uint32_t uart0_dll;
uint32_t uart0_dlm;
uint32_t uart0_psd;
// Internal
uint32_t uart0_rxcue[UART_RX_FIFO_SIZE];
uint32_t uart0_rxcuestoreindex;
uint32_t uart0_rxcueloadindex;
volatile uint32_t uart0_rxcuecount;

static inline void UpdateTimer(struct cpu_context* context); // TODO: Remove

#ifdef DEBUG
volatile signed int debug;
volatile signed int debug_trap;
volatile signed int debug_trapret;
volatile signed int debug_timeint;
volatile signed int debug_pagead;
volatile signed int debug_memaccess;

uint32_t debugP_addr = 0x00000000;
#endif

#ifdef WASM_BUILD
uint32_t currently_allocated;
#endif

#ifdef WASM_BUILD
void console_log(uint32_t);
void terminal_write_char(uint32_t);
void js_get_timestamp_us(uint64_t*);
#endif

void uart0_output_char(uint8_t charactor) {
#ifndef WASM_BUILD
	dprintf(STDOUT, "%c", charactor);
#else
	terminal_write_char(charactor);
#endif
	return;
}

#ifdef WASM_BUILD
static void* malloc(size_t size) {
	void* ret = &__heap_base;
	ret += currently_allocated;
	currently_allocated += size;
	return ret;
}
static void memcpy(void* dest, void* src, size_t n) {
	unsigned char* dptr = dest;
	unsigned char* sptr = src;
	while (n > 0) {
		*dptr = *sptr;
		dptr++;
		sptr++;
		n--;
	}
	return;
}
static void memset(void* s, int c, size_t n) {
	unsigned char* dptr = s;
	while (n > 0) {
		*dptr = (unsigned char)(c & 0xFF);
		dptr++;
		n--;
	}
	return;
}
void thread_lock(volatile uint32_t* spn_lk_ptr) {
	while (1) {
		uint32_t spn_lk = *spn_lk_ptr;
		if (spn_lk == 0) {
			spn_lk = __atomic_exchange_n(spn_lk_ptr, 1, __ATOMIC_SEQ_CST);
			if (spn_lk == 0) {
				return;
			}
		}
	}
}
void thread_unlock(volatile uint32_t* spn_lk_ptr) {
	*spn_lk_ptr = 0;
	return;
}
#else
#ifdef DEBUG
static void print_reg_state(struct cpu_context *context, uint32_t data) {
	dprintf(STDERR, "PC: 0x%08X, Data: 0x%08X\n\r", context->pc, data);
	dprintf(STDERR, "\tMode: %d\n\r", context->mode);
	for (int i = 0; i < 32; i += 2) {
		dprintf(STDERR, "\tx%02d: 0x%08X, x%02d: 0x%08X\n\r", i, context->xr[i], i + 1, context->xr[i + 1]);
	}
	return;
}
#endif
static void DestroyEmu() {
	free(memory);
	free(mmdata);
	return;
}
#endif

void InitEmu(uint32_t firmware_length, uint32_t disk_image_length) {
#ifdef WASM_BUILD
	currently_allocated = 0;
	spinlk = 0;
#else
	sem_init(&spinlk, 0, 1);
#endif
	
#ifdef DEBUG
	debug = 0;
	debug_trap = 0;
	debug_trapret = 0;
	debug_timeint = 1;
	debug_pagead = 0;
	debug_memaccess = 0;
#endif
	
	running = 0;
	
	memory = malloc(0x08000000);
	if (disk_image_length & 0x3) {
		disk_image_length +=  4;
		disk_image_length &= -4;
	}
	mmdata = malloc(disk_image_length);
	mmdata_length = disk_image_length;
	
	memset(&cpu_cntxt, 0, sizeof(struct cpu_context));
	cpu_cntxt.pc = 0x80000000;
	cpu_cntxt.mode = 3;
	cpu_cntxt.lr_reserve_set = (sint32_t)-1;
	cpu_cntxt.csr[CSR_MISA] = 0x40000000 | (1 << 0) | (1 << 8) | (1 << 18) | (1 << 20);
	
	// PLIC Regs
	plic_source_priority = 0;
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
	// Internal
	clint_time = 0;
	clint_timeh = 0;

#ifndef WASM_BUILD
	struct timespec tm_spc;
	clock_gettime(CLOCK_REALTIME, &tm_spc);
	clint_start_time64  = tm_spc.tv_sec * 1000000;
	clint_start_time64 += tm_spc.tv_nsec / 1000;
#else
	js_get_timestamp_us(&clint_start_time64);
#endif

	// UART0 Regs
	//uart0_rhr = 0;
	//uart0_thr = 0;
	uart0_ier = 0x00;
	uart0_isr = 0x01;
	uart0_fcr = 0x00;
	uart0_lcr = 0x00;
	uart0_mcr = 0x08;
	uart0_lsr = 0x60;
	uart0_msr = 0xB0;
	uart0_spr = 0x00;
	uart0_dll = 0x00;
	uart0_dlm = 0x00;
	uart0_psd = 0x00;
	// Internal
	memset(uart0_rxcue, 0, sizeof(uint32_t) * UART_RX_FIFO_SIZE);
	uart0_rxcuestoreindex = 0;
	uart0_rxcueloadindex = 0;
	uart0_rxcuecount = 0;
	
	return;
}

#ifdef WASM_BUILD
void* get_uart0_rxfifo_circbuf_loc() {
	return uart0_rxcue;
}
uint32_t get_uart0_rxfifo_circbuf_len() {
	return UART_RX_FIFO_SIZE;
}
void* get_uart0_rxfifo_circbuf_index_loc() {
	return &uart0_rxcuestoreindex;
}
void* get_uart0_rxfifo_circbuf_quecount_loc() {
	return (void*)&uart0_rxcuecount;
}
void* get_firmware_loc() {
	return memory;
}
void* get_disk_image_loc() {
	return mmdata;
}
void* get_cmp_time_hi_loc() {
	return &clint_mtimecmph;
}
void* get_cmp_time_lo_loc() {
	return &clint_mtimecmp;
}
void* get_start_time_hi_loc() {
	void* ptr = &clint_start_time64;
	return ptr + 4;
}
void* get_start_time_lo_loc() {
	void* ptr = &clint_start_time64;
	return ptr;
}
void* get_running_state_loc() {
	return (void*)&running;
}
void* get_lock_loc() {
	return (void*)&spinlk;
}
#endif

static inline uint32_t ReadPhysMemory(uint32_t addr, unsigned int bitwidth, struct cpu_context *context) {
	if        (addr >= ADDR_RAM_START && addr < ADDR_RAM_START + ADDR_RAM_LENGTH) {
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
	} else if (addr >= ADDR_PLIC_START && addr < ADDR_PLIC_START + ADDR_PLIC_LENGTH) {
		addr -= ADDR_PLIC_START;
		if (bitwidth == 32) {
			if        (addr == 0x000004 * PLIC_SOURCENUM_UART) {
				return plic_source_priority;
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
	} else if (addr >= ADDR_CLINT_START && addr < ADDR_CLINT_START + ADDR_CLINT_LENGTH) {
		addr -= ADDR_CLINT_START;
		if (bitwidth == 32) {
			if        (addr == 0x00000) {
				return (context->csr[CSR_MIP] & 0x8) >> 3;
			} else if (addr == 0x04000) {
				return clint_mtimecmp;
			} else if (addr == 0x04004) {
				return clint_mtimecmph;
			}
		}
	} else if (addr >= ADDR_DATAIMAGE_START && addr < ADDR_DATAIMAGE_START + ADDR_DATAIMAGE_LENGTH) {
		addr -= ADDR_DATAIMAGE_START;
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
	} else if (addr >= ADDR_UART_START && addr < ADDR_UART_START + ADDR_UART_LENGTH) {
		addr -= ADDR_UART_START;
		if (bitwidth == 8) {
			if        (addr == 0x00) {
				// rhr / dll
				if (uart0_lcr & 0x80) {
					return uart0_dll;
				} else {
					uint32_t cuecount = uart0_rxcuecount;
					if (cuecount > 0) {
						uint32_t value = uart0_rxcue[uart0_rxcueloadindex];
						uart0_rxcueloadindex = (uart0_rxcueloadindex + 1) % UART_RX_FIFO_SIZE;
						thread_lock(&spinlk);
						uart0_rxcuecount--;
						thread_unlock(&spinlk);
						return value & 0xFF;
					}
					return 0;
				}
			} else if (addr == 0x01) {
				// ier / dlm
				if (uart0_lcr & 0x80) {
					return uart0_dlm;
				} else {
					return uart0_ier;
				}
			} else if (addr == 0x02) {
				// isr
				return uart0_isr;
			} else if (addr == 0x03) {
				// lcr
				return uart0_lcr;
			} else if (addr == 0x04) {
				// mcr
				return uart0_mcr;
			} else if (addr == 0x05) {
				// lsr
				return uart0_lsr;
			} else if (addr == 0x06) {
				// msr
				return uart0_msr;
			} else if (addr == 0x07) {
				// spr
				return uart0_spr;
			}
		}
	}
	return 0;
}

static inline void SavePhysMemory(uint32_t addr, unsigned int bitwidth, struct cpu_context *context, uint32_t value) {
	if        (addr >= ADDR_RAM_START && addr < ADDR_RAM_START + ADDR_RAM_LENGTH) {
		addr -= ADDR_RAM_START;
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
	} else if (addr >= ADDR_PLIC_START && addr < ADDR_PLIC_START + ADDR_PLIC_LENGTH) {
		addr -= ADDR_PLIC_START;
		if (bitwidth == 32) {
			if        (addr == 0x000004 * PLIC_SOURCENUM_UART) {
				plic_source_priority = value & 0x7;
			} else if (addr == 0x001000) {
				// pending_array - Do Nothing
			} else if (addr == 0x002000) {
				plic_h0_m_inter_en = value & (1 << PLIC_SOURCENUM_UART);
			} else if (addr == 0x002080) {
				plic_h0_s_inter_en = value & (1 << PLIC_SOURCENUM_UART);
			} else if (addr == 0x200000) {
				plic_h0_m_pri_thres = value & 0x7;
			} else if (addr == 0x200004) {
				// h0_m_claim_compl
				if (value == PLIC_SOURCENUM_UART) {
					if (plic_h0_m_inter_en & (1 << value)) {
						plic_pending_array &= ~(1 << value);
					}
				}
			} else if (addr == 0x201000) {
				plic_h0_s_pri_thres = value & 0x7;
			} else if (addr == 0x201004) {
				// h0_s_claim_compl
				if (value == PLIC_SOURCENUM_UART) {
					if (plic_h0_s_inter_en & (1 << value)) {
						plic_pending_array &= ~(1 << value);
					}
				}
			}
		}
	} else if (addr >= ADDR_CLINT_START && addr < ADDR_CLINT_START + ADDR_CLINT_LENGTH) {
		addr -= ADDR_CLINT_START;
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
	} else if (addr >= ADDR_UART_START && addr < ADDR_UART_START + ADDR_UART_LENGTH) {
		addr -= ADDR_UART_START;
		
		if (bitwidth == 8) {
			if        (addr == 0x00) {
				// thr / dll
				if (uart0_lcr & 0x80) {
					// dll
					uart0_dll = value & 0xFF;
				} else {
					// thr - TX Data
					uart0_output_char(value & 0xFF);
				}
			} else if (addr == 0x01) {
				// ier / dlm
				if (uart0_lcr & 0x80) {
					// dlm
					uart0_dlm = value & 0xFF;
				} else {
					// ier
					uart0_ier = value & 0x0F;
				}
			} else if (addr == 0x02) {
				// fcr
				uart0_fcr = value & 0xC7;
			} else if (addr == 0x03) {
				// lcr
				uart0_lcr = value & 0xFF;
			} else if (addr == 0x04) {
				// mcr
				uart0_mcr = value & 0x1F;
			} else if (addr == 0x05) {
				// psd
				// TODO
				uart0_psd = value & 0x0F;
			} else if (addr == 0x07) {
				// spr
				uart0_spr = value & 0xFF;
			}
		}
	}
	return;
}

static inline struct retvals WalkPTs(uint32_t location, uint32_t csr_satp, uint32_t access_type, struct cpu_context *context) {
	uint32_t mem_addr;
	
	// Get the inital value of the satp CSR
	sint32_t page_walk = csr_satp;
	
	// Is Virtual Memory Active?
	if (page_walk < 0) {
		
		// Shift to match PTE entry offset to ready for entry to the PT Walk loop
		page_walk <<= 10;
		
		// Walk the PTs
		uint32_t entry_next_pt_location;
		sint32_t shift_ammount = 10 + 12 - 2;
		do {
			// Correct Offset: Left Shift and then Right Arithmetic Shift for Sign Extension
			page_walk <<= 2;
			page_walk >>= 0;
			
			// Find the location of the next entry from the next page
			entry_next_pt_location  =                  page_walk  & ~((uint32_t)0xFFF);
			entry_next_pt_location |= (location >> shift_ammount) &             0xFFC;
			
			// Update page_walk with the next
			page_walk = ReadPhysMemory(entry_next_pt_location, 32, context);
			
			shift_ammount -= 10;
		} while ((page_walk & 0xF) == 1 && shift_ammount >= (12 - 2));
		
		uint32_t page_walk2 = page_walk;
		
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
#ifdef DEBUG
			if (debug_pagead) {
				dprintf(STDERR, "Page Accessed\n\r");
			}
#endif
			
			page_walk2 |= 0x40;
			SavePhysMemory(entry_next_pt_location, 32, context, page_walk2);
			
			/*
			struct retvals rtn;
			rtn.value = 0;
			rtn.error = 1; // Page Fault
			return rtn;
			*/
		}
		if ((page_walk & 0x80) == 0) {
			// pte.d == 0
			
			if (access_type == 1) {
				// Write
				
#ifdef DEBUG
				if (debug_pagead) {
					dprintf(STDERR, "Page Dirty\n\r");
				}
#endif
				
				page_walk2 |= 0x80;
				SavePhysMemory(entry_next_pt_location, 32, context, page_walk2);
				
				/*
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = 1; // Page Fault
				return rtn;
				*/
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

static inline struct retvals ExecMemory(uint32_t addr, struct cpu_context *context) {
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

static inline struct retvals ReadMemory(uint32_t addr, unsigned int bitwidth, struct cpu_context *context) {
	
#ifdef DEBUG
	uint32_t addr2 = addr;
#endif
	
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
	
#ifdef DEBUG
	if (debug_memaccess) {
		if ((addr2 & -3) == debugP_addr) {
			dprintf(STDERR, "[Memory Access] PC: 0x%08X, Load Virt Addr: 0x%08X, Phys Addr: 0x%08X, Value: 0x%08X\n\r", context->pc, addr2, addr, value);
		}
	}
#endif
	
	struct retvals rtn;
	rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
	rtn.value = value;
	return rtn;
}

static inline struct retvals SaveMemory(uint32_t addr, unsigned int bitwidth, struct cpu_context *context, uint32_t value) {
	
#ifdef DEBUG
	uint32_t addr2 = addr;
#endif
	
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
	
	if (context->lr_reserve_set == (addr & ~((uint32_t)0x3))) {
		context->lr_reserve_set = (sint32_t)-1;
	}
	
#ifdef DEBUG
	if (debug_memaccess) {
		if ((addr2 & -3) == debugP_addr) {
			dprintf(STDERR, "[Memory Access] PC: 0x%08X, Save Virt Addr: 0x%08X, Phys Addr: 0x%08X, Value: 0x%08X\n\r", context->pc, addr2, addr, value);
		}
	}
#endif
	
	SavePhysMemory(addr, bitwidth, context, value);
	
	struct retvals rtn;
	rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
	rtn.value = 0;
	return rtn;
}

static inline struct retvals CSR_Read(uint32_t csr_addr, struct cpu_context* context, uint32_t perm) {
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
		rtn.value = context->csr[CSR_MIP] & context->csr[CSR_MIE];
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
		rtn.value = context->csr[CSR_MIP] & context->csr[CSR_MIE] & context->csr[CSR_MIDELEG]; // SIP
	} else if (csr_addr == 0x180) {
		rtn.value = context->csr[CSR_SATP];
	} else if (csr_addr == 0xC01) {
		UpdateTimer(context);
		rtn.value = clint_time;
	} else if (csr_addr == 0xC81) {
		UpdateTimer(context);
		rtn.value = clint_timeh;
	} else {
		// CSR Does Not Exist
		rtn.error = 1;
		rtn.value = 0;
	}
	
	return rtn;
}

static inline void CSR_Write(uint32_t csr_addr, struct cpu_context* context, uint32_t value) {
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

static inline struct retvals ExecuteInstruction(uint32_t inst, struct cpu_context* context) {
	// Execute Instruction
	
#ifdef DEBUG
	if (debug) {
		print_reg_state(context, inst);
		//dprintf(STDERR, "PC: 0x%08X, DATA: 0x%08X, OPCODE: 0x%08X\n\r", context->pc, inst, OPCODE(inst));
	}
#endif
	
	if (OPCODE(inst) == 0x37) {
		// Instruction: LUI
		// U-type
		
#ifdef DEBUG
		if (debug) {
			dprintf(STDERR, "\tLUI x%d, 0x%08X\n\r", U_rd(inst), U_imm(inst));
		}
#endif
		if (U_rd(inst) != 0) {
			RegVal[U_rd(inst)] = U_imm(inst);
		}
		
	} else if (OPCODE(inst) == 0x17) {
		// Instruction: AUIPC
		// U-type
		
#ifdef DEBUG
		if (debug) {
			dprintf(STDERR, "\tAUIPC x%d, 0x%08X\n\r", U_rd(inst), U_imm(inst));
		}
#endif
		
		if (U_rd(inst) != 0) {
			RegVal[U_rd(inst)] = U_imm(inst) + context->pc;
		}
		
	} else if (OPCODE(inst) == 0x6F) {
		// Instruction: JAL
		// J-type
		
#ifdef DEBUG
		if (debug) {
			dprintf(STDERR, "\tJAL x%d, 0x%08X\n\r", J_rd(inst), J_imm(inst));
		}
#endif
		
		uint32_t offset = J_imm(inst);
		// Sign Extend
		if (offset & 0x00100000) {
			offset |= 0xFFE00000;
		}
		offset += context->pc;
		if (offset & 0x3) {
			// Instruction address misaligned
			struct retvals rtn;
			rtn.value = offset;
			rtn.error = INSTRUCTION_ADDRESS_MISALIGNED;
			return rtn;
		}
		if (J_rd(inst) != 0) {
			RegVal[J_rd(inst)] = context->pc + 4;
		}
		context->pc = offset;
		struct retvals rtn;
		rtn.value = 0;
		rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
		return rtn;
		
	} else if (OPCODE(inst) == 0x67) {
		// Possible: JALR
		// I-type
		
		if (I_funct3(inst) == 0) {
			// Instruction: JALR
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tJALR x%d, 0x%03X(x%d)\n\r", I_rd(inst), I_imm(inst), I_rs1(inst));
			}
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
				struct retvals rtn;
				rtn.value = offset;
				rtn.error = INSTRUCTION_ADDRESS_MISALIGNED;
				return rtn;
			}
			if (I_rd(inst) != 0) {
				RegVal[I_rd(inst)] = context->pc + 4;
			}
			context->pc = offset;
			struct retvals rtn;
			rtn.value = 0;
			rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
			return rtn;
		} else {
			// Invalid Op-code
			struct retvals rtn;
			rtn.value = inst;
			rtn.error = ILLEGAL_INSTRUCTION;
			return rtn;
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
			if (debug) {
				dprintf(STDERR, "\tBEQ x%d, x%d, 0x%08X\n\r", B_rs1(inst), B_rs2(inst), B_imm(inst));
			}
#endif
			
			uint32_t rs1b = RegVal[B_rs1(inst)];
			uint32_t rs2b = RegVal[B_rs2(inst)];
			if (rs1b == rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x1) {
			// Instruction: BNE
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tBNE x%d, x%d, 0x%08X\n\r", B_rs1(inst), B_rs2(inst), B_imm(inst));
			}
#endif
			
			uint32_t rs1b = RegVal[B_rs1(inst)];
			uint32_t rs2b = RegVal[B_rs2(inst)];
			if (rs1b != rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x4) {
			// Instruction: BLT
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tBLT x%d, x%d, 0x%08X\n\r", B_rs1(inst), B_rs2(inst), B_imm(inst));
			}
#endif
			
			sint32_t rs1b = RegVal[B_rs1(inst)];
			sint32_t rs2b = RegVal[B_rs2(inst)];
			if (rs1b < rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x5) {
			// Instruction: BGE
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tBGE x%d, x%d, 0x%08X\n\r", B_rs1(inst), B_rs2(inst), B_imm(inst));
			}
#endif
			
			sint32_t rs1b = RegVal[B_rs1(inst)];
			sint32_t rs2b = RegVal[B_rs2(inst)];
			if (rs1b >= rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x6) {
			// Instruction: BLTU
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tBLTU x%d, x%d, 0x%08X\n\r", B_rs1(inst), B_rs2(inst), B_imm(inst));
			}
#endif
			
			uint32_t rs1b = RegVal[B_rs1(inst)];
			uint32_t rs2b = RegVal[B_rs2(inst)];
			if (rs1b < rs2b) {
				take_branch = 1;
			}
			
		} else if (B_funct3(inst) == 0x7) {
			// Instruction: BGEU
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tBGEU x%d, x%d, 0x%08X\n\r", B_rs1(inst), B_rs2(inst), B_imm(inst));
			}
#endif
			
			uint32_t rs1b = RegVal[B_rs1(inst)];
			uint32_t rs2b = RegVal[B_rs2(inst)];
			
			if (rs1b >= rs2b) {
				take_branch = 1;
			}
			
		} else {
			// Invalid Op-code
			struct retvals rtn;
			rtn.value = inst;
			rtn.error = ILLEGAL_INSTRUCTION;
			return rtn;
		}
		
		if (take_branch) {
			if (offset & 0x3) {
				// Misaligned Exception
				struct retvals rtn;
				rtn.value = offset;
				rtn.error = INSTRUCTION_ADDRESS_MISALIGNED;
				return rtn;
			}
			context->pc = offset;
			struct retvals rtn;
			rtn.value = 0;
			rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
			return rtn;
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
			if (debug) {
				if (I_funct3(inst) & 0x4) {
					dprintf(STDERR, "\tLBU");
				} else {
					dprintf(STDERR, "\tLB");
				}
				dprintf(STDERR, " x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
			}
#endif
			
			struct retvals rtn;
			rtn = ReadMemory(offset, 8, context);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				rtn.value = offset;
				return rtn;
			}
			rd = rtn.value;
			
			// Sign Extend
			if ((I_funct3(inst) & 0x4) == 0 && (rd & 0x00000080)) {
				rd |= 0xFFFFFF00;
			}
			
		} else if ((I_funct3(inst) & 0x3) == 0x1) {
			// Instruction: LH
			
#ifdef DEBUG
			if (debug) {
				if (I_funct3(inst) & 0x4) {
					dprintf(STDERR, "\tLHU");
				} else {
					dprintf(STDERR, "\tLH");
				}
				dprintf(STDERR, " x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
			}
#endif
			
			struct retvals rtn;
			rtn = ReadMemory(offset, 16, context);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				rtn.value = offset;
				return rtn;
			}
			rd = rtn.value;
			
			// Sign Extend
			if ((I_funct3(inst) & 0x4) == 0 && (rd & 0x00008000)) {
				rd |= 0xFFFF0000;
			}
			
		} else if (I_funct3(inst) == 0x2) {
			// Instruction: LW
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tLW x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
			}
#endif
			
			struct retvals rtn;
			rtn = ReadMemory(offset, 32, context);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				rtn.value = offset;
				return rtn;
			}
			rd = rtn.value;
			
		} else {
			// Invalid Op-code
			struct retvals rtn;
			rtn.value = inst;
			rtn.error = ILLEGAL_INSTRUCTION;
			return rtn;
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
			if (debug) {
				dprintf(STDERR, "\tSB x%d, x%d, 0x%08X\n\r", S_rs1(inst), S_rs2(inst), S_imm(inst));
			}
#endif
			
			struct retvals rtn;
			rtn = SaveMemory(offset, 8, context, RegVal[S_rs2(inst)] & 0xFF);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				rtn.value = offset;
				return rtn;
			}
			
		} else if (S_funct3(inst) == 0x1) {
			// Instruction: SH
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tSH x%d, x%d, 0x%08X\n\r", S_rs1(inst), S_rs2(inst), S_imm(inst));
			}
#endif
			
			struct retvals rtn;
			rtn = SaveMemory(offset, 16, context, RegVal[S_rs2(inst)] & 0xFFFF);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				rtn.value = offset;
				return rtn;
			}
			
		} else if (S_funct3(inst) == 0x2) {
			// Instruction: SW
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tSW x%d, x%d, 0x%08X\n\r", S_rs1(inst), S_rs2(inst), S_imm(inst));
			}
#endif
			
			struct retvals rtn;
			rtn = SaveMemory(offset, 32, context, RegVal[S_rs2(inst)]);
			if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
				rtn.value = offset;
				return rtn;
			}
			
		} else {
			// Invalid Op-code
			struct retvals rtn;
			rtn.value = inst;
			rtn.error = ILLEGAL_INSTRUCTION;
			return rtn;
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
			if (debug) {
				dprintf(STDERR, "\tADDI x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
			}
#endif
			
			uint32_t rs1b = RegVal[I_rs1(inst)];
			uint32_t immb = imm;
			rd = rs1b + immb;
			
		} else if (I_funct3(inst) == 0x2) {
			// Instruction: SLTI
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tSLTI x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
			}
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
			if (debug) {
				dprintf(STDERR, "\tSLTIU x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
			}
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
			if (debug) {
				dprintf(STDERR, "\tXORI x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
			}
#endif
			
			uint32_t rs1b = RegVal[I_rs1(inst)];
			uint32_t immb = imm;
			rd = immb ^ rs1b;
			
		} else if (I_funct3(inst) == 0x6) {
			// Instruction: ORI
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tORI x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
			}
#endif
			
			uint32_t rs1b = RegVal[I_rs1(inst)];
			uint32_t immb = imm;
			rd = immb | rs1b;
			
		} else if (I_funct3(inst) == 0x7) {
			// Instruction: ANDI
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tANDI x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
			}
#endif
			
			uint32_t rs1b = RegVal[I_rs1(inst)];
			uint32_t immb = imm;
			rd = immb & rs1b;
			
		} else if (I_funct3(inst) == 0x1) {
			// Possible: SLLI
			
			if ((imm & 0xFE0) == 0x000) {
				// Instruction: SLLI
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSLLI x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[I_rs1(inst)];
				uint32_t immb = imm & 0x1F;
				rd = rs1b << immb;
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		} else if (I_funct3(inst) == 0x5) {
			// Possible: SRLI, SRAI
			
			if ((imm & 0xFE0) == 0x000) {
				// Instruction: SRLI
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSRLI x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[I_rs1(inst)];
				uint32_t immb = imm & 0x1F;
				rd = rs1b >> immb;
				
			} else if ((imm & 0xFE0) == 0x400) {
				// Instruction: SRAI
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSRAI x%d, x%d, 0x%08X\n\r", I_rd(inst), I_rs1(inst), I_imm(inst));
				}
#endif
				
				sint32_t rs1b = RegVal[I_rs1(inst)];
				sint32_t immb = imm & 0x1F;
				rd = rs1b >> immb;
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
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
				if (debug) {
					dprintf(STDERR, "\tADD x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b + rs2b;
				
			} else if (R_funct7(inst) == 0x20) {
				// Instruction: SUB
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSUB x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b - rs2b;
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		} else if (R_funct3(inst) == 0x1) {
			// Possible: SLL
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: SLL
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSLL x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b << (rs2b & 0x1F);
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		} else if (R_funct3(inst) == 0x2) {
			// Possible: SLT
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: SLT
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSLT x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
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
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		} else if (R_funct3(inst) == 0x3) {
			// Possible: SLTU
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: SLTU
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSLTU x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
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
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		} else if (R_funct3(inst) == 0x4) {
			// Possible: XOR
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: XOR
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tXOR x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b ^ rs2b;
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		} else if (R_funct3(inst) == 0x5) {
			// Possible: SRL, SRA
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: SRL
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSRL x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b >> (rs2b & 0x1F);
				
			} else if (R_funct7(inst) == 0x20) {
				// Instruction: SRA
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSRA x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
#endif
				
				sint32_t rs1b = RegVal[R_rs1(inst)];
				sint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b >> (rs2b & 0x1F);
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		} else if (R_funct3(inst) == 0x6) {
			// Possible: OR
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: OR
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tOR x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b | rs2b;
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		} else if (R_funct3(inst) == 0x7) {
			// Possible: AND
			
			if (R_funct7(inst) == 0x00) {
				// Instruction: AND
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAND x%d, x%d, x%d\n\r", R_rd(inst), R_rs1(inst), R_rs2(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				rd = rs1b & rs2b;
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		}
		if (R_rd(inst) != 0) {
			RegVal[R_rd(inst)] = rd;
		}
		
	} else if (OPCODE(inst) == 0x0F) {
		// Possible: FENCE, FENCE.I
		// I-type
		
		if        (I_funct3(inst) == 0x0) {
			// Instruction: FENCE
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tFENCE\n\r");
			}
#endif
			
			// Do nothing, implemented as a NO-OP
			
		} else if (I_funct3(inst) == 0x1) {
			// Instruction: FENCE.I
			
#ifdef DEBUG
			if (debug) {
				dprintf(STDERR, "\tFENCE.I\n\r");
			}
#endif
			
			// Do nothing, implemented as a NO-OP
			
		} else {
			// Invalid Op-code
			struct retvals rtn;
			rtn.value = inst;
			rtn.error = ILLEGAL_INSTRUCTION;
			return rtn;
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
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				
				uint32_t addr = RegVal[R_rs1(inst)];
				
#ifdef DEBUG
				uint32_t addr2 = addr;
				if (debug) {
					dprintf(STDERR, "\tLR.W\n\r");
				}
#endif
				
				if (addr & 0x3) {
					struct retvals rtn;
					rtn.value = addr;
					rtn.error = LOAD_ADDRESS_MISALIGNED;
					return rtn;
				}
				
				if (context->mode < 3) {
					struct retvals rtn;
					rtn = WalkPTs(addr, context->csr[CSR_SATP], 0, context);
					if (rtn.error) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = LOAD_PAGE_FAULT;
						return rtn;
					}
					addr = rtn.value;
				}
				
				context->lr_reserve_set = addr;
				uint32_t rd;
				rd = ReadPhysMemory(addr, 32, context);
				
#ifdef DEBUG
				if (debug_memaccess) {
					if ((addr2 & -3) == debugP_addr) {
						dprintf(STDERR, "[Memory Access] PC: 0x%08X, Load Virt Addr: 0x%08X, Phys Addr: 0x%08X, Value: 0x%08X\n\r", context->pc, addr2, addr, rd);
					}
				}
#endif
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x0C) {
				// Instruction: SC.W
				
				uint32_t addr = RegVal[R_rs1(inst)];
				uint32_t rs2b = RegVal[R_rs2(inst)];
				
#ifdef DEBUG
				uint32_t addr2 = addr;
				if (debug) {
					dprintf(STDERR, "\tSC.W\n\r");
				}
#endif
				
				if (addr & 0x3) {
					struct retvals rtn;
					rtn.value = addr;
					rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
					return rtn;
				}
				
				if (context->mode < 3) {
					struct retvals rtn;
					rtn = WalkPTs(addr, context->csr[CSR_SATP], 1, context);
					if (rtn.error) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					addr = rtn.value;
				}
				
				uint32_t rd;
				if (context->lr_reserve_set == addr) {
					SavePhysMemory(addr, 32, context, rs2b);
					rd = 0;
				} else {
					rd = 1;
				}
				context->lr_reserve_set = (sint32_t)-1;
				
#ifdef DEBUG
				if (debug_memaccess) {
					if ((addr2 & -3) == debugP_addr) {
						dprintf(STDERR, "[Memory Access] PC: 0x%08X, Save Virt Addr: 0x%08X, Phys Addr: 0x%08X, Value: 0x%08X, Rd: %d\n\r", context->pc, addr2, addr, rs2b, rd);
					}
				}
#endif
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x04) {
				// Instruction: AMOSWAP.W
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAMOSWAP.W\n\r");
				}
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
						return rtn;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ACCESS_FAULT;
						return rtn;
					}
					rtn.value = 0;
					return rtn;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working  = RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					rtn.value = addr;
					return rtn;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x00) {
				// Instruction: AMOADD.W
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAMOADD.W\n\r");
				}
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
						return rtn;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ACCESS_FAULT;
						return rtn;
					}
					rtn.value = 0;
					return rtn;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working += RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					rtn.value = addr;
					return rtn;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x10) {
				// Instruction: AMOXOR.W
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAMOXOR.W\n\r");
				}
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
						return rtn;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ACCESS_FAULT;
						return rtn;
					}
					rtn.value = 0;
					return rtn;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working ^= RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					rtn.value = addr;
					return rtn;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x30) {
				// Instruction: AMOAND.W
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAMOAND.W\n\r");
				}
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
						return rtn;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ACCESS_FAULT;
						return rtn;
					}
					rtn.value = 0;
					return rtn;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working &= RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					rtn.value = addr;
					return rtn;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x20) {
				// Instruction: AMOOR.W
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAMOOR.W\n\r");
				}
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
						return rtn;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ACCESS_FAULT;
						return rtn;
					}
					rtn.value = 0;
					return rtn;
				}
				uint32_t rd;
				uint32_t working;
				rd = rtn.value;
				working = rd;
				
				working |= RegVal[R_rs2(inst)];
				
				rtn = SaveMemory(addr, 32, context, working);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					rtn.value = addr;
					return rtn;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x40) {
				// Instruction: AMOMIN.W
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAMOMIN.W\n\r");
				}
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
						return rtn;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ACCESS_FAULT;
						return rtn;
					}
					rtn.value = 0;
					return rtn;
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
					rtn.value = addr;
					return rtn;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x50) {
				// Instruction: AMOMAX.W
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAMOMAX.W\n\r");
				}
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
						return rtn;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ACCESS_FAULT;
						return rtn;
					}
					rtn.value = 0;
					return rtn;
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
					rtn.value = addr;
					return rtn;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x60) {
				// Instruction: AMOMINU.W
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAMOMINU.W\n\r");
				}
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
						return rtn;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ACCESS_FAULT;
						return rtn;
					}
					rtn.value = 0;
					return rtn;
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
					rtn.value = addr;
					return rtn;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else if (funct7_prefix == 0x70) {
				// Instruction: AMOMAXU.W
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tAMOMAXU.W\n\r");
				}
#endif
				
				uint32_t addr = RegVal[R_rs1(inst)];
				struct retvals rtn;
				rtn = ReadMemory(addr, 32, context);
				if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
					if (rtn.error == LOAD_ADDRESS_MISALIGNED) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ADDRESS_MISALIGNED;
						return rtn;
					}
					if (rtn.error == LOAD_PAGE_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_PAGE_FAULT;
						return rtn;
					}
					if (rtn.error == LOAD_ACCESS_FAULT) {
						struct retvals rtn;
						rtn.value = addr;
						rtn.error = STORE_AMO_ACCESS_FAULT;
						return rtn;
					}
					rtn.value = 0;
					return rtn;
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
					rtn.value = addr;
					return rtn;
				}
				
				if (R_rd(inst) != 0) {
					RegVal[R_rd(inst)] = rd;
				}
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
		} else {
			// Invalid Op-code
			struct retvals rtn;
			rtn.value = inst;
			rtn.error = ILLEGAL_INSTRUCTION;
			return rtn;
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
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
			
			if        (I_imm(inst) == 0) {
				// Instruction: ECALL
				
				if (I_rs1(inst) != 0) {
					// Invalid Op-code
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tECALL\n\r");
				}
#endif
				
				if        (context->mode == 0) {
					// U-Mode
					struct retvals rtn;
					rtn.value = 0;
					rtn.error = ENVIRONMENT_CALL_FROM_U_MODE;
					return rtn;
				} else if (context->mode == 1) {
					// S-Mode
					struct retvals rtn;
					rtn.value = 0;
					rtn.error = ENVIRONMENT_CALL_FROM_S_MODE;
					return rtn;
				} else {// context->mode == 3
					// M-Mode
					struct retvals rtn;
					rtn.value = 0;
					rtn.error = ENVIRONMENT_CALL_FROM_M_MODE;
					return rtn;
				}
				
			} else if (I_imm(inst) == 1) {
				// Instruction: EBREAK
				
				if (I_rs1(inst) != 0) {
					// Invalid Op-code
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tEBREAK\n\r");
				}
#endif
				
				/* TODO
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = BREAKPOINT;
				return rtn;
				*/
				
			} else if (I_imm(inst) == 0x102) {
				// Instruction: SRET
				
				if (I_rs1(inst) != 0 || context->mode < 0x1) {
					// Invalid Op-code
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				
#ifdef DEBUG
				if (debug || debug_trapret) {
					dprintf(STDERR, "\tSRET to: 0x%08X\n\r", context->csr[CSR_SEPC]);
				}
#endif
				
				uint32_t bits_pp  = (context->csr[CSR_MSTATUS] >> 8) & 0x1;
				uint32_t bits_pie = (context->csr[CSR_MSTATUS] >> 5) & 0x1;
				context->csr[CSR_MSTATUS] &= 0xFFFFFEDD;
				context->csr[CSR_MSTATUS] |= bits_pie << 1;
				context->csr[CSR_MSTATUS] |= 1 << 5;
				context->mode = bits_pp;
				context->pc = context->csr[CSR_SEPC];
				context->lr_reserve_set = (sint32_t)-1;
				
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
				return rtn;
				
			} else if (I_imm(inst) == 0x302) {
				// Instruction: MRET
				
				if (I_rs1(inst) != 0 || context->mode < 0x3) {
					// Invalid Op-code
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				
				uint32_t bits_pp  = (context->csr[CSR_MSTATUS] >> 11) & 0x3;
				uint32_t bits_pie = (context->csr[CSR_MSTATUS] >>  7) & 0x1;
				if (bits_pp == 2) {
					// Invalid Execution Mode
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				
#ifdef DEBUG
				if (debug || debug_trapret) {
					dprintf(STDERR, "\tMRET to: 0x%08X with mode: %d\n\r", context->csr[CSR_MEPC], bits_pp);
				}
#endif
				
				context->csr[CSR_MSTATUS] &= 0xFFFFE777;
				context->csr[CSR_MSTATUS] |= bits_pie << 3;
				context->csr[CSR_MSTATUS] |= 1 << 7;
				context->mode = bits_pp;
				context->pc = context->csr[CSR_MEPC];
				context->lr_reserve_set = (sint32_t)-1;
				
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
				return rtn;
				
			} else if (I_imm(inst) == 0x105) {
				// Instruction: WFI
				
				if (I_rs1(inst) != 0) {
					// Invalid Op-code
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tWFI\n\r");
				}
#endif
				
				context->pc += 4;
				struct retvals rtn;
				rtn.value = 0;
				rtn.error = CUSTOM_INTERNAL_WFI_SLEEP;
				return rtn;
				
			} else if (R_funct7(inst) == 0x09) {
				// Instruction: SFENCE.VMA
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tSFENCE.VMA\n\r");
				}
#endif
				
				// Do nothing, implemented as a NO-OP
				
			} else {
				// Invalid Op-code
				struct retvals rtn;
				rtn.value = inst;
				rtn.error = ILLEGAL_INSTRUCTION;
				return rtn;
			}
		} else if (I_funct3(inst) == 0x1) {
				// Instruction: CSRRW
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tCSRRW x%d, 0x%03X, x%d\n\r", I_rd(inst), I_imm(inst), I_rs1(inst));
				}
#endif
				
				uint32_t rs1b = RegVal[I_rs1(inst)];
				struct retvals rtn = CSR_Read(I_imm(inst), context, 1);
				if (rtn.error) {
					// CSR Read/Write Failed
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				CSR_Write(I_imm(inst), context, rs1b);
				
		} else if (I_funct3(inst) == 0x2) {
				// Instruction: CSRRS
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tCSRRS x%d, 0x%03X, x%d\n\r", I_rd(inst), I_imm(inst), I_rs1(inst));
				}
#endif
				
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
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				if (perm) {
					CSR_Write(I_imm(inst), context, rs1b | rtn.value);
				}
				
		} else if (I_funct3(inst) == 0x3) {
				// Instruction: CSRRC
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tCSRRC x%d, 0x%03X, x%d\n\r", I_rd(inst), I_imm(inst), I_rs1(inst));
				}
#endif
				
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
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				if (perm) {
					CSR_Write(I_imm(inst), context, ~rs1b & rtn.value);
				}
				
		} else if (I_funct3(inst) == 0x5) {
				// Instruction: CSRRWI
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tCSRRWI x%d, 0x%03X, 0x%X\n\r", I_rd(inst), I_imm(inst), I_rs1(inst));
				}
#endif
				
				uint32_t rs1b = I_rs1(inst);
				struct retvals rtn = CSR_Read(I_imm(inst), context, 1);
				if (rtn.error) {
					// CSR Read/Write Failed
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				CSR_Write(I_imm(inst), context, rs1b);
				
		} else if (I_funct3(inst) == 0x6) {
				// Instruction: CSRRSI
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tCSRRSI x%d, 0x%03X, 0x%X\n\r", I_rd(inst), I_imm(inst), I_rs1(inst));
				}
#endif
				
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
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				if (perm) {
					CSR_Write(I_imm(inst), context, rs1b | rtn.value);
				}
				
		} else if (I_funct3(inst) == 0x7) {
				// Instruction: CSRRCI
				
#ifdef DEBUG
				if (debug) {
					dprintf(STDERR, "\tCSRRCI x%d, 0x%03X, 0x%X\n\r", I_rd(inst), I_imm(inst), I_rs1(inst));
				}
#endif
				
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
					struct retvals rtn;
					rtn.value = inst;
					rtn.error = ILLEGAL_INSTRUCTION;
					return rtn;
				}
				if (I_rd(inst) != 0) {
					RegVal[I_rd(inst)] = rtn.value;
				}
				if (perm) {
					CSR_Write(I_imm(inst), context, ~rs1b & rtn.value);
				}
				
		} else {
			// Invalid Op-code
			struct retvals rtn;
			rtn.value = inst;
			rtn.error = ILLEGAL_INSTRUCTION;
			return rtn;
		}
		
	} else {
		// Invalid Op-code
		struct retvals rtn;
		rtn.value = inst;
		rtn.error = ILLEGAL_INSTRUCTION;
		return rtn;
	}
	
	context->pc += 0x4;
	struct retvals rtn;
	rtn.value = 0;
	rtn.error = CUSTOM_INTERNAL_EXECUTION_SUCCESS;
	return rtn;
}

static inline void UpdateTimer(struct cpu_context* context) {
	uint64_t up_time_us;
	
#ifdef WASM_BUILD
	js_get_timestamp_us(&up_time_us);
#else
	struct timespec curr_time;
	clock_gettime(CLOCK_REALTIME, &curr_time);
	up_time_us  = curr_time.tv_sec  * 1000000;
	up_time_us += curr_time.tv_nsec / 1000;
#endif
	
	up_time_us -= clint_start_time64;
	
	clint_time  = (uint32_t)((up_time_us >>  0) & 0xFFFFFFFF);
	clint_timeh = (uint32_t)((up_time_us >> 32) & 0xFFFFFFFF);
	
	uint64_t cmp_time_us;
	cmp_time_us   = clint_mtimecmph;
	cmp_time_us <<= 32;
	cmp_time_us  |= clint_mtimecmp;
	
#ifdef DEBUG
	if (debug_timeint) {
#endif
		if (cmp_time_us <= up_time_us) { // TODO: Double-Check if '<=' or '<' in RISC-V specs
			// Set timer interrupt
			context->csr[CSR_MIP] |=  ((uint32_t)0x080);
		} else {
			// Clear timer interrupt
			context->csr[CSR_MIP] &= ~((uint32_t)0x080);
		}
#ifdef DEBUG
	} else {
		context->csr[CSR_MIP] &= ~((uint32_t)0x080);
	}
#endif
	
	return;
}

static inline void UpdateUART() {
	// Determine RX FIFO Threshold
	uint32_t thresh = uart0_fcr >> 6;
	if        (thresh == 0x0) {
		thresh = 1;
	} else if (thresh == 0x1) {
		thresh = 4;
	} else if (thresh == 0x2) {
		thresh = 8;
	} else {
		thresh = 14;
	}
	
	// Check for interrupt in order of priority.
	// Don't worry about error conditions in emulated HW.
	uint32_t cuecount = uart0_rxcuecount;
	uint32_t timeout_interrupt = 0;
	if (cuecount > 0) {
		if (uart0_isr & uart0_lsr & 0x1) {
			timeout_interrupt = 1;
		}
		uart0_lsr |=  0x1;
	} else {
		uart0_lsr &= ~0x1;
	}
	if        ((uart0_ier & 0x01) &&  (uart0_fcr & 0x1) && (cuecount >= thresh)) {
		// RX Data Interrupt Enabled
		// FIFO Enabled.
		// RX FIFO filled to Threshold
		uart0_isr = 0x04;
	} else if ((uart0_ier & 0x01) && !(uart0_fcr & 0x1) && (cuecount > 0)) {
		// RX Data Interrupt Enabled
		// FIFO Disabled.
		// Data waiting in RX Buffer Register
		uart0_isr = 0x04;
	} else if  (timeout_interrupt) {
		uart0_isr = 0x0C;
	} else if  (uart0_ier & 0x02) {
		// TX Empty Interrupt Enabled
		// TX FIFO/Buffer Register is clear (Will always be the case in emulated HW)
		uart0_isr = 0x02;
	} else {
		uart0_isr = 0x01;
	}
	
	// Set the FIFO Enabled flags if the FIFOs are enabled
	if (uart0_fcr & 0x1) {
		uart0_isr |= 0xC0;
	}
	
	return;
}

static inline void UpdatePLIC(struct cpu_context* context) {
	// Update Peripherals
	UpdateUART();
	
	//plic_pending_array = 0;
	// Update Pending Array from Peripherals
	if (!(uart0_isr & 0x1) && (uart0_mcr & 0x08)) {
		// Wired to Interrupt <PLIC_SOURCENUM_UART>
		plic_pending_array |= (1 << PLIC_SOURCENUM_UART);
	}
	
	// Update mip.MEIP and mip.SEIP Bit Flags
	context->csr[CSR_MIP] &= ~(1 << 11);
	context->csr[CSR_MIP] &= ~(1 << 9);
	if (plic_pending_array & (1 << PLIC_SOURCENUM_UART)) {
		uint32_t priorty = plic_source_priority;
		if (plic_h0_m_pri_thres < priorty) {
			if (plic_h0_m_inter_en & (1 << PLIC_SOURCENUM_UART)) {
				context->csr[CSR_MIP] |= (1 << 11);
			}
		}
		if (plic_h0_s_pri_thres < priorty) {
			if (plic_h0_s_inter_en & (1 << PLIC_SOURCENUM_UART)) {
				context->csr[CSR_MIP] |= (1 << 9);
			}
		}
	}
	
	// Update Claim Registers
	plic_h0_m_claim_compl = 0;
	plic_h0_s_claim_compl = 0;
	if (plic_pending_array & (1 << PLIC_SOURCENUM_UART)) {
		if (plic_h0_m_inter_en & (1 << PLIC_SOURCENUM_UART)) {
			plic_h0_m_claim_compl |= PLIC_SOURCENUM_UART;
		}
		if (plic_h0_s_inter_en & (1 << PLIC_SOURCENUM_UART)) {
			plic_h0_s_claim_compl |= PLIC_SOURCENUM_UART;
		}
	}
	return;
}

static inline void TakeTrap(uint32_t exec_mode, uint32_t cause, uint32_t is_interrupt, uint32_t xtval, struct cpu_context* context) {
	
#ifdef DEBUG
	if (debug_trap) {
		dprintf(STDERR, "[Trap] PC: 0x%X, Into Mode: %d, From Mode: %d, Cause: %d, Is Int.: %d\n\r", context->pc, exec_mode, context->mode, cause, is_interrupt);
	}
#endif
	
	uint32_t xtvec;
	uint32_t ncause = cause;
	if (is_interrupt) {
		ncause |= 0x80000000;
	} else {
		cause = 0;
	}
	if (exec_mode == 3) {
		// New mode: M-Mode
		xtvec = context->csr[CSR_MTVEC];
		context->csr[CSR_MEPC] = context->pc;
		context->csr[CSR_MCAUSE] = ncause;
		context->csr[CSR_MTVAL] = xtval;
		uint32_t nstatus;
		nstatus  =  context->csr[CSR_MSTATUS] & 0xFFFFE777;
		nstatus |= (context->mode & 0x3) << 11;
		nstatus |= (context->csr[CSR_MSTATUS] & 0x8) << 4;
		context->csr[CSR_MSTATUS] = nstatus;
	} else {
		// New mode: S-Mode
		xtvec = context->csr[CSR_STVEC];
		context->csr[CSR_SEPC] = context->pc;
		context->csr[CSR_SCAUSE] = ncause;
		context->csr[CSR_STVAL] = xtval;
		uint32_t nstatus;
		nstatus  =  context->csr[CSR_MSTATUS] & 0xFFFFFEDD;
		nstatus |= (context->mode & 0x1) << 8;
		nstatus |= (context->csr[CSR_MSTATUS] & 0x2) << 4;
		context->csr[CSR_MSTATUS] = nstatus;
	}
	if (xtvec & 0x1) {
		context->pc = (xtvec & 0xFFFFFFFC) + (cause * 4);
	} else {
		context->pc =  xtvec & 0xFFFFFFFC;
	}
	context->mode = exec_mode;
	context->lr_reserve_set = (sint32_t)-1;
	return;
}

int RunLoop(struct cpu_context* context) {
	unsigned int i = 0;
	while (running) {
		// Check for interrupts
		// Update external interrupts
		UpdatePLIC(context);
		// Update timer interrupts
		if (i == 1000) {
			UpdateTimer(context);
			i = 0;
		} else {
			i++;
		}
		
		// Fire interrupts
		{
			uint32_t exec_mode = context->mode;
			uint32_t csr_mip = context->csr[CSR_MIP];
			uint32_t csr_mie = context->csr[CSR_MIE];
			uint32_t csr_mideleg = context->csr[CSR_MIDELEG];
			uint32_t csr_mstatus = context->csr[CSR_MSTATUS];
			if        (exec_mode == 0) {
				// U-Mode
				if        (csr_mip & csr_mie & 0x800) {
					// External M-Mode Interrupt
					TakeTrap(3, 11, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x008) {
					// Software M-Mode Interrupt
					TakeTrap(3, 3, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x080) {
					// Timer M-Mode Interrupt
					TakeTrap(3, 7, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x200) {
					// External S-Mode Interrupt
					if (csr_mideleg & 0x200) {
						TakeTrap(1, 9, 1, 0, context);
					} else {
						TakeTrap(3, 9, 1, 0, context);
					}
				} else if (csr_mip & csr_mie & 0x002) {
					// Software S-Mode Interrupt
					if (csr_mideleg & 0x002) {
						TakeTrap(1, 1, 1, 0, context);
					} else {
						TakeTrap(3, 1, 1, 0, context);
					}
				} else if (csr_mip & csr_mie & 0x020) {
					// Timer S-Mode Interrupt
					if (csr_mideleg & 0x020) {
						TakeTrap(1, 5, 1, 0, context);
					} else {
						TakeTrap(3, 5, 1, 0, context);
					}
				}
			} else if (exec_mode == 1) {
				// S-Mode
				if        (csr_mip & csr_mie & 0x800) {
					// External M-Mode Interrupt
					TakeTrap(3, 11, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x008) {
					// Software M-Mode Interrupt
					TakeTrap(3, 3, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x080) {
					// Timer M-Mode Interrupt
					TakeTrap(3, 7, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x200 & ~csr_mideleg) {
					// External S-Mode Interrupt - Not Delegated
					TakeTrap(3, 9, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x200 &  csr_mideleg && csr_mstatus & 0x2) {
					// External S-Mode Interrupt - Delegated
					TakeTrap(1, 9, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x002 & ~csr_mideleg) {
					// Software S-Mode Interrupt - Not Delegated
					TakeTrap(3, 1, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x002 &  csr_mideleg && csr_mstatus & 0x2) {
					// Software S-Mode Interrupt - Delegated
					TakeTrap(1, 1, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x020 & ~csr_mideleg) {
					// Timer S-Mode Interrupt - Not Delegated
					TakeTrap(3, 5, 1, 0, context);
				} else if (csr_mip & csr_mie & 0x020 &  csr_mideleg && csr_mstatus & 0x2) {
					// Timer S-Mode Interrupt - Delegated
					TakeTrap(1, 5, 1, 0, context);
				}
			} else {
				// M-Mode
				if        (csr_mstatus & 0x8) {
					if        (csr_mip & csr_mie & 0x800) {
						// External M-Mode Interrupt
						TakeTrap(3, 11, 1, 0, context);
					} else if (csr_mip & csr_mie & 0x008) {
						// Software M-Mode Interrupt
						TakeTrap(3, 3, 1, 0, context);
					} else if (csr_mip & csr_mie & 0x080) {
						// Timer M-Mode Interrupt
						TakeTrap(3, 7, 1, 0, context);
					} else if (csr_mip & csr_mie & 0x200 & ~csr_mideleg) {
						// External S-Mode Interrupt - Not Delegated
						TakeTrap(3, 9, 1, 0, context);
					} else if (csr_mip & csr_mie & 0x002 & ~csr_mideleg) {
						// Software S-Mode Interrupt - Not Delegated
						TakeTrap(3, 1, 1, 0, context);
					} else if (csr_mip & csr_mie & 0x020 & ~csr_mideleg) {
						// Timer S-Mode Interrupt - Not Delegated
						TakeTrap(3, 5, 1, 0, context);
					}
				}
			}
		}
		
		struct retvals rtn;
		rtn = ExecMemory(context->pc, context);
		if (rtn.error == CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
			rtn = ExecuteInstruction(rtn.value, context);
		} else {
			rtn.value = context->pc;
		}
		
		if (rtn.error != CUSTOM_INTERNAL_EXECUTION_SUCCESS) {
			if (rtn.error == CUSTOM_INTERNAL_WFI_SLEEP) {
				// WFI
				return 1;
			} else {
				TakeTrap(3, rtn.error, 0, rtn.value, context);
			}
		}
		
	}
	
	return 0;
}

void* RunEmu(void* ptr) {
#ifndef WASM_BUILD
	sigset_t extn_int_sig;
	sigemptyset(&extn_int_sig);
	sigaddset(&extn_int_sig, EXTERN_INT_SIG);
	pthread_sigmask(SIG_UNBLOCK, &extn_int_sig, 0);
#endif
	
	restart_loop:
	thread_lock(&spinlk);
	if (running == 2) {
		running = 1;
	} else {
		running = 0;
	}
	thread_unlock(&spinlk);
	uint32_t retval = 0;
	if (RunLoop(&cpu_cntxt)) {
		thread_lock(&spinlk);
		UpdatePLIC(&cpu_cntxt);
		UpdatePLIC(&cpu_cntxt); // Update twice because of UART implementation
		UpdateTimer(&cpu_cntxt);
		if (cpu_cntxt.csr[CSR_MIP] & cpu_cntxt.csr[CSR_MIE]) {
			retval = 0x10000;
		}
		retval |= cpu_cntxt.csr[CSR_MIE];
		
#ifndef WASM_BUILD
		if (retval & 0x10000 && running != 0) {
			running = 2;
			thread_unlock(&spinlk);
			goto restart_loop;
		}
#ifdef DEBUG
		if (retval & 0x080 && debug_timeint) {
#else
		if (retval & 0x080) {
#endif
			uint64_t up_time_us;
			up_time_us    = clint_timeh;
			up_time_us  <<= 32;
			up_time_us   |= clint_time;
			
			uint64_t cmp_time_us;
			cmp_time_us   = clint_mtimecmph;
			cmp_time_us <<= 32;
			cmp_time_us  |= clint_mtimecmp;
			
			cmp_time_us -= up_time_us; // Time until next timer interrupt
			cmp_time_us *= 1000; // Convert to nanoseconds
			
			uint64_t sec;
			uint64_t nsec;
			
			sec  = cmp_time_us / 1000000000;
			nsec = cmp_time_us % 1000000000;
			
			struct timespec slp_tm;
			slp_tm.tv_sec = sec;
			slp_tm.tv_nsec = nsec;
			
			if (running != 0) {
				running = 4;
			}
			thread_unlock(&spinlk);
			nanosleep(&slp_tm, 0);
			thread_lock(&spinlk);
			if (running == 4) {
				running = 2;
			}
			thread_unlock(&spinlk);
			goto restart_loop;
		}
#endif
		
		running = 0;
		thread_unlock(&spinlk);
	}
	thread_lock(&spinlk);
	running = 0;
	thread_unlock(&spinlk);
	return (void*)((unsigned long)retval);
}

#ifndef WASM_BUILD
void return_signal_received() {
	return;
}

static void StartEmu(pthread_t* thread) {
	thread_lock(&spinlk);
	if        (running == 0) {
		running = 2;
		pthread_create(thread, NULL, RunEmu, NULL);
	} else if (running == 4) {
		kill(0, EXTERN_INT_SIG);
	}
	thread_unlock(&spinlk);
	return;
}

signed int main(unsigned int argc, char *argv[], char *envp[]) {
	if (argc <= 2) {
		return 1;
	}
	
	signed int fd;
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
	off_t memory_size;
	memory_size = statbuf.st_size;
	void* memory_buf = malloc(memory_size);
	ret_val = read(fd, memory_buf, memory_size);
	close(fd);
	if (ret_val != memory_size) {
		free(memory_buf);
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
	off_t mmdata_size;
	mmdata_size = statbuf.st_size;
	void* mmdata_buf = malloc(mmdata_size);
	ret_val = read(fd, mmdata_buf, mmdata_size);
	close(fd);
	if (ret_val != mmdata_size) {
		free(mmdata_buf);
		free(memory_buf);
		return 11;
	}
	
	sigset_t extn_int_sig;
	sigemptyset(&extn_int_sig);
	sigaddset(&extn_int_sig, EXTERN_INT_SIG);
	sigprocmask(SIG_BLOCK, &extn_int_sig, 0);
	
	struct sigaction sact;
	sact.sa_handler = return_signal_received;
	sigemptyset(&sact.sa_mask);
	sact.sa_flags = 0;
	sigaction(EXTERN_INT_SIG, &sact, 0);
	
	// START: Setup the Terminal
	// Set TTY to Raw mode
	struct termios old_tty_settings;
	{
		struct termios raw_tty_settings;
		ioctl(STDIN, TCGETS, &old_tty_settings);
		memcpy(&raw_tty_settings, &old_tty_settings, sizeof(struct termios));
		raw_tty_settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		raw_tty_settings.c_oflag &= ~OPOST;
		raw_tty_settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		raw_tty_settings.c_cflag &= ~(CSIZE | PARENB);
		raw_tty_settings.c_cflag |= CS8;
		ioctl(STDIN, TCSETS, &raw_tty_settings);
	}
	
	InitEmu(memory_size, mmdata_size);
	
	memcpy(memory, memory_buf, memory_size);
	memcpy(mmdata, mmdata_buf, mmdata_size);
	free(memory_buf);
	free(mmdata_buf);
	
	pthread_t thread;
	StartEmu(&thread);
	
	int loop = 1;
	struct pollfd pfd;
	pfd.fd = STDIN;
	pfd.events = POLLIN;
	pfd.revents = 0;
	do {
		do {
			ret_val = poll(&pfd, 1, 5000);
		} while (ret_val < 0);
		
		if (pfd.revents & POLLIN) {
			char val;
			ret_val = read(STDIN, &val, 1);
			if (ret_val <= 0) {
				if (ret_val == EINTR) {
					continue;
				}
				dprintf(STDOUT, "Read Error\n\r");
				loop = 0;
				running = 0;
				break;
			}
			if        (val == ('q' + 1 - 'a')) {
				loop = 0;
				thread_lock(&spinlk);
				if (running == 4) {
					kill(0, EXTERN_INT_SIG);
				}
				running = 0;
				thread_unlock(&spinlk);
#ifdef DEBUG
			} else if (val == ('d' + 1 - 'a')) {
				debug = !debug;
			} else if (val == ('f' + 1 - 'a')) {
				debug_trap = !debug_trap;
			} else if (val == ('g' + 1 - 'a')) {
				debug_trapret = !debug_trapret;
			} else if (val == ('h' + 1 - 'a')) {
				debug_pagead = !debug_pagead;
			} else if (val == ('j' + 1 - 'a')) {
				debug_memaccess = !debug_memaccess;
			} else if (val == ('t' + 1 - 'a')) {
				debug_timeint = !debug_timeint;
				StartEmu(&thread);
#endif
			} else {
				thread_lock(&spinlk);
				if (uart0_rxcuecount < UART_RX_FIFO_SIZE) {
					uart0_rxcue[uart0_rxcuestoreindex] = val;
					uart0_rxcuestoreindex = (uart0_rxcuestoreindex + 1) % UART_RX_FIFO_SIZE;
					uart0_rxcuecount++;
				}
				thread_unlock(&spinlk);
				StartEmu(&thread);
			}
		}
	} while (loop);
	pthread_join(thread, NULL);
	
	ioctl(STDIN, TCSETS, &old_tty_settings);
	dprintf(STDOUT, "\e[?25h");
	
	DestroyEmu();
	
	sigprocmask(SIG_UNBLOCK, &extn_int_sig, 0);
	
	return 0;
}
#endif
