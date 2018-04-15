/*-
 * Copyright (c) 2016 Jared McNeill <jmcneill@invisible.ca>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "cpu.h"

#define	CPU_READ8(c, addr)	((c)->read8((addr)))
#define	CPU_WRITE8(c, addr, v)	((c)->write8((addr), (v)))
#define	CPU_READ16(c, addr)	(CPU_READ8((c), (addr)) | ((uint16_t)CPU_READ8((c), (addr)+1))<<8)

#define	CPU_PAGE_CROSSED(addr, off)	(((addr) & 0xff) + (off) > 0xff)

#define	NMI_VECTOR_L	0xFFFA
#define	NMI_VECTOR_H	0xFFFB

#define	RESET_VECTOR_L	0xFFFC
#define	RESET_VECTOR_H	0xFFFD

#define	INTR_VECTOR_L	0xFFFE
#define	INTR_VECTOR_H	0xFFFF

#define	STACK_TOP	0x0100

#define	OPCODE_COUNT	256

enum cpu_opcode_type {
	OP_def,
	OP_imm,
	OP_zp, OP_zpx, OP_zpy,
	OP_izx, OP_izy,
	OP_abs, OP_abx, OP_aby,
	OP_ind,
	OP_rel
};

struct cpu_opcode {
	uint8_t opcode;
	const char *name;
	enum cpu_opcode_type type;
	uint8_t args;
	int cycles;
	int (*op)(struct cpu_context *, struct cpu_opcode *);
};

#define	OP(o,n,t,a,c,f) \
	[(o)] = { .opcode = (o), .name = (n), .type = (t), .args = (a), .cycles = (c),.op = (f) }

static void
cpu_stackpush(struct cpu_context *c, uint8_t val)
{
	struct cpu_frame *f = &c->frame;

	CPU_WRITE8(c, STACK_TOP + f->SP, val);
	f->SP--;
}

static uint8_t
cpu_stackpull(struct cpu_context *c)
{
	struct cpu_frame *f = &c->frame;

	f->SP++;
	return CPU_READ8(c, STACK_TOP + f->SP);
}

static int
cpuop_notimpl(struct cpu_context *c, struct cpu_opcode *o)
{
	printf("[OPCODE %02X] not implemented\n", o->opcode);
	abort();
}

static int
cpuop_nop(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_break(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	printf("BRK $%04X\n", f->PC + o->args);

	/* Push PC and P to stack */
	cpu_stackpush(c, (f->PC + o->args) >> 8);
	cpu_stackpush(c, (f->PC + o->args) & 0xff);
	cpu_stackpush(c, f->P);

	/* Load IRQ vector into PC */
	f->PC = CPU_READ16(c, INTR_VECTOR_L);
	/* Set break flag */
	f->P |= P_B;

	return o->cycles;
}

static int
cpuop_load(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t addr;
	uint8_t nv;
	int cycles = o->cycles;

	switch (o->opcode) {
	case 0xA0:	/* LDY Immediate */
		nv = f->Y = CPU_READ8(c, f->PC + 1);
		break;
	case 0xA4:	/* LDY Zero Page */
		nv = f->Y = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		break;
	case 0xA5:	/* LDA Zero Page */
		nv = f->A = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		break;
	case 0xA6:	/* LDX Zero Page */
		nv = f->X = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		break;
	case 0xA9:	/* LDA Immediate */
		nv = f->A = CPU_READ8(c, f->PC + 1);
		break;
	case 0xAC:	/* LDY Absolute */
		nv = f->Y = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		break;
	case 0xAD:	/* LDA Absolute */
		nv = f->A = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		break;
	case 0xAE:	/* LDX Absolute */
		nv = f->X = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		break;
	case 0xB4:	/* LDY Zero Page,X */
		nv = f->Y = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		break;
	case 0xB5:	/* LDA Zero Page,X */
		nv = f->A = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		break;
	case 0xB6:	/* LDX Zero Page,Y */
		nv = f->X = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->Y));
		break;
	case 0xB9:	/* LDA Absolute,Y */
		addr = CPU_READ16(c, f->PC + 1);
		nv = f->A = CPU_READ8(c, addr + f->Y);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0xBC:	/* LDY Absolute,X */
		addr = CPU_READ16(c, f->PC + 1);
		nv = f->Y = CPU_READ8(c, addr + f->X);
		if (CPU_PAGE_CROSSED(addr, f->X))
			cycles++;
		break;
	case 0xBD:	/* LDA Absolute,X */
		addr = CPU_READ16(c, f->PC + 1);
		nv = f->A = CPU_READ8(c, addr + f->X);
		if (CPU_PAGE_CROSSED(addr, f->X))
			cycles++;
		break;
	case 0xBE:	/* LDX Absolute,Y */
		addr = CPU_READ16(c, f->PC + 1);
		nv = f->X = CPU_READ8(c, addr + f->Y);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0xA1:	/* LDA Indirect,X */
		addr = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X)) |
		       (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X + 1)) << 8;
		nv = f->A = CPU_READ8(c, addr);
		break;
	case 0xA2:	/* LDX Immediate */
		nv = f->X = CPU_READ8(c, f->PC + 1);
		break;
	case 0xB1:	/* LDA Indirect,Y */
		addr = CPU_READ8(c, CPU_READ8(c, f->PC + 1)) |
		    (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + 1)) << 8;
		nv = f->A = CPU_READ8(c, addr + f->Y);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	default:
		assert("Unsupported load register opcode" == NULL);
	}

	/* Set Zero Flag if A = 0 */
	if (nv == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;

	/* Set Negative Flag if bit 7 of A is set */
	if ((nv & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->PC += o->args;

	return cycles;
}

static int
cpuop_store(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t addr;

	switch (o->opcode) {
	case 0x81:	/* STA Indirect,X */
		addr = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X)) |
		       (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X + 1)) << 8;
		CPU_WRITE8(c, addr, f->A);
		break;
	case 0x84:	/* STY Zero Page */
		CPU_WRITE8(c, CPU_READ8(c, f->PC + 1), f->Y);
		
		break;
	case 0x85:	/* STA Zero Page */
		CPU_WRITE8(c, CPU_READ8(c, f->PC + 1), f->A);
		break;
	case 0x86:	/* STX Zero Page */
		CPU_WRITE8(c, CPU_READ8(c, f->PC + 1), f->X);
		break;
	case 0x8C:	/* STY Absolute */
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1), f->Y);
		break;
	case 0x8D:	/* STA Absolute */
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1), f->A);
		break;
	case 0x8E:	/* STX Absolute */
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1), f->X);
		break;
	case 0x91:	/* STA Indirect,Y */
		addr = CPU_READ8(c, CPU_READ8(c, f->PC + 1)) |
		    (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + 1)) << 8;
		CPU_WRITE8(c, addr + f->Y, f->A);
		break;
	case 0x94:	/* STY Zero Page,X */
		CPU_WRITE8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X), f->Y);
		break;
	case 0x95:	/* STA Zero Page,X */
		CPU_WRITE8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X), f->A);
		break;
	case 0x96:	/* STX Zero Page,Y */
		CPU_WRITE8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->Y), f->X);
		break;
	case 0x99:	/* STA Absolute,Y */
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1) + f->Y, f->A);
		break;
	case 0x9D:	/* STA Absolute,X */
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1) + f->X, f->A);
		break;
	default:
		assert("Unsupported store opcode" == NULL);
	}

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_compare(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t addr;
	uint8_t M, v;
	int cycles = o->cycles;

	switch (o->opcode) {
	case 0xC0:	/* CPY Immediate */
		M = CPU_READ8(c, f->PC + 1);
		v = f->Y;
		break;
	case 0xC1:	/* CMP Indirect,X */
		addr = CPU_READ8(c, CPU_READ8(c, f->PC + 1) + f->X) |
		       (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X + 1)) << 8;
		M = CPU_READ8(c, addr);
		v = f->A;
		break;
	case 0xC4:	/* CPY Zero Page */
		M = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		v = f->Y;
		break;
	case 0xC5:	/* CMP Zero Page */
		M = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		v = f->A;
		break;
	case 0xC9:	/* CMP Immediate */
		M = CPU_READ8(c, f->PC + 1);
		v = f->A;
		break;
	case 0xCC:	/* CPY Absolute */
		M = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		v = f->Y;
		break;
	case 0xCD:	/* CMP Absolute */
		M = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		v = f->A;
		break;
	case 0xD1:	/* CMP Indirect,Y */
		addr = CPU_READ16(c, CPU_READ8(c, f->PC + 1));
		M = CPU_READ8(c, addr + f->Y);
		v = f->A;
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0xD5:	/* CMP Zero Page,X */
		M = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		v = f->A;
		break;
	case 0xD9:	/* CMP Absolute,Y */
		addr = CPU_READ16(c, f->PC + 1);
		M = CPU_READ8(c, addr + f->Y);
		v = f->A;
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0xDD:	/* CMP Absolute,X */
		addr = CPU_READ16(c, f->PC + 1);
		M = CPU_READ8(c, addr + f->X);
		v = f->A;
		if (CPU_PAGE_CROSSED(addr, f->X))
			cycles++;
		break;
	case 0xE0:	/* CPX Immediate */
		M = CPU_READ8(c, f->PC + 1);
		v = f->X;
		break;
	case 0xE4:	/* CPX Zero Page */
		M = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		v = f->X;
		break;
	case 0xEC:	/* CPX Absolute */
		M = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		v = f->X;
		break;
	default:
		assert("Unsupported compare opcode" == NULL);
	}

	/* Set Carry Flag if value >= M */
	if (v >= M)
		f->P |= P_C;
	else
		f->P &= ~P_C;
	/* Set Zero Flag if value = M */
	if (v == M)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;
	/* Set Negative Flag if bit 7 of the result (v-M) is set */
	if ((((int)v - (int)M) & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->PC += o->args;

	return cycles;
}

static int
cpuop_increment(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint8_t ov, nv;

	switch (o->opcode) {
	case 0xC8:	/* Increment Y register */
		nv = ++f->Y;
		break;
	case 0xE6:	/* Increment Memory Zero Page */
		ov = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		nv = ov + 1;
		CPU_WRITE8(c, CPU_READ8(c, f->PC + 1), nv);
		break;
	case 0xE8:	/* Increment X register */
		nv = ++f->X;
		break;
	case 0xEE:	/* Increment Memory Absolute */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		nv = ov + 1;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1), nv);
		break;
	case 0xF6:	/* Increment Memory Zero Page,X */
		ov = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		nv = ov + 1;
		CPU_WRITE8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X), nv);
		break;
	case 0xFE:	/* Increment Memory Absolute,X */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1) + f->X);
		nv = ov + 1;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1) + f->X, nv);
		break;
	default:
		assert("Unsupported increment opcode" == NULL);
	}

	if (nv == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;
	if ((nv & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_decrement(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint8_t ov, nv;

	switch (o->opcode) {
	case 0x88:	/* Decrement Y register */
		nv = --f->Y;
		break;
	case 0xC6:	/* Decrement Memory Zero Page */
		ov = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		nv = ov - 1;
		CPU_WRITE8(c, CPU_READ8(c, f->PC + 1), nv);
		break;
	case 0xCA:	/* Decrement X register */
		nv = --f->X;
		break;
	case 0xD6:	/* Decrement Memory Zero Page,X */
		ov = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		nv = ov - 1;
		CPU_WRITE8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X), nv);
		break;
	case 0xCE:	/* Decrement Memory Absolute */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		nv = ov - 1;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1), nv);
		break;
	case 0xDE:	/* Decrement Memory Absolute,X */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1) + f->X);
		nv = ov - 1;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1) + f->X, nv);
		break;
	default:
		assert("Unsupported decrement opcode" == NULL);
	}

	if (nv == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;
	if ((nv & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_add(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t addr;
	uint8_t M, C;
	unsigned int nv;
	int cycles = o->cycles;

	C = (f->P & P_C) ? 1 : 0;

	switch (o->opcode) {
	case 0x61:	/* Add with Carry Indirect,X */
		addr = CPU_READ8(c, CPU_READ8(c, f->PC + 1) + f->X) |
		       (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X + 1)) << 8;
		M = CPU_READ8(c, addr);
		nv = f->A + M + C;
		break;
	case 0x65:	/* Add with Carry Zero Page */
		M = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		nv = f->A + M + C;
		break;
	case 0x69:	/* Add with Carry Immediate */
		M = CPU_READ8(c, f->PC + 1);
		nv = f->A + M + C;
		break;
	case 0x6D:	/* Add with Carry Absolute */
		M = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		nv = f->A + M + C;
		break;
	case 0x71:	/* Add with Carry Indirect,Y */
		addr = CPU_READ16(c, CPU_READ8(c, f->PC + 1));
		M = CPU_READ8(c, addr + f->Y);
		nv = f->A + M + C;
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0x75:	/* Add with Carry Zero Page,X */
		M = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		nv = f->A + M + C;
		break;
	case 0x79:	/* Add with Carry Absolute,Y */
		addr = CPU_READ16(c, f->PC + 1);
		M = CPU_READ8(c, addr + f->Y);
		nv = f->A + M + C;
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0x7D:	/* Add with Carry Absolute,X */
		addr = CPU_READ16(c, f->PC + 1);
		M = CPU_READ8(c, addr + f->X);
		nv = f->A + M + C;
		if (CPU_PAGE_CROSSED(addr, f->X))
			cycles++;
		break;
	default:
		assert("Unsupported add opcode" == NULL);
	}

	/* Set Zero Flag if A = 0 */
	if ((nv & 0xff) == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;

	/* Set Overflow Flag if sign bit is incorrect */
	if (((f->A ^ nv) & 0x80) && !((f->A ^ M) & 0x80))
		f->P |= P_V;
	else
		f->P &= ~P_V;

	/* Set Carry Flag if overflow in bit 7 */
	if (nv > 0xff)
		f->P |= P_C;
	else
		f->P &= ~P_C;

	/* Set Negative Flag if bit 7 is set */
	if ((nv & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->A = nv & 0xff;

	f->PC += o->args;

	return cycles;
}

static int
cpuop_subtract(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t addr;
	uint8_t M, C;
	unsigned int nv;
	int cycles = o->cycles;

	C = (f->P & P_C) ? 1 : 0;

	switch (o->opcode) {
	case 0xE1:	/* Subtract with Carry Indirect,X */
		addr = CPU_READ8(c, CPU_READ8(c, f->PC + 1) + f->X) |
		       (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X + 1)) << 8;
		M = CPU_READ8(c, addr);
		nv = f->A - M - (1 - C);
		break;
	case 0xE5:	/* Subtract with Carry Zero Page */
		M = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		nv = f->A - M - (1 - C);
		break;
	case 0xE9:	/* Subtract with Carry Immediate */
		M = CPU_READ8(c, f->PC + 1);
		nv = f->A - M - (1 - C);
		break;
	case 0xED:	/* Subtract with Carry Absolute */
		M = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		nv = f->A - M - (1 - C);
		break;
	case 0xF1:	/* Subtract with Carry Indirect,Y */
		addr = CPU_READ16(c, CPU_READ8(c, f->PC + 1));
		M = CPU_READ8(c, addr + f->Y);
		nv = f->A - M - (1 - C);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0xF5:	/* Subtract with Carry Zero Page,X */
		M = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		nv = f->A - M - (1 - C);
		break;
	case 0xF9:	/* Subtract with Carry Absolute,Y */
		addr = CPU_READ16(c, f->PC + 1);
		M = CPU_READ8(c, addr + f->Y);
		nv = f->A - M - (1 - C);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0xFD:	/* Subtract with Carry Absolute,X */
		addr = CPU_READ16(c, f->PC + 1);
		M = CPU_READ8(c, addr + f->X);
		nv = f->A - M - (1 - C);
		if (CPU_PAGE_CROSSED(addr, f->X))
			cycles++;
		break;
	default:
		assert("Unsupported subtract opcode" == NULL);
	}

	/* Set Zero Flag if A = 0 */
	if ((nv & 0xff) == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;

	/* Set Overflow Flag if sign bit is incorrect */
	if (((f->A ^ nv) & 0x80) && ((f->A ^ M) & 0x80))
		f->P |= P_V;
	else
		f->P &= ~P_V;

	/* Set Carry Flag if overflow in bit 7 */
	if (nv < 0x100)
		f->P |= P_C;
	else
		f->P &= ~P_C;

	/* Set Negative Flag if bit 7 is set */
	if ((nv & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->A = nv & 0xff;

	f->PC += o->args;

	return cycles;
}

static int
cpuop_shift(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint8_t ov, nv;

	switch (o->opcode) {
	case 0x06:	/* Arithmetic Shift Left Zero Page */
		ov = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		nv = ov << 1;
		/* Set Carry Flag to contents of old bit 7 */
		if ((ov & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ8(c, f->PC + 1), nv);
		break;
	case 0x0A:	/* Arithmetic Shift Left Accumulator */
		nv = f->A << 1;
		/* Set Carry Flag to contents of old bit 7 */
		if ((f->A & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		f->A = nv;
		break;
	case 0x0E:	/* Arithmetic Shift Left Absolute */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		nv = ov << 1;
		/* Set Carry Flag to contents of old bit 7 */
		if ((ov & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1), nv);
		break;
	case 0x16:	/* Arithmetic Shift Left Zero Page,X */
		ov = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		nv = ov << 1;
		/* Set Carry Flag to contents of old bit 7 */
		if ((ov & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X), nv);
		break;
	case 0x1E:	/* Arithmetic Shift Left Absolute,X */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1) + f->X);
		nv = ov << 1;
		/* Set Carry Flag to contents of old bit 7 */
		if ((ov & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1) + f->X, nv);
		break;
	case 0x26:	/* Rotate Left Zero Page */
		ov = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		nv = (ov << 1) | ((f->P & P_C) ? 1 : 0);
		/* Set Carry Flag to contents of old bit 7 */
		if ((ov & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ8(c, f->PC + 1), nv);
		break;
	case 0x2A:	/* Rotate Left Accumulator */
		ov = f->A;
		nv = (ov << 1) | ((f->P & P_C) ? 1 : 0);
		/* Set Carry Flag to contents of old bit 7 */
		if ((ov & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		f->A = nv;
		break;
	case 0x2E:	/* Rotate Right Absolute */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		nv = (ov << 1) | ((f->P & P_C) ? 1 : 0);
		/* Set Carry Flag to contents of old bit 7 */
		if ((ov & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1), nv);
		break;
	case 0x36:	/* Rotate Left Zero Page,X */
		ov = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		nv = (ov << 1) | ((f->P & P_C) ? 1 : 0);
		/* Set Carry Flag to contents of old bit 7 */
		if ((ov & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X), nv);
		break;
	case 0x3E:	/* Rotate Left Absolute,X */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1) + f->X);
		nv = (ov << 1) | ((f->P & P_C) ? 1 : 0);
		/* Set Carry Flag to contents of old bit 7 */
		if ((ov & 0x80) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1) + f->X, nv);
		break;
	case 0x46:	/* Logical Shift Right Zero Page */
		ov = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		nv = ov >> 1;
		/* Set Carry Flag to contents of old bit 0 */
		if ((ov & 0x1) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ8(c, f->PC + 1), nv);
		break;
	case 0x4A:	/* Logical Shift Right Accumulator */
		nv = f->A >> 1;
		/* Set Carry Flag to contents of old bit 0 */
		if ((f->A & 0x1) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		f->A = nv;
		break;
	case 0x4E:	/* Logical Shift Right Absolute */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		nv = ov >> 1;
		/* Set Carry Flag to contents of old bit 0 */
		if ((ov & 0x1) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1), nv);
		break;
	case 0x56:	/* Logical Shift Right Zero Page,X */
		ov = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		nv = ov >> 1;
		/* Set Carry Flag to contents of old bit 0 */
		if ((ov & 0x1) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X), nv);
		break;
	case 0x5E:	/* Logical Shift Right Absolute,X */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1) + f->X);
		nv = ov >> 1;
		/* Set Carry Flag to contents of old bit 0 */
		if ((ov & 0x1) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1) + f->X, nv);
		break;
	case 0x66:	/* Rotate Right Zero Page */
		ov = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		nv = (ov >> 1) | ((f->P & P_C) ? (1 << 7) : 0);
		/* Set Carry Flag to contents of old bit 0 */
		if ((ov & 0x01) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ8(c, f->PC + 1), nv);
		break;
	case 0x6A:	/* Rotate Right Accumulator */
		ov = f->A;
		nv = (ov >> 1) | ((f->P & P_C) ? (1 << 7) : 0);
		/* Set Carry Flag to contents of old bit 0 */
		if ((ov & 0x01) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		f->A = nv;
		break;
	case 0x6E:	/* Rotate Right Absolute */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		nv = (ov >> 1) | ((f->P & P_C) ? (1 << 7) : 0);
		/* Set Carry Flag to contents of old bit 0 */
		if ((ov & 0x01) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1), nv);
		break;
	case 0x76:	/* Rotate Right Zero Page,X */
		ov = CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		nv = (ov >> 1) | ((f->P & P_C) ? (1 << 7) : 0);
		/* Set Carry Flag to contents of old bit 0 */
		if ((ov & 0x01) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X), nv);
		break;
	case 0x7E:	/* Rotate Right Absolute,X */
		ov = CPU_READ8(c, CPU_READ16(c, f->PC + 1) + f->X);
		nv = (ov >> 1) | ((f->P & P_C) ? (1 << 7) : 0);
		/* Set Carry Flag to contents of old bit 0 */
		if ((ov & 0x01) != 0)
			f->P |= P_C;
		else
			f->P &= ~P_C;
		CPU_WRITE8(c, CPU_READ16(c, f->PC + 1) + f->X, nv);
		break;
	default:
		assert("Unsupported shift opcode" == NULL);
	}

	/* Set Zero Flag if A = 0 */
	if (nv == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;

	/* Set Negative Flag if bit 7 is set */
	if ((nv & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_or(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t addr;
	int cycles = o->cycles;

	switch (o->opcode) {
	case 0x01:	/* Logical inclusive OR Indirect,X */
		addr = CPU_READ8(c, CPU_READ8(c, f->PC + 1) + f->X) |
		       (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X + 1)) << 8;
		f->A |= CPU_READ8(c, addr);
		break;
	case 0x05:	/* Logical inclusive OR Zero Page */
		f->A |= CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		break;
	case 0x09:	/* Logical inclusive OR Immediate */
		f->A |= CPU_READ8(c, f->PC + 1);
		break;
	case 0x0D:	/* Logical inclusive OR Absolute */
		f->A |= CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		break;
	case 0x11:	/* Logical inclusive OR Indirect,Y */
		addr = CPU_READ16(c, CPU_READ8(c, f->PC + 1));
		f->A |= CPU_READ8(c, addr + f->Y);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0x15:	/* Logical inclusive OR Zero Page,X */
		f->A |= CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		break;
	case 0x19:	/* Logical inclusive OR Absolute,Y */
		addr = CPU_READ16(c, f->PC + 1);
		f->A |= CPU_READ8(c, addr + f->Y);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0x1D:	/* Logical inclusive OR Absolute,X */
		addr = CPU_READ16(c, f->PC + 1);
		f->A |= CPU_READ8(c, addr + f->X);
		if (CPU_PAGE_CROSSED(addr, f->X))
			cycles++;
		break;
	default:
		assert("Unsupported or opcode" == NULL);
	}

	/* Set Zero Flag if A = 0 */
	if (f->A == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;

	/* Set Negative Flag if bit 7 is set */
	if ((f->A & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->PC += o->args;

	return cycles;
}

static int
cpuop_xor(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t addr;
	int cycles = o->cycles;

	switch (o->opcode) {
	case 0x41:	/* Exclusive OR Indirect,X */
		addr = CPU_READ8(c, CPU_READ8(c, f->PC + 1) + f->X) |
		       (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X + 1)) << 8;
		f->A ^= CPU_READ8(c, addr);
		break;
	case 0x45:	/* Exclusive OR Zero Page */
		f->A ^= CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		break;
	case 0x49:	/* Exclusive OR Immediate */
		f->A ^= CPU_READ8(c, f->PC + 1);
		break;
	case 0x4D:	/* Exclusive OR Absolute */
		f->A ^= CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		break;
	case 0x51:	/* Exclusive OR Indirect,Y */
		addr = CPU_READ16(c, CPU_READ8(c, f->PC + 1));
		f->A ^= CPU_READ8(c, addr + f->Y);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0x55:	/* Exclusive OR Zero Page,X */
		f->A ^= CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		break;
	case 0x59:	/* Exclusive OR Absolute,Y */
		addr = CPU_READ16(c, f->PC + 1);
		f->A ^= CPU_READ8(c, addr + f->Y);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0x5D:	/* Exclusive OR Absolute,X */
		addr = CPU_READ16(c, f->PC + 1);
		f->A ^= CPU_READ8(c, addr + f->X);
		if (CPU_PAGE_CROSSED(addr, f->X))
			cycles++;
		break;
	default:
		assert("Unsupported xor opcode" == NULL);
	}

	/* Set Zero Flag if A = 0 */
	if (f->A == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;

	/* Set Negative Flag if bit 7 is set */
	if ((f->A & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->PC += o->args;

	return cycles;
}

static int
cpuop_and(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t addr;
	int cycles = o->cycles;

	switch (o->opcode) {
	case 0x21:	/* Logical AND Indirect,X */
		addr = CPU_READ8(c, CPU_READ8(c, f->PC + 1) + f->X) |
		       (uint16_t)CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X + 1)) << 8;
		f->A &= CPU_READ8(c, addr);
		break;
	case 0x25:	/* Logical AND Zero Page */
		f->A &= CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		break;
	case 0x29:	/* Logical AND Immediate */
		f->A &= CPU_READ8(c, f->PC + 1);
		break;
	case 0x2D:	/* Logical AND Absolute */
		f->A &= CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		break;
	case 0x31:	/* Logical AND Indirect,Y */
		addr = CPU_READ16(c, CPU_READ8(c, f->PC + 1));
		f->A &= CPU_READ8(c, addr + f->Y);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	case 0x35:	/* Logical AND Zero Page,X */
		f->A &= CPU_READ8(c, (uint8_t)(CPU_READ8(c, f->PC + 1) + f->X));
		break;
	case 0x3D:	/* Logical AND Absolute,X */
		addr = CPU_READ16(c, f->PC + 1);
		f->A &= CPU_READ8(c, addr + f->X);
		if (CPU_PAGE_CROSSED(addr, f->X))
			cycles++;
		break;
	case 0x39:	/* Logical AND Absolute,Y */
		addr = CPU_READ16(c, f->PC + 1);
		f->A &= CPU_READ8(c, addr + f->Y);
		if (CPU_PAGE_CROSSED(addr, f->Y))
			cycles++;
		break;
	default:
		assert("Unsupported or opcode" == NULL);
	}

	/* Set Zero Flag if A = 0 */
	if (f->A == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;

	/* Set Negative Flag if bit 7 is set */
	if ((f->A & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->PC += o->args;

	return cycles;
}

static int
cpuop_test(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint8_t val, M;

	switch (o->opcode) {
	case 0x24:	/* Bit Test Zero Page */
		M = CPU_READ8(c, CPU_READ8(c, f->PC + 1));
		val = f->A & M;
		break;
	case 0x2C:	/* Bit Test Absolute */
		M = CPU_READ8(c, CPU_READ16(c, f->PC + 1));
		val = f->A & M;
		break;
	default:
		assert("Unsupported test opcode" == NULL);
	}

	/* Set Zero Flag if result = 0 */
	if (val == 0)
		f->P |= P_Z;
	else
		f->P &= ~P_Z;

	/* Set Overflow Flag if bit 6 in the memory value is set */
	if ((M & 0x40) != 0)
		f->P |= P_V;
	else
		f->P &= ~P_V;
 
	/* Set Negative Flag if bit 7 in the memory value is set */
	if ((M & 0x80) != 0)
		f->P |= P_N;
	else
		f->P &= ~P_N;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_jmp(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t addr, pc;

	switch (o->opcode) {
	case 0x20:	/* Jump to Subroutine */
		//printf("JMP (return point $%04X)\n", f->PC + o->args);
		/* Push the address (minus one) of the return point to the stack */
		pc = f->PC + o->args - 1;
		cpu_stackpush(c, pc >> 8);
		cpu_stackpush(c, pc & 0xff);
		/* Set program counter to the target memory address */
		f->PC = CPU_READ16(c, f->PC + 1);
		break;
	case 0x4C:	/* Jump Absolute */
		f->PC = CPU_READ16(c, f->PC + 1);
		break;
	case 0x6C:	/* Jump Indirect */
		addr = CPU_READ16(c, f->PC + 1);
		/* Errata: JMP indirect does not advance pages if the lower eight bits of the specified address is $FF. */
		if ((addr & 0xff) == 0xff) {
			f->PC = CPU_READ8(c, addr) |
			    (uint16_t)CPU_READ8(c, addr & ~0xff) << 8;
		} else {
			f->PC = CPU_READ16(c, addr);
		}
		break;
	default:
		assert("Unsupported jmp opcode" == NULL);
	}

	return o->cycles;
}

static int
cpuop_ret(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	uint16_t pc;

	switch (o->opcode) {
	case 0x40:	/* Return from Interrupt */
		/* Pull the P flags from the stack */
		f->P = cpu_stackpull(c);
		/* Pop the PC (minus one) from the stack */
		f->PC = cpu_stackpull(c) | (uint16_t)cpu_stackpull(c) << 8;
		//printf("RTI $%04X\n", f->PC);
		break;
	case 0x60:	/* Return from Subroutine */
		/* Pull the PC (minus one) from the stack */
		pc = (cpu_stackpull(c) | (uint16_t)cpu_stackpull(c) << 8) + 1;
		f->PC = pc;
		//printf("RTS $%04X\n", f->PC);
		break;
	default:
		assert("Unsupported ret opcode" == NULL);
	}

	return o->cycles;
}

static int
cpuop_push(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	switch (o->opcode) {
	case 0x08:	/* Push Processor Status */
		cpu_stackpush(c, f->P | P_U);
		break;
	case 0x48:	/* Push Accumulator */
		cpu_stackpush(c, f->A);
		break;
	default:
		assert("Unsupported push opcode" == NULL);
	}

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_pull(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	switch (o->opcode) {
	case 0x28:	/* Pull Processor Status */
		f->P = cpu_stackpull(c) | P_U;
		break;
	case 0x68:	/* Pull Accumulator */
		f->A = cpu_stackpull(c);
		/* Set Zero Flag if A = 0 */
		if (f->A == 0)
			f->P |= P_Z;
		else
			f->P &= ~P_Z;
		/* Set Negative Flag if bit 7 of A is set */
		if ((f->A & 0x80) != 0)
			f->P |= P_N;
		else
			f->P &= ~P_N;
		break;
	default:
		assert("Unsupported pull opcode" == NULL);
	}

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_cli(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	f->P &= ~P_I;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_sei(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	f->P |= P_I;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_cld(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	f->P &= ~P_D;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_sed(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	f->P |= P_D;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_sec(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	f->P |= P_C;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_clc(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	f->P &= ~P_C;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_clv(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	f->P &= ~P_V;

	f->PC += o->args;

	return o->cycles;
}

static int
cpuop_branch(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	int8_t rel;
	int cycles = o->cycles;
	int branch;

	rel = (int8_t)CPU_READ8(c, f->PC + 1);

	f->PC += o->args;

	switch (o->opcode) {
	case 0x10:	/* Branch if positive */
		branch = (f->P & P_N) == 0;
		break;
	case 0x30:	/* Branch if minus */
		branch = (f->P & P_N) != 0;
		break;
	case 0x50:	/* Branch if overflow clear */
		branch = (f->P & P_V) == 0;
		break;
	case 0x70:	/* Branch if overflow set */
		branch = (f->P & P_V) != 0;
		break;
	case 0x90:	/* Branch if carry clear */
		branch = (f->P & P_C) == 0;
		break;
	case 0xB0:	/* Branch if carry set */
		branch = (f->P & P_C) != 0;
		break;
	case 0xD0:	/* Branch if not equal */
		branch = (f->P & P_Z) == 0;
		break;
	case 0xF0:	/* Branch if equal */
		branch = (f->P & P_Z) != 0;
		break;
	default:
		assert("Unsupported branch opcode" == NULL);
	}

	if (branch) {
		cycles++;
		if (CPU_PAGE_CROSSED(f->PC, rel))
			cycles += 2;
		f->PC += rel;
	}

	return cycles;
}

static int
cpuop_transfer(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;

	switch (o->opcode) {
	case 0x8A:	/* Transfer X to accumulator */
		f->A = f->X;
		/* Set Zero Flag if A = 0 */
		if (f->A == 0)
			f->P |= P_Z;
		else
			f->P &= ~P_Z;
		/* Set Negative Flag if bit 7 is set */
		if ((f->A & 0x80) != 0)
			f->P |= P_N;
		else
			f->P &= ~P_N;
		break;
	case 0x98:	/* Transfer Y to accumulator */
		f->A = f->Y;
		/* Set Zero Flag if A = 0 */
		if (f->A == 0)
			f->P |= P_Z;
		else
			f->P &= ~P_Z;
		/* Set Negative Flag if bit 7 is set */
		if ((f->A & 0x80) != 0)
			f->P |= P_N;
		else
			f->P &= ~P_N;
		break;
	case 0x9A:	/* Transfer X to stack pointer */
		f->SP = f->X;
		break;
	case 0xA8:	/* Transfer accumulator to Y */
		f->Y = f->A;
		if (f->Y == 0)
			f->P |= P_Z;
		else
			f->P &= ~P_Z;
		/* Set Negative Flag if bit 7 is set */
		if ((f->Y & 0x80) != 0)
			f->P |= P_N;
		else
			f->P &= ~P_N;
		break;
	case 0xAA:	/* Transfer accumulator to X */
		f->X = f->A;
		if (f->X == 0)
			f->P |= P_Z;
		else
			f->P &= ~P_Z;
		/* Set Negative Flag if bit 7 is set */
		if ((f->X & 0x80) != 0)
			f->P |= P_N;
		else
			f->P &= ~P_N;
		break;
	case 0xBA:	/* Transfer Stack Pointer to X */
		f->X = f->SP;
		/* Set Zero Flag if X = 0 */
		if (f->X == 0)
			f->P |= P_Z;
		else
			f->P &= ~P_Z;
		/* Set Negative Flag if bit 7 is set */
		if ((f->X & 0x80) != 0)
			f->P |= P_N;
		else
			f->P &= ~P_N;
		break;
	default:
		assert("Unsupported transfer opcode" == NULL);
	}

	f->PC += o->args;

	return o->cycles;
}



static struct cpu_opcode cpu_opcodes[OPCODE_COUNT] = {
	OP(0x00, "BRK", OP_def, 2, 7, cpuop_break),		/* Break */
	OP(0x01, "ORA", OP_izx, 2, 6, cpuop_or),		/* Logical Inclusive OR (indirect,X) */
	OP(0x05, "ORA", OP_zp,  2, 3, cpuop_or),		/* Logical Inclusive OR (zero page) */
	OP(0x06, "ASL", OP_zp,  2, 5, cpuop_shift),		/* Arithmetic shift left (zero page) */
	OP(0x08, "PHP", OP_def, 1, 3, cpuop_push),		/* Push processor status */
	OP(0x09, "ORA", OP_imm, 2, 2, cpuop_or),		/* Logical Inclusive OR (immediate) */
	OP(0x0A, "ASL", OP_def, 1, 2, cpuop_shift),		/* Arithmetic shift left (accumulator) */
	OP(0x0D, "ANC", OP_abs, 3, 4, cpuop_or),		/* Logical Inclusive OR (absolute) */
	OP(0x0E, "ASL", OP_abs, 3, 6, cpuop_shift),		/* Arithmetic shift left (absolute) */
	OP(0x10, "BPL", OP_rel, 2, 2, cpuop_branch),		/* Branch if positive */
	OP(0x11, "ORA", OP_izy, 2, 5, cpuop_or),		/* Logical Inclusive OR (indirect,Y) */
	OP(0x15, "ORA", OP_zpx, 2, 4, cpuop_or),		/* Logical Inclusive OR (zero page,X) */
	OP(0x16, "ASL", OP_zpx, 2, 6, cpuop_shift),		/* Arithmetic shift left (zero page,X) */
	OP(0x18, "CLC", OP_def, 1, 2, cpuop_clc),		/* Clear carry flag */
	OP(0x19, "ORA", OP_aby, 3, 4, cpuop_or),		/* Logical Inclusive OR (absolute,Y) */
	OP(0x1D, "ORA", OP_abx, 3, 4, cpuop_or),		/* Logical Inclusive OR (absolute,X) */
	OP(0x1E, "ASL", OP_abx, 3, 7, cpuop_shift),		/* Arithmetic shift left (absolute,X) */
	OP(0x20, "JSR", OP_abs, 3, 6, cpuop_jmp),		/* Jump to subroutine */
	OP(0x21, "AND", OP_izx, 2, 6, cpuop_and),		/* Logical AND (indirect,X) */
	OP(0x24, "BIT", OP_zp,  2, 3, cpuop_test),		/* Bit test (zero page) */
	OP(0x25, "AND", OP_zp,  2, 3, cpuop_and),		/* Logical AND (zero page) */
	OP(0x26, "ROL", OP_zp,  2, 5, cpuop_shift),		/* Rotate left (zero page) */
	OP(0x28, "PLP", OP_def, 1, 4, cpuop_pull),		/* Pull processor status */
	OP(0x29, "AND", OP_imm, 2, 2, cpuop_and),		/* Logical AND (immediate) */
	OP(0x2A, "ROL", OP_def, 1, 2, cpuop_shift),		/* Rotate left (accumulator) */
	OP(0x2C, "BIT", OP_abs, 3, 4, cpuop_test),		/* Bit test (absolute) */
	OP(0x2D, "AND", OP_abs, 3, 4, cpuop_and),		/* Logical AND (absolute) */
	OP(0x2E, "ROL", OP_abs, 3, 6, cpuop_shift),		/* Rotate left (absolute) */
	OP(0x30, "BMI", OP_rel, 2, 2, cpuop_branch),		/* Branch if minus */
	OP(0x31, "AND", OP_izy, 2, 5, cpuop_and),		/* Logical AND (indirect,Y) */
	OP(0x35, "AND", OP_zpx, 2, 4, cpuop_and),		/* Logical AND (zero page,X) */
	OP(0x36, "ROL", OP_zpx, 2, 6, cpuop_shift),		/* Rotate left (zero page,X) */
	OP(0x38, "SEC", OP_def, 1, 2, cpuop_sec),		/* Set carry flag */
	OP(0x39, "AND", OP_aby, 3, 4, cpuop_and),		/* Logical AND (absolute,Y) */
	OP(0x3D, "AND", OP_abx, 3, 4, cpuop_and),		/* Logical AND (absolute,X) */
	OP(0x3E, "ROL", OP_abx, 3, 7, cpuop_shift),		/* Rotate left (absolute,X) */
	OP(0x40, "RTI", OP_def, 1, 6, cpuop_ret),		/* Return from interrupt */
	OP(0x41, "EOR", OP_izx, 2, 6, cpuop_xor),		/* Exclusive OR (indirect,X) */
	OP(0x45, "EOR", OP_zp,  2, 3, cpuop_xor),		/* Exclusive OR (zero page) */
	OP(0x46, "LSR", OP_zp,  2, 5, cpuop_shift),		/* Logical shift right (zero page) */
	OP(0x48, "PHA", OP_def, 1, 3, cpuop_push),		/* Push accumulator */
	OP(0x49, "EOR", OP_imm, 2, 2, cpuop_xor),		/* Exclusive OR (immediate) */
	OP(0x4A, "LSR", OP_def, 1, 2, cpuop_shift),		/* Logical shift right (accumulator) */
	OP(0x4C, "JMP", OP_abs, 3, 3, cpuop_jmp),		/* Jump (absolute) */
	OP(0x4D, "EOR", OP_abs, 3, 4, cpuop_xor),		/* Exclusive OR (absolute) */
	OP(0x4E, "LSR", OP_abs, 3, 6, cpuop_shift),		/* Logical shift right (absolute) */
	OP(0x50, "BVC", OP_rel, 2, 2, cpuop_branch),		/* Branch if overflow clear */
	OP(0x51, "EOR", OP_izy, 2, 5, cpuop_xor),		/* Exclusive OR (indirect,Y) */
	OP(0x55, "EOR", OP_zpx, 2, 4, cpuop_xor),		/* Exclusive OR (zero page,X) */
	OP(0x56, "LSR", OP_zpx, 2, 6, cpuop_shift),		/* Logical shift right (zero page,X) */
	OP(0x58, "CLI", OP_def, 1, 2, cpuop_cli),		/* Clear interrupt disable */
	OP(0x59, "EOR", OP_aby, 3, 4, cpuop_xor),		/* Exclusive OR (absolute,Y) */
	OP(0x5D, "EOR", OP_abx, 3, 4, cpuop_xor),		/* Exclusive OR (absolute,X) */
	OP(0x5E, "LSR", OP_abx, 3, 7, cpuop_shift),		/* Logical shift right (absolute,X) */
	OP(0x60, "RTS", OP_def, 1, 6, cpuop_ret),		/* Return from subroutine */
	OP(0x61, "ADC", OP_izx, 2, 6, cpuop_add),		/* Add with carry (indirect,X) */
	OP(0x65, "ADC", OP_zp,  2, 3, cpuop_add),		/* Add with carry (zero page) */
	OP(0x66, "ROR", OP_zp,  2, 5, cpuop_shift),		/* Rotate right (zero page) */
	OP(0x68, "PLA", OP_def, 1, 4, cpuop_pull),		/* Pull accumulator */
	OP(0x69, "ADC", OP_imm, 2, 2, cpuop_add),		/* Add with carry (immediate) */
	OP(0x6A, "ROR", OP_def, 1, 2, cpuop_shift),		/* Rotate right accumulator */
	OP(0x6C, "JMP", OP_ind, 3, 5, cpuop_jmp),		/* Jump (indirect) */
	OP(0x6D, "ADC", OP_abs, 3, 4, cpuop_add),		/* Add with carry (absolute) */
	OP(0x6E, "ROR", OP_abs, 3, 6, cpuop_shift),		/* Rotate right (absolute) */
	OP(0x70, "BVS", OP_rel, 2, 2, cpuop_branch),		/* Branch if overflow set */
	OP(0x71, "ADC", OP_izy, 2, 5, cpuop_add),		/* Add with carry (indirect,Y) */
	OP(0x75, "ADC", OP_zpx, 2, 4, cpuop_add),		/* Add with carry (zero page,X) */
	OP(0x76, "ROR", OP_zpx, 2, 6, cpuop_shift),		/* Rotate right (zero page,X) */
	OP(0x78, "SEI", OP_def, 1, 2, cpuop_sei),		/* Set interrupt disable */
	OP(0x79, "ADC", OP_aby, 3, 4, cpuop_add),		/* Add with carry (absolute,Y) */
	OP(0x7D, "ADC", OP_abx, 3, 4, cpuop_add),		/* Add with carry (absolute,X) */
	OP(0x7E, "ROR", OP_abx, 3, 7, cpuop_shift),		/* Rotate right (absolute,X) */
	OP(0x81, "STA", OP_izx, 2, 6, cpuop_store),		/* STA (indirect,X) */
	OP(0x84, "STY", OP_zp,  2, 3, cpuop_store),		/* STY (zero page) */
	OP(0x85, "STA", OP_zp,  2, 3, cpuop_store),		/* STA (zero page) */
	OP(0x86, "STX", OP_zp,  2, 3, cpuop_store),		/* STX (zero page) */
	OP(0x88, "DEY", OP_def, 1, 2, cpuop_decrement),		/* Decrement Y register */
	OP(0x8A, "TXA", OP_def, 1, 2, cpuop_transfer),		/* Transfer X to accumulator */
	OP(0x8C, "STY", OP_abs, 3, 4, cpuop_store),		/* STY (absolute) */
	OP(0x8D, "STA", OP_abs, 3, 4, cpuop_store),		/* STA (absolute) */
	OP(0x8E, "STX", OP_abs, 3, 4, cpuop_store),		/* STX (absolute) */
	OP(0x90, "BCC", OP_rel, 2, 2, cpuop_branch),		/* Branch if carry clear */
	OP(0x91, "STA", OP_izy, 2, 6, cpuop_store),		/* STA (indirect,Y) */
	OP(0x94, "STY", OP_zpx, 2, 4, cpuop_store),		/* STY (zero page,X) */
	OP(0x95, "STA", OP_zpx, 2, 4, cpuop_store),		/* STA (zero page,X) */
	OP(0x96, "STX", OP_zpy, 2, 4, cpuop_store),		/* STX (zero page,Y) */
	OP(0x98, "TYA", OP_def, 1, 2, cpuop_transfer),		/* Transfer Y to accumulator */
	OP(0x99, "STA", OP_aby, 3, 5, cpuop_store),		/* STA (absolute,Y) */
	OP(0x9A, "TXS", OP_def, 1, 2, cpuop_transfer),		/* Transfer X to stack pointer */
	OP(0x9D, "STA", OP_abx, 3, 5, cpuop_store),		/* STA (absolute,X) */
	OP(0xA0, "LDY", OP_imm, 2, 2, cpuop_load),		/* LDY (immediate) */
	OP(0xA1, "LDA", OP_izx, 2, 6, cpuop_load),		/* LDA (indirect,X) */
	OP(0xA2, "LDX", OP_imm, 2, 2, cpuop_load),		/* LDX (immediate) */
	OP(0xA4, "LDY", OP_zp,  2, 3, cpuop_load),		/* LDY (zero page) */
	OP(0xA5, "LDA", OP_zp,  2, 3, cpuop_load),		/* LDA (zero page) */
	OP(0xA6, "LDX", OP_zp,  2, 3, cpuop_load),		/* LDX (zero page) */
	OP(0xA8, "TAY", OP_def, 1, 2, cpuop_transfer),		/* Transfer accumulator to Y */
	OP(0xA9, "LDA", OP_imm, 2, 2, cpuop_load),		/* LDA (immediate) */
	OP(0xAA, "TXA", OP_def, 1, 2, cpuop_transfer),		/* Transfer accumulator to X */
	OP(0xAC, "LDY", OP_abs, 3, 4, cpuop_load),		/* LDY (absolute) */
	OP(0xAD, "LDA", OP_abs, 3, 4, cpuop_load),		/* LDA (absolute) */
	OP(0xAE, "LDX", OP_abs, 3, 4, cpuop_load),		/* LDX (absolute) */
	OP(0xB0, "BCS", OP_rel, 2, 2, cpuop_branch),		/* Branch if carry set */
	OP(0xB1, "LDA", OP_izy, 2, 5, cpuop_load),		/* LDA (indirect,Y) */
	OP(0xB4, "LDY", OP_zpx, 2, 4, cpuop_load),		/* LDY (zero page,X) */
	OP(0xB5, "LDA", OP_zpx, 2, 4, cpuop_load),		/* LDA (zero page,X) */
	OP(0xB6, "LDA", OP_zpy, 2, 4, cpuop_load),		/* LDA (zero page,Y) */
	OP(0xB8, "CLV", OP_def, 1, 2, cpuop_clv),		/* Clear overflow flag */
	OP(0xB9, "LDA", OP_aby, 3, 4, cpuop_load),		/* LDA (absolute,Y) */
	OP(0xBA, "TSX", OP_def, 1, 2, cpuop_transfer),		/* Transfer stack pointer to X */
	OP(0xBC, "LDY", OP_abx, 3, 4, cpuop_load),		/* LDY (absolute,X) */
	OP(0xBD, "LDA", OP_abx, 3, 4, cpuop_load),		/* LDA (absolute,X) */
	OP(0xBE, "LDX", OP_aby, 3, 4, cpuop_load),		/* LDX (absolute,Y) */
	OP(0xC0, "CPY", OP_imm, 2, 2, cpuop_compare),		/* Compare Y register (immediate) */
	OP(0xC1, "CMP", OP_izx, 2, 6, cpuop_compare),		/* Compare (indirect,X) */
	OP(0xC4, "CPY", OP_zp,  2, 3, cpuop_compare),		/* Compare Y register (zero page) */
	OP(0xC5, "CMP", OP_zp,  2, 3, cpuop_compare),		/* Compare (zero page) */
	OP(0xC6, "DEC", OP_zp,  2, 5, cpuop_decrement),		/* Decrement memory (zero page) */
	OP(0xC8, "INY", OP_def, 1, 2, cpuop_increment),		/* Increment Y register */
	OP(0xC9, "CMP", OP_imm, 2, 2, cpuop_compare),		/* Compare (immediate) */
	OP(0xCA, "DEX", OP_def, 1, 2, cpuop_decrement),		/* Decrement X register */
	OP(0xCC, "CPY", OP_abs, 3, 4, cpuop_compare),		/* Compare Y register (absolute) */
	OP(0xCD, "CMP", OP_abs, 3, 4, cpuop_compare),		/* Compare (absolute) */
	OP(0xCE, "DEC", OP_abs, 3, 6, cpuop_decrement),		/* Decrement memory (absolute) */
	OP(0xD0, "BNE", OP_rel, 2, 2, cpuop_branch),		/* Branch if not equal */
	OP(0xD1, "CMP", OP_izy, 2, 5, cpuop_compare),		/* Compare (indirect,Y) */
	OP(0xD5, "CMP", OP_zpx, 2, 4, cpuop_compare),		/* Compare (zero page,X) */
	OP(0xD6, "DEC", OP_zpx, 2, 6, cpuop_decrement),		/* Decrement memory (zero page,X) */
	OP(0xD8, "CLD", OP_def, 1, 2, cpuop_cld),		/* Clear decimal mode */
	OP(0xD9, "CMP", OP_aby, 3, 4, cpuop_compare),		/* Compare (absolute,Y) */
	OP(0xDD, "CMP", OP_abx, 3, 4, cpuop_compare),		/* Compare (absolute,X) */
	OP(0xDE, "DEC", OP_abx, 3, 7, cpuop_decrement),		/* Decrement memory (absolute,X) */
	OP(0xE0, "CPX", OP_imm, 2, 2, cpuop_compare),		/* Compare X register (immediate) */
	OP(0xE1, "SBC", OP_izx, 2, 6, cpuop_subtract),		/* Subtract with carry (indirect,X) */
	OP(0xE4, "CPX", OP_zp,  2, 3, cpuop_compare),		/* Compare X register (zero page) */
	OP(0xE5, "SBC", OP_zp,  2, 3, cpuop_subtract),		/* Subtract with carry (zero page) */
	OP(0xE6, "INC", OP_zp,  2, 5, cpuop_increment),		/* Increment memory (zero page) */
	OP(0xE8, "INX", OP_def, 1, 2, cpuop_increment),		/* Increment X register */
	OP(0xE9, "SBC", OP_imm, 2, 2, cpuop_subtract),		/* Subtract with carry (immediate) */
	OP(0xEA, "NOP", OP_def, 1, 2, cpuop_nop),		/* No operation (implied) */
	OP(0xEC, "CPX", OP_abs, 3, 4, cpuop_compare),		/* Compare X register (absolute) */
	OP(0xED, "SBC", OP_abs, 3, 4, cpuop_subtract),		/* Subtract with carry (absolute) */
	OP(0xEE, "INC", OP_abs, 3, 6, cpuop_increment),		/* Increment memory (absolute) */
	OP(0xF0, "BEQ", OP_rel, 2, 2, cpuop_branch),		/* Branch if equal */
	OP(0xF1, "SBC", OP_izy, 2, 5, cpuop_subtract),		/* Subtract with carry (indirect,Y) */
	OP(0xF5, "SBC", OP_zpx, 2, 4, cpuop_subtract),		/* Subtract with carry (zero page,X) */
	OP(0xF6, "INC", OP_zpx, 2, 6, cpuop_increment),		/* Increment memory (zero page,X) */
	OP(0xF8, "SED", OP_def, 1, 2, cpuop_sed),		/* Set decimal flag */
	OP(0xF9, "SBC", OP_aby, 3, 4, cpuop_subtract),		/* Subtract with carry (absolute,Y) */
	OP(0xFD, "SBC", OP_abx, 3, 4, cpuop_subtract),		/* Subtract with carry (absolute,X) */
	OP(0xFE, "INC", OP_abx, 3, 7, cpuop_increment),		/* Increment memory (absolute,X) */
};

static void
cpu_decodeop(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	int i;

	printf("%s", o->name);
	switch (o->type) {
	case OP_imm:
		assert(o->args == 2);
		printf(" #$%02X", CPU_READ8(c, f->PC + 1));
		break;
	case OP_zp:
		assert(o->args == 2);
		printf(" $%02X", CPU_READ8(c, f->PC + 1));
		break;
	case OP_zpx:
		assert(o->args == 2);
		printf(" $%02X,X", CPU_READ8(c, f->PC + 1));
		break;
	case OP_zpy:
		assert(o->args == 2);
		printf(" $%02X,Y", CPU_READ8(c, f->PC + 1));
		break;
	case OP_izx:
		assert(o->args == 2);
		printf(" ($%02X,X)", CPU_READ8(c, f->PC + 1));
		break;
	case OP_izy:
		assert(o->args == 2);
		printf(" ($%02X),Y", CPU_READ8(c, f->PC + 1));
		break;
	case OP_abs:
		assert(o->args == 3);
		printf(" $%04X", CPU_READ16(c, f->PC + 1));
		break;
	case OP_abx:
		assert(o->args == 3);
		printf(" $%04X,X", CPU_READ16(c, f->PC + 1));
		break;
	case OP_aby:
		assert(o->args == 3);
		printf(" $%04X,Y", CPU_READ16(c, f->PC + 1));
		break;
	case OP_ind:
		assert(o->args == 3);
		printf(" ($%04X)", CPU_READ16(c, f->PC + 1));
		break;
	case OP_rel:
		assert(o->args == 2);
		printf(" $%04X # PC-relative", f->PC + o->args + CPU_READ8(c, f->PC + 1));
		break;
	case OP_def:
		break;
	default:
		for (i = 1; i < o->args; i++)
			printf(" $%02X", CPU_READ8(c, f->PC + i));
		break;
	}
	printf("\n");
}

static void
cpu_dumpop(struct cpu_context *c, struct cpu_opcode *o)
{
	struct cpu_frame *f = &c->frame;
	int i;

	printf("%8d ", (int)c->insns);

	printf("[PC=$%04X SP=$%02X A=$%02X X=$%02X Y=$%02X P=$%02X %c%c%c%c%c%c%c%c]        ",
	    f->PC, f->SP, (uint8_t)f->A, f->X, f->Y, f->P,
	    (f->P & P_N) ? 'N' : '-',
	    (f->P & P_V) ? 'V' : '-',
	    (f->P & P_U) ? 'U' : '-',
	    (f->P & P_B) ? 'B' : '-',
	    (f->P & P_D) ? 'D' : '-',
	    (f->P & P_I) ? 'I' : '-',
	    (f->P & P_Z) ? 'Z' : '-',
	    (f->P & P_C) ? 'C' : '-'
	);

	if (o) {
		cpu_decodeop(c, o);
	}

#if 0
	printf("    ZERO PAGE:");
	for (i = 0; i < 16; i++)
		printf(" %02X", CPU_READ8(c, i));
	printf(" ...\n");
#endif
}

int
cpu_init(struct cpu_context *c)
{
	struct cpu_frame *f = &c->frame;

	c->insns = 0;
	c->ticks = 0;
	c->delay = 0;
	c->fetch = 1;
	c->debug = 0;

	if (getenv("AVNES_CPU_DEBUG") && *getenv("AVNES_CPU_DEBUG") == '1')
		c->debug = 1;

	memset(f, 0, sizeof(*f));

	f->PC = CPU_READ16(c, RESET_VECTOR_L);
	f->P = P_U | P_I | P_B;
	f->SP = 0;

	return 0;
}

void
cpu_step(struct cpu_context *c)
{
	struct cpu_frame *f = &c->frame;
	struct cpu_opcode *o;
	uint8_t opcode;
	int cycles;

	++c->ticks;

	if (c->fetch) {
		opcode = CPU_READ8(c, f->PC);

		o = &cpu_opcodes[opcode];
		if (o->op == NULL) {
			printf("PANIC: unknown opcode $%02X\n", opcode);
			cpu_dumpop(c, NULL);
			abort();
		}

		c->delay += o->cycles;
		c->fetch = 0;
	}

	if (--c->delay == 0) {
		++c->insns;
		opcode = CPU_READ8(c, f->PC);
		o = &cpu_opcodes[opcode];
		if (c->debug)
			cpu_dumpop(c, o);
		c->delay = o->op(c, o) - o->cycles;
		c->fetch = 1;
	}
}

void
cpu_nmi(struct cpu_context *c)
{
	struct cpu_frame *f = &c->frame;

	/* Push PC and P to stack */
	cpu_stackpush(c, f->PC >> 8);
	cpu_stackpush(c, f->PC & 0xff);
	cpu_stackpush(c, f->P);

	//printf("NMI PC=$%04X SP=%d\n", f->PC, f->SP);

	/* Load NMI vector into PC */
	f->PC = CPU_READ16(c, NMI_VECTOR_L);

	c->delay = 0;
	c->fetch = 1;
}

void
cpu_irq(struct cpu_context *c)
{
	struct cpu_frame *f = &c->frame;

	if (f->P & P_I)
		return;	/* interrupts disabled */

	/* Push PC and P to stack */
	cpu_stackpush(c, f->PC >> 8);
	cpu_stackpush(c, f->PC & 0xff);
	cpu_stackpush(c, f->P);

	//printf("IRQ PC=$%04X SP=%d\n", f->PC, f->SP);

	/* Load IRQ vector into PC */
	f->PC = CPU_READ16(c, INTR_VECTOR_L);

	c->delay = 0;
	c->fetch = 1;
}
