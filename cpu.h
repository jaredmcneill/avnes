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

#ifndef _CPU_H
#define _CPU_H

struct cpu_frame {
	uint16_t	PC;	/* Program Counter */
	uint8_t		SP;	/* Stack Pointer */
	uint8_t		A;	/* Accumulator */
	uint8_t		X;	/* Index Register X */
	uint8_t		Y;	/* Index Register Y */
	uint8_t		P;	/* Processor Status */
#define		P_C	(1 << 0)	/* Carry Flag */
#define		P_Z	(1 << 1)	/* Zero Flag */
#define		P_I	(1 << 2)	/* Interrupt Disable */
#define		P_D	(1 << 3)	/* Decimal Mode */
#define		P_B	(1 << 4)	/* Break Command */
#define		P_U	(1 << 5)	/* Unused */
#define		P_V	(1 << 6)	/* Overflow Flag */
#define		P_N	(1 << 7)	/* Negative Flag */
};

struct cpu_context {
	struct cpu_frame frame;

	uint8_t (*read8)(uint16_t);
	void (*write8)(uint16_t, uint8_t);

	uint64_t insns;
	uint64_t ticks;

	int delay, fetch;
};

int	cpu_init(struct cpu_context *);
void	cpu_step(struct cpu_context *);
void	cpu_nmi(struct cpu_context *);
void	cpu_irq(struct cpu_context *);

#endif /* !_CPU_H */
