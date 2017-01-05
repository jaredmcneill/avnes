/*-
 * Copyright (c) 2017 Jared McNeill <jmcneill@invisible.ca>
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

//#define APU_DEBUG

#include <stdio.h>

#include "avnes.h"
#include "apu.h"

#define	APU_CYCLE(i,f)	(((i) << 1) + (f == 5 ? 1 : 0))

/* Pulse 1 channel (write) */
#define	REG_PULSE1_DDLCNNNN	0x4000
#define	REG_PULSE1_EPPPNSSS	0x4001
#define	REG_PULSE1_LLLLLLLL	0x4002
#define	REG_PULSE1_LLLLLHHH	0x4003

/* Pulse 2 channel (write) */
#define	REG_PULSE2_DDLCNNNN	0x4004
#define	REG_PULSE2_EPPPNSSS	0x4005
#define	REG_PULSE2_LLLLLLLL	0x4006
#define	REG_PULSE2_LLLLLHHH	0x4007

/* Triangle channel (write) */
#define	REG_TRIANGLE_CRRRRRRR	0x4008
#define	REG_TRIANGLE_LLLLLLLL	0x400a
#define	REG_TRIANGLE_LLLLLHHH	0x400b

/* Status */
#define	REG_STATUS		0x4015

/* Frame counter */
#define	REG_FRAME_COUNTER	0x4017

/* APU pulse waveform sequences */
static uint8_t
apu_pulse_sequence[4][8] = {
	[0] = { 0, 1, 0, 0, 0, 0, 0, 0 },	/* 12.5% */
	[1] = { 0, 1, 1, 0, 0, 0, 0, 0 },	/* 25% */
	[2] = { 0, 1, 1, 1, 1, 0, 0, 0 },	/* 50% */
	[3] = { 1, 0, 0, 1, 1, 1, 1, 1 },	/* 75% (25% negated) */
};

/* APU triangle waveform sequence */
static uint8_t
apu_triangle_sequence[32] = {
	15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
	 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
};

/* APU length counter table */
static uint8_t
apu_length_counter[32] = {
	10,254, 20,  2, 40,  4, 80,  6, 160,  8, 60, 10, 14, 12, 26, 14,
	12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30
};

int
apu_init(struct apu_context *a)
{

	return 0;
}

uint8_t
apu_read(struct apu_context *a, uint16_t addr)
{
	uint8_t val;

	switch (addr) {
	case REG_STATUS:
		val = 0;
		if (a->status.frame_interrupt) {
			/* Reading status clears frame interrupt status */
			val |= 0x40;
			a->status.frame_interrupt = 0;
		}
		if (a->status.pulse_enable[0] && a->pulse[0].length_counter > 0)
			val |= 0x01;
		if (a->status.pulse_enable[1] && a->pulse[1].length_counter > 0)
			val |= 0x02;
		if (a->status.triangle_enable && a->triangle.length_counter > 0)
			val |= 0x04;
#ifdef APU_DEBUG
		printf("[%s] status = $%02X\n", __func__, val);
#endif
		break;
	default:
#ifdef APU_DEBUG
		printf("TODO: %s addr=$%04X\n", __func__, addr);
#endif
		val = 0x00;
		break;
	}

	return val;
}

void
apu_write(struct apu_context *a, uint16_t addr, uint8_t val)
{
	struct apu_pulse *ap;
	struct apu_triangle *at;

	switch (addr) {
	case REG_PULSE1_DDLCNNNN:
	case REG_PULSE2_DDLCNNNN:
		ap = addr == REG_PULSE1_DDLCNNNN ? &a->pulse[0] : &a->pulse[1];

		ap->duty_cycle = (val & 0xc0) >> 6;
		ap->length_counter_halt = (val & 0x20) ? 1 : 0;
		ap->constant_vol_env_flag = (val & 0x10) ? 1 : 0;
		ap->vol_env_div_period = val & 0xf;

		if (ap->length_counter_halt) {
			ap->length_counter = apu_length_counter[ap->length];
		}

		break;

	case REG_PULSE1_EPPPNSSS:
	case REG_PULSE2_EPPPNSSS:
		ap = addr == REG_PULSE1_EPPPNSSS ? &a->pulse[0] : &a->pulse[1];

		ap->sweep_enabled = (val & 0x80) ? 1 : 0;
		ap->sweep_div_period = (val & 0x70) >> 4;
		ap->sweep_negate = (val & 0x08) ? 1 : 0;
		ap->sweep_shift_count = val & 0x7;

		break;

	case REG_PULSE1_LLLLLLLL:
	case REG_PULSE2_LLLLLLLL:
		ap = addr == REG_PULSE1_LLLLLLLL ? &a->pulse[0] : &a->pulse[1];

		ap->timer &= ~0xff;
		ap->timer |= val;

		break;

	case REG_PULSE1_LLLLLHHH:
	case REG_PULSE2_LLLLLHHH:
		ap = addr == REG_PULSE1_LLLLLHHH ? &a->pulse[0] : &a->pulse[1];

		ap->timer &= ~(0x7 << 8);
		ap->timer |= ((uint16_t)val & 0x7) << 8;
		ap->length = (val & 0xf8) >> 3;
		ap->length_counter = apu_length_counter[ap->length];

		ap->timer_counter = ap->timer;

		break;

	case REG_TRIANGLE_CRRRRRRR:
		at = &a->triangle;

		at->length_counter_halt = (val & 0x80) ? 1 : 0;
		at->counter_reload = (val & 0x7f);

		break;

	case REG_TRIANGLE_LLLLLLLL:
		at = &a->triangle;

		at->timer &= ~0xff;
		at->timer |= val;

		break;

	case REG_TRIANGLE_LLLLLHHH:
		at = &a->triangle;

		at->timer &= ~(0x7 << 8);
		at->timer |= ((uint16_t)val & 0x7) << 8;
		at->length = (val & 0xf8) >> 3;

		at->linear_counter_reload = 1;
		at->timer_counter = at->timer;

		break;

	case REG_STATUS:
		a->status.pulse_enable[0] = (val & 0x01) ? 1 : 0;
		a->status.pulse_enable[1] = (val & 0x02) ? 1 : 0;
		a->status.triangle_enable = (val & 0x04) ? 1 : 0;
		a->status.noise_enable = (val & 0x08) ? 1 : 0;
		if (a->status.pulse_enable[0] == 0)
			a->pulse[0].length_counter = 0;
		if (a->status.pulse_enable[1] == 0)
			a->pulse[1].length_counter = 0;
		if (a->status.triangle_enable == 0)
			a->triangle.length_counter = 0;
		break;

	case REG_FRAME_COUNTER:
		/* Sequencer mode (0: 4-step, 1: 5-step) */
		a->counter.mode = (val & 0x80) ? 1 : 0;
		/* Interrupt inhibit flag */
		a->counter.interrupt_inhibit = (val & 0x40) ? 1 : 0;
		/* Reset timer */
		a->cycle = 0;
		/* Clear frame interrupt flag if inhibit flag is set */
		if (a->counter.interrupt_inhibit)
			a->status.frame_interrupt = 0;
#ifdef APU_DEBUG
		printf("[%s] val=$%02X mode=%d interrupt_inhibit=%d (frame_interrupt=%d)\n", __func__,
		    val, a->counter.mode, a->counter.interrupt_inhibit, a->status.frame_interrupt);
#endif
		break;
	default:
#ifdef APU_DEBUG
		printf("TODO: %s addr=$%04X val=$%02X\n", __func__, addr, val);
#endif
		break;
	}
}

static void
apu_pulse_step(struct apu_context *a, struct apu_pulse *ap)
{
	if (ap->timer_counter > 0) {
		--ap->timer_counter;
		return;
	}

	ap->seqval = apu_pulse_sequence[ap->duty_cycle][ap->seqno];

	/* Increment sequencer step number */
	ap->seqno = (ap->seqno + 1) & 0x7;

	ap->timer_counter = ap->timer;
}

static void
apu_triangle_step(struct apu_context *a, struct apu_triangle *at)
{
	if (at->timer_counter > 0) {
		--at->timer_counter;
		return;
	}

	at->seqval = apu_triangle_sequence[at->seqno];

	/* Increment sequencer step number */
	at->seqno = (at->seqno + 1) & 0x1f;

	at->timer_counter = at->timer;
}

int
apu_step(struct apu_context *a)
{
	const int last_cycle = a->counter.mode ? APU_CYCLE(18641,0) : APU_CYCLE(14914,0);
	const int half_cycle = a->cycle & 1;
	int irq = 0, length_counter = 0, linear_counter = 0;

	/* Pulse channel timers run at APU (CPU/2) rate */
	if (!half_cycle) {
		if (a->status.pulse_enable[0])
			apu_pulse_step(a, &a->pulse[0]);
		if (a->status.pulse_enable[1])
			apu_pulse_step(a, &a->pulse[1]);
	}
	/* Triangle channel timer runs at CPU rate */
	if (a->status.triangle_enable)
		apu_triangle_step(a, &a->triangle);

	switch (a->cycle) {
	case APU_CYCLE(3728,5):
		linear_counter = 1;
		break;
	case APU_CYCLE(7456,5):
		linear_counter = 1;
		length_counter = 1;
		break;
	case APU_CYCLE(11185,5):
		linear_counter = 1;
		break;
	case APU_CYCLE(14914,0):
		irq = 1;
		break;
	case APU_CYCLE(14914,5):
		irq = 1;
		if (a->counter.mode == 0) {
			length_counter = 1;
			linear_counter = 1;
		}
		break;
	case APU_CYCLE(18640,5):
		if (a->counter.mode == 1) {
			length_counter = 1;
			linear_counter = 1;
		}
		break;
	}

	a->cycle++;
	if (a->cycle == last_cycle) {
		a->cycle = 0;
		irq = 1;
	}

	if (length_counter) {
		if (a->status.pulse_enable[0] && !a->pulse[0].length_counter_halt) {
			a->pulse[0].length_counter--;
			if (a->pulse[0].length_counter == 0) {
				a->status.pulse_enable[0] = 0;
				a->pulse[0].seqval = 0;
			}
		}
		if (a->status.pulse_enable[1] && !a->pulse[1].length_counter_halt) {
			a->pulse[1].length_counter--;
			if (a->pulse[1].length_counter == 0) {
				a->status.pulse_enable[1] = 0;
				a->pulse[1].seqval = 0;
			}
		}
		if (a->status.triangle_enable && !a->triangle.length_counter_halt) {
			a->triangle.length_counter--;
			if (a->triangle.length_counter == 0) {
				a->status.triangle_enable = 0;
				a->triangle.length_counter = a->triangle.linear_counter = 0;
				a->triangle.seqval = 0;
			}
		}
	}
	if (linear_counter) {
		if (a->status.triangle_enable) {
			if (a->triangle.linear_counter_reload) {
				a->triangle.linear_counter = a->triangle.counter_reload;
			} else if (a->triangle.linear_counter > 0) {
				a->triangle.linear_counter--;
			}
			if (!a->triangle.length_counter_halt) {
				a->triangle.linear_counter_reload = 0;
			}
			if (a->triangle.linear_counter == 0) {
				a->status.triangle_enable = 0;
				a->triangle.seqval = 0;
			}
		}
	}

	if (a->play)
		a->play(a);

	if (irq && a->counter.mode == 0 && a->counter.interrupt_inhibit == 0) {
		a->status.frame_interrupt = 1;
		//printf("[%s]: frame interrupt\n", __func__);
		cpu_irq(a->c);
	}

	return 0;
}
