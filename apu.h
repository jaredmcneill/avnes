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

#ifndef _APU_H
#define _APU_H

#include "cpu.h"

struct apu_frame_counter {
	int			mode;
	int			interrupt_inhibit;
};

struct apu_status {
	int			dmc_interrupt;
	int			frame_interrupt;
	int			dmc_active;

	int			noise_enable;
	int			triangle_enable;
	int			pulse_enable[2];
	/* TODO: length counters */
};

struct apu_pulse {
	uint8_t			duty_cycle : 2;
	uint8_t			length_counter_halt : 1;
	uint8_t			constant_vol_env_flag : 1;
	uint8_t			vol_env_div_period : 4;

	uint8_t			sweep_enabled : 1;
	uint8_t			sweep_div_period : 3;
	uint8_t			sweep_negate : 1;
	uint8_t			sweep_shift_count : 3;

	uint16_t		timer : 11;
	uint16_t		timer_counter : 11;

	uint8_t			length : 5;
	uint8_t			length_counter : 5;

	/* Sequencer state */
	uint8_t			seqno;
	uint8_t			seqval;
};

struct apu_triangle {
	uint8_t			length_counter_halt : 1;
	uint8_t			counter_reload : 7;

	uint16_t		timer : 11;
	uint16_t		timer_counter : 11;

	uint8_t			length : 5;
	uint8_t			length_counter : 5;

	uint8_t			linear_counter : 7;

	uint8_t			linear_counter_reload : 1;

	/* Sequencer state */
	uint8_t			seqno;
	uint8_t			seqval;
};

struct apu_noise {
	uint8_t			length_counter_halt : 1;
	uint8_t			constant_vol_env_flag : 1;
	uint8_t			vol_env_div_period : 1;

	uint8_t			mode : 1;
	uint8_t			noise_period : 4;

	uint16_t		timer : 11;
	uint16_t		timer_counter : 11;

	uint8_t			length : 5;
	uint8_t			length_counter : 5;

	/* Sequencer state */
	uint8_t			seqval;
};

struct apu_dmc {
	uint8_t			irq_enable : 1;
	uint8_t			loop_flag : 1;
	uint8_t			rate_index : 4;

	uint16_t		sample_address : 16;
	uint16_t		sample_length : 12;

	uint16_t		timer : 11;
	uint16_t		timer_counter : 11;

	/* Sequencer state */
	uint8_t			cur_sample;
	uint8_t			cur_sample_bits;
	uint16_t		cur_sample_address;
	uint16_t		cur_sample_length;
	uint8_t			seqval;
	int			silence;
};

struct apu_context {
	struct cpu_context	*c;

	struct apu_status	status;
	struct apu_frame_counter counter;

	struct apu_pulse	pulse[2];
	struct apu_triangle	triangle;
	struct apu_noise	noise;
	struct apu_dmc		dmc;

	uint16_t		noise_shift_reg;

	int			cycle;

	uint8_t (*read8)(uint16_t);
	void (*write8)(uint16_t, uint8_t);
	void (*play)(struct apu_context *);
};

int	apu_init(struct apu_context *);
uint8_t	apu_read(struct apu_context *, uint16_t);
void	apu_write(struct apu_context *, uint16_t, uint8_t);
int	apu_step(struct apu_context *);

#endif /* !_APU_H */
