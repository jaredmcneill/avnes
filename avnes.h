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

#ifndef _AVNES_H
#define _AVNES_H

#include <stdint.h>

#include "cpu.h"
#include "ppu.h"
#include "io.h"
#include "mapper.h"

#ifndef nitems
#define nitems(x)	(sizeof((x)) / sizeof((x)[0]))
#endif

#define	AVNES_RAM_SIZE		0x800	/* 2KB internal RAM */
#define	AVNES_VRAM_SIZE		0x800	/* 2KB video RAM */
#define	AVNES_PALETTERAM_SIZE	0x20	/* 32B palette RAM */

struct avnes_context {
	struct cpu_context c;
	struct ppu_context p;
	struct mapper_context m;
	struct io_context io;

	uint8_t *rom_data;
	size_t rom_datalen;

	off_t prg_start;
	size_t prg_len;
	off_t chr_start;
	size_t chr_len;

	uint8_t ram[AVNES_RAM_SIZE];
	uint8_t vram[AVNES_VRAM_SIZE];
	uint8_t paletteram[AVNES_PALETTERAM_SIZE];
};

#endif /* !_AVNES_H */
