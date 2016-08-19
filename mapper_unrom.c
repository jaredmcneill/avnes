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

#include "avnes.h"
#include "mapper.h"
#include "rom.h"

#define	CHR_RAM_SIZE	0x2000

struct unrom_context {
	uint8_t mirror;
	uint32_t banksel;	/* PRG bank */

	uint8_t chr_ram[CHR_RAM_SIZE];
};

static uint8_t
unrom_cpuread(struct avnes_context *av, uint16_t addr)
{
	struct unrom_context *mc = av->m.priv;

#ifdef UNROM_DEBUG
	printf("[%s] addr=$%04X\n", __func__, addr);
#endif

	if (addr >= 0x8000 && addr <= 0xBFFF) {
		/* 16KB switchable PRG ROM bank */
		return av->rom_data[av->prg_start + (addr - 0x8000) + (mc->banksel * 0x4000)];
	}

	if (addr >= 0xC000 && addr <= 0xFFFF) {
		/* 16KB PRG ROM bank, fixed to the last bank */
		return av->rom_data[av->prg_start + (addr - 0xC000) + (av->prg_len - 0x4000)];
	}

	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
	return 0;
}

static void
unrom_cpuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	struct unrom_context *mc = av->m.priv;

#ifdef UNROM_DEBUG
	printf("[%s] addr=$%04X val=$%02X\n", __func__, addr, val);
#endif

	if (addr >= 0x8000 && addr <= 0xFFFF) {
		/* Bank select */
		mc->banksel = val;
		return;
	}

	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
}

static uint8_t
unrom_ppuread(struct avnes_context *av, uint16_t addr)
{
	struct unrom_context *mc = av->m.priv;
	uint8_t val;

#ifdef UNROM_DEBUG
	printf("[%s] addr=$%04X\n", __func__, addr);
#endif

	if (addr >= 0x0000 && addr <= 0x1FFF) {
		if (av->chr_len)
			return av->rom_data[av->chr_start + addr];
		else
			return mc->chr_ram[addr];
	}

	if (addr >= 0x2000 && addr <= 0x3EFF) {
		/* Nametable */
		if (addr >= 0x3000)
			addr -= 0x1000;

		off_t off;
		switch (mc->mirror) {
		case ROM_MIRROR_H:
			off = (addr & 0x3FF) + (addr < 0x2800 ? 0 : 0x400);
			break;
		case ROM_MIRROR_V:
			off = (addr & 0x7FF);
			break;
		default:
			assert("Unsupported mirror mode" == NULL);
		}
		return av->vram[off];
	}

	printf("[%s] PPU address $%04X not mapped\n", __func__, addr);
	return 0;
}

static void
unrom_ppuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	struct unrom_context *mc = av->m.priv;

#ifdef UNROM_DEBUG
	printf("[%s] addr=$%04X val=$%02X\n", __func__, addr, val);
#endif

	if (av->chr_len == 0 && addr >= 0x0000 && addr <= 0x1FFF) {
		mc->chr_ram[addr] = val;
		return;
	}

	if (addr >= 0x2000 && addr <= 0x3EFF) {
		/* Nametable */
		if (addr >= 0x3000)
			addr -= 0x1000;

		off_t off;
		switch (mc->mirror) {
		case ROM_MIRROR_H:
			off = (addr & 0x3FF) + (addr < 0x2800 ? 0 : 0x400);
			break;
		case ROM_MIRROR_V:
			off = (addr & 0x7FF);
			break;
		default:
			assert("Unsupported mirror mode" == NULL);
		}
		av->vram[off] = val;
		return;
	}

	printf("[%s] PPU address $%04X not mapped\n", __func__, addr);
}

static int
unrom_init(struct mapper_context *m, const uint8_t *data, size_t datalen)
{
	struct unrom_context *mc;

	mc = calloc(1, sizeof(*mc));
	if (mc == NULL)
		return ENOMEM;

	mc->mirror = data[6] & ROM_MIRROR_MASK;

	m->cpuread = unrom_cpuread;
	m->cpuwrite = unrom_cpuwrite;
	m->ppuread = unrom_ppuread;
	m->ppuwrite = unrom_ppuwrite;
	m->priv = mc;

	return 0;
}

MAPPER_DECL(unrom, "UNROM", unrom_init);
