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

struct gxrom_context {
	uint8_t mirror;
	unsigned int chr_bank;
	unsigned int prg_bank;
};

static uint8_t
gxrom_cpuread(struct avnes_context *av, uint16_t addr)
{
	struct gxrom_context *gc = av->m.priv;

#ifdef MMC1_DEBUG
	printf("[%s] addr=$%04X\n", __func__, addr);
#endif

	if (addr >= 0x8000 && addr <= 0xFFFF) {
		return av->rom_data[av->prg_start + (addr & 0x7FFF) + (gc->prg_bank * 0x8000)];
	}

	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
	return 0;
}

static void
gxrom_cpuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	struct gxrom_context *gc = av->m.priv;

#ifdef MMC1_DEBUG
	printf("[%s] addr=$%04X val=$%02X\n", __func__, addr, val);
#endif

	if (addr >= 0x8000 && addr <= 0xFFFF) {
		gc->chr_bank = (val & 0x03) >> 0;
		gc->prg_bank = (val & 0x30) >> 4;
		return;
	}

	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
}

static uint8_t
gxrom_ppuread(struct avnes_context *av, uint16_t addr)
{
	struct gxrom_context *gc = av->m.priv;

#ifdef MMC1_DEBUG
	printf("[%s] addr=$%04X\n", __func__, addr);
#endif

	if (addr >= 0x0000 && addr <= 0x1FFF) {
		return av->rom_data[av->chr_start + (addr & 0x1FFF) + (gc->chr_bank * 0x2000)];
	}

	if (addr >= 0x2000 && addr <= 0x3EFF) {
		/* Nametable */
		if (addr >= 0x3000)
			addr -= 0x1000;

		off_t off;
		switch (gc->mirror) {
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
gxrom_ppuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	struct gxrom_context *gc = av->m.priv;

#ifdef MMC1_DEBUG
	printf("[%s] addr=$%04X val=$%02X\n", __func__, addr, val);
#endif

	if (addr >= 0x2000 && addr <= 0x3EFF) {
		/* Nametable */
		if (addr >= 0x3000)
			addr -= 0x1000;

		off_t off;
		switch (gc->mirror) {
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
gxrom_init(struct mapper_context *m, const uint8_t *data, size_t datalen)
{
	struct gxrom_context *gc;

	gc = calloc(1, sizeof(*gc));
	if (gc == NULL)
		return ENOMEM;

	gc->mirror = data[6] & ROM_MIRROR_MASK;

	m->cpuread = gxrom_cpuread;
	m->cpuwrite = gxrom_cpuwrite;
	m->ppuread = gxrom_ppuread;
	m->ppuwrite = gxrom_ppuwrite;
	m->priv = gc;

	return 0;
}

MAPPER_DECL(gxrom, "GxROM", gxrom_init);
