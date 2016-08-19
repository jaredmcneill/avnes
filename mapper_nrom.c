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

#define	PRG_RAM_SIZE	0x2000

struct nrom_context {
	uint8_t mirror;

	uint8_t prg_ram[PRG_RAM_SIZE];
};

static uint8_t
nrom_cpuread(struct avnes_context *av, uint16_t addr)
{
	if (addr >= 0x8000 && addr <= 0xFFFF)
		return av->rom_data[av->prg_start + ((addr - 0x8000) & (av->prg_len - 1))];

	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
	return 0;
}

static void
nrom_cpuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
}

static uint8_t
nrom_ppuread(struct avnes_context *av, uint16_t addr)
{
	struct nrom_context *mc = av->m.priv;

	if (addr >= 0x0000 && addr <= 0x1FFF) {
		/* Pattern table */
		if (av->chr_len) {
			return av->rom_data[av->chr_start + (addr & (av->chr_len - 1))];
		} else {
			return mc->prg_ram[addr];
		}
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
nrom_ppuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	struct nrom_context *mc = av->m.priv;

	if (av->chr_len == 0 && addr >= 0x0000 && addr <= 0x1FFF) {
		mc->prg_ram[addr] = val;
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
nrom_init(struct mapper_context *m, const uint8_t *data, size_t datalen)
{
	struct nrom_context *mc;

	mc = calloc(1, sizeof(*mc));
	if (mc == NULL)
		return ENOMEM;

	mc->mirror = data[6] & ROM_MIRROR_MASK;

	m->cpuread = nrom_cpuread;
	m->cpuwrite = nrom_cpuwrite;
	m->ppuread = nrom_ppuread;
	m->ppuwrite = nrom_ppuwrite;
	m->priv = mc;

	return 0;
}

MAPPER_DECL(nrom, "NROM", nrom_init);
