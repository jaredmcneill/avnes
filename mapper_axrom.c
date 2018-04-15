/*-
 * Copyright (c) 2018 Jared McNeill <jmcneill@invisible.ca>
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

#define	AXROM_BANKSEL_PRG	0x07
#define	AXROM_BANKSEL_VRAM	0x10

struct axrom_context {
	uint8_t chr_ram[CHR_RAM_SIZE];

	uint8_t banksel;
};

static uint8_t
axrom_cpuread(struct avnes_context *av, uint16_t addr)
{
	struct axrom_context *ac = av->m.priv;

#ifdef AXROM_DEBUG
	printf("[%s] addr=$%04X\n", __func__, addr);
#endif

	if (addr >= 0x8000 && addr <= 0xFFFF) {
		const int bank = ac->banksel & AXROM_BANKSEL_PRG;
		return av->rom_data[av->prg_start + (addr & 0x7FFF) + (bank * 0x8000)];
	}

	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
	return 0;
}

static void
axrom_cpuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	struct axrom_context *ac = av->m.priv;

	if (addr >= 0x8000 && addr <= 0xFFFF) {
		ac->banksel = val;
		return;
	}

	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
}

static uint8_t
axrom_ppuread(struct avnes_context *av, uint16_t addr)
{
	struct axrom_context *ac = av->m.priv;

#ifdef AXROM_DEBUG
	printf("[%s] addr=$%04X\n", __func__, addr);
#endif

	if (addr >= 0x0000 && addr <= 0x1FFF) {
		return ac->chr_ram[addr];
	}

	if (addr >= 0x2000 && addr <= 0x3EFF) {
		/* Nametable */
		if (addr >= 0x3000)
			addr -= 0x1000;

		off_t off;
		if (ac->banksel & AXROM_BANKSEL_VRAM)
			off = (addr & 0x3FF) + 0x0400;
		else
			off = (addr & 0x3FF) + 0x0000;

		return av->vram[off];
	}

	printf("[%s] PPU address $%04X not mapped\n", __func__, addr);
	return 0;
}

static void
axrom_ppuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	struct axrom_context *ac = av->m.priv;

#ifdef AXROM_DEBUG
	printf("[%s] addr=$%04X val=$%02X\n", __func__, addr, val);
#endif

	if (addr >= 0x0000 && addr <= 0x1FFF) {
		ac->chr_ram[addr] = val;
		return;
	}

	if (addr >= 0x2000 && addr <= 0x3EFF) {
		/* Nametable */
		if (addr >= 0x3000)
			addr -= 0x1000;

		off_t off;
		if (ac->banksel & AXROM_BANKSEL_VRAM)
			off = (addr & 0x3FF) + 0x0400;
		else
			off = (addr & 0x3FF) + 0x0000;

		av->vram[off] = val;
		return;
	}

	printf("[%s] PPU address $%04X not mapped\n", __func__, addr);
}

static int
axrom_init(struct mapper_context *m, const uint8_t *data, size_t datalen)
{
	struct axrom_context *ac;

	ac = calloc(1, sizeof(*ac));
	if (ac == NULL)
		return ENOMEM;

	ac->banksel = 0;

	m->cpuread = axrom_cpuread;
	m->cpuwrite = axrom_cpuwrite;
	m->ppuread = axrom_ppuread;
	m->ppuwrite = axrom_ppuwrite;
	m->priv = ac;

	return 0;
}

MAPPER_DECL(axrom, "AxROM", axrom_init);
