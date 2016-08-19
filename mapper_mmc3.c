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

struct mmc3_context {
	uint8_t prg_ram[PRG_RAM_SIZE];

	uint8_t bank[8];
	int irq_enable;
	int last_scanline;
	uint8_t irq_counter;

	uint8_t banksel;	/* Bank select */
	uint8_t mirror;		/* Mirroring */
	uint8_t prg_ram_prot;	/* PRG RAM protect */
	uint8_t	irq_latch;	/* IRQ latch */
};

static uint32_t
mmc3_cpuaddr2romoffset(struct avnes_context *av, uint16_t addr)
{
	struct mmc3_context *mc = av->m.priv;

	if (addr >= 0xE000 && addr <= 0xFFFF) {
		/* 8KB PRG ROM bank, fixed to the last bank */
		return (addr - 0xE000) + (av->prg_len - 0x2000);
	}

	if (addr >= 0xA000 && addr <= 0xBFFF) {
		/* 8KB switchable PRG ROM bank */
		return (addr - 0xA000) + (mc->bank[7] * 0x2000);
	}

	if (((mc->banksel >> 6) & 1) == 0) {
		/* $8000-$9FFF swappable, $C000-$DFFF fixed to second-last bank */
		if (addr >= 0x8000 && addr <= 0x9FFF) {
			/* 8KB switchable PRG ROM bank */
			return (addr - 0x8000) + (mc->bank[6] * 0x2000);
		} else {
			/* 8KB PRG ROM bank, fixed to the second-last last bank */
			return (addr - 0xC000) + (av->prg_len - 0x4000);
		}
	} else {
		/* $C000-$DFFF swappable, $8000-$9FFF fixed to second-last bank */
		if (addr >= 0xC000 && addr <= 0xEFFF) {
			/* 8KB switchable PRG ROM bank */
			return (addr - 0xC000) + (mc->bank[6] * 0x2000);
		} else {
			/* 8KB PRG ROM bank, fixed to the second-last last bank */
			return (addr - 0x8000) + (av->prg_len - 0x4000);
		}
	}
}

static uint8_t
mmc3_cpuread(struct avnes_context *av, uint16_t addr)
{
	struct mmc3_context *mc = av->m.priv;

#ifdef MMC3_DEBUG
	printf("[%s] addr=$%04X\n", __func__, addr);
#endif

	if (addr >= 0x6000 && addr <= 0x7FFF) {
		/* 8KB PRG RAM bank */
		return mc->prg_ram[addr - 0x6000];
	}

	if (addr >= 0x8000 && addr <= 0xFFFF) {
		return av->rom_data[av->prg_start + mmc3_cpuaddr2romoffset(av, addr)];
	}

	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
	return 0;
}

static void
mmc3_cpuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	struct mmc3_context *mc = av->m.priv;

#ifdef MMC3_DEBUG
	printf("[%s] addr=$%04X val=$%02X\n", __func__, addr, val);
#endif

	assert(addr >= 0x6000 && addr <= 0xFFFF);

	if (addr >= 0x6000 && addr <= 0x7FFF) {
		/* 8KB PRG RAM bank */
		mc->prg_ram[addr - 0x6000] = val;
		return;
	}

	if (addr >= 0x8000 && addr <= 0x9FFF) {
		if ((addr & 1) == 0) {
			/* Bank select */
			mc->banksel = val;
#ifdef MMC3_DEBUG
			printf("[%s] select bank %d (%02X)\n", __func__, mc->banksel & 0x7, mc->banksel);
#endif
		} else {
			/* Bank data */
			int bank = mc->banksel & 0x7;

			if (bank == 0 || bank == 1)
				val &= 0xfe;
			else if (bank == 6 || bank == 7)
				val &= 0x3f;

			mc->bank[bank] = val;
#ifdef MMC3_DEBUG
			printf("[%s] bank %d data %02X\n", __func__, bank, val);
#endif
		}
		return;
	}

	if (addr >= 0xA000 && addr <= 0xBFFF) {
		if ((addr & 1) == 0) {
			/* Mirroring */
			mc->mirror = val;
		} else {
			/* PRG RAM protect */
			mc->prg_ram_prot = val;
		}
		return;
	}

	if (addr >= 0xC000 && addr <= 0xDFFF) {
		if ((addr & 1) == 0) {
			/* IRQ latch */
			mc->irq_latch = val;
		} else {
			/* IRQ reload */
			mc->irq_counter = mc->irq_latch;
		}
		return;
	}

	if (addr >= 0xE000 && addr <= 0xFFFF) {
		if ((addr & 1) == 0) {
			/* IRQ disable */
			mc->irq_enable = 0;
			mc->irq_counter = 0;
		} else {
			/* IRQ enable */
			mc->irq_enable = 1;
		}
#ifdef MMC3_DEBUG
		printf("[%s] IRQ %s\n", __func__, mc->irq_enable ? "enabled" : "disabled");
#endif
		return;
	}

	printf("[%s] CPU address $%04X not mapped\n", __func__, addr);
}

static uint32_t
mmc3_ppuaddr2romoffset(struct avnes_context *av, uint16_t addr)
{
	struct mmc3_context *mc = av->m.priv;

	if (((mc->banksel >> 7) & 1) == 0) {
		/* Two 2KB banks at $0000-$0FFF, four 1KB banks at $1000-$1FFF */
		if (addr >= 0x0000 && addr <= 0x07FF) {
			/* 2KB switchable CHR ROM bank */
			return (addr - 0x0000) + (mc->bank[0] * 0x400);
		}
		if (addr >= 0x0800 && addr <= 0x0FFF) {
			/* 2KB switchable CHR ROM bank */
			return (addr - 0x0800) + (mc->bank[1] * 0x400);
		}
		if (addr >= 0x1000 && addr <= 0x13FF) {
			/* 1KB switchable CHR ROM bank */
			return (addr - 0x1000) + (mc->bank[2] * 0x400);
		}
		if (addr >= 0x1400 && addr <= 0x17FF) {
			/* 1KB switchable CHR ROM bank */
			return (addr - 0x1400) + (mc->bank[3] * 0x400);
		}
		if (addr >= 0x1800 && addr <= 0x1BFF) {
			/* 1KB switchable CHR ROM bank */
			return (addr - 0x1800) + (mc->bank[4] * 0x400);
		}
		if (addr >= 0x1C00 && addr <= 0x1FFF) {
			/* 1KB switchable CHR ROM bank */
			return (addr - 0x1C00) + (mc->bank[5] * 0x400);
		}
	} else {
		/* Two 2KB banks at $1000-$1FFF, four 1KB banks at $0000-$1FFF */
		if (addr >= 0x1000 && addr <= 0x17FF) {
			/* 2KB switchable CHR ROM bank */
			return (addr - 0x1000) + (mc->bank[0] * 0x400);
		}
		if (addr >= 0x1800 && addr <= 0x1FFF) {
			/* 2KB switchable CHR ROM bank */
			return (addr - 0x1800) + (mc->bank[1] * 0x400);
		}
		if (addr >= 0x0000 && addr <= 0x03FF) {
			/* 1KB switchable CHR ROM bank */
			return (addr - 0x0000) + (mc->bank[2] * 0x400);
		}
		if (addr >= 0x0400 && addr <= 0x07FF) {
			/* 1KB switchable CHR ROM bank */
			return (addr - 0x0400) + (mc->bank[3] * 0x400);
		}
		if (addr >= 0x0800 && addr <= 0x0BFF) {
			/* 1KB switchable CHR ROM bank */
			return (addr - 0x0800) + (mc->bank[4] * 0x400);
		}
		if (addr >= 0x0C00 && addr <= 0x0FFF) {
			/* 1KB switchable CHR ROM bank */
			return (addr - 0x0C00) + (mc->bank[5] * 0x400);
		}
	}

	printf("[%s] PPU address %04X not mapped\n", __func__, addr);
	abort();
}

static uint8_t
mmc3_ppuread(struct avnes_context *av, uint16_t addr)
{
	struct mmc3_context *mc = av->m.priv;
	uint8_t val;

#ifdef MMC3_DEBUG
	printf("[%s] addr=$%04X\n", __func__, addr);
#endif

	if (addr >= 0x0000 && addr <= 0x1FFF) {
		return av->rom_data[av->chr_start + (mmc3_ppuaddr2romoffset(av, addr) & (av->chr_len - 1))];
	}

	if (addr >= 0x2000 && addr <= 0x3EFF) {
		/* Nametable */
		if (addr >= 0x3000)
			addr -= 0x1000;

		const unsigned int tick = PPU_TICKS_PER_FRAME - av->p.frame_ticks;
		const int scanline = (tick / 341) - 1;

		if (scanline >= 0 && scanline <= 239) {
			const unsigned int scanline_cycle = (tick % 341) - 1;
			if (scanline_cycle == 247 && mc->last_scanline != scanline) {
				mc->last_scanline = scanline;
				if (mc->irq_counter == 0)
					mc->irq_counter = mc->irq_latch + 1;
				else
					mc->irq_counter--;

				if (mc->irq_counter == 0 && mc->irq_enable)
					cpu_irq(&av->c);
			}
		}

		off_t off;
		if (mc->mirror == 0) {
			/* Vertical */
			off = (addr & 0x7FF);
		} else {
			/* Horizontal */
			off = (addr & 0x3FF) + (addr < 0x2800 ? 0 : 0x400);
		}
		return av->vram[off];
	}

	printf("[%s] PPU address $%04X not mapped\n", __func__, addr);
	return 0;
}

static void
mmc3_ppuwrite(struct avnes_context *av, uint16_t addr, uint8_t val)
{
	struct mmc3_context *mc = av->m.priv;

#ifdef MMC3_DEBUG
	printf("[%s] addr=$%04X val=$%02X\n", __func__, addr, val);
#endif

	if (addr >= 0x2000 && addr <= 0x3EFF) {
		/* Nametable */
		if (addr >= 0x3000)
			addr -= 0x1000;

		off_t off;
		if (mc->mirror == 0) {
			/* Vertical */
			off = (addr & 0x7FF);
		} else {
			/* Horizontal */
			off = (addr & 0x3FF) + (addr < 0x2800 ? 0 : 0x400);
		}
		av->vram[off] = val;
		return;
	}

	printf("[%s] PPU address $%04X not mapped\n", __func__, addr);
}

static int
mmc3_init(struct mapper_context *m, const uint8_t *data, size_t datalen)
{
	struct mmc3_context *mc;

	mc = calloc(1, sizeof(*mc));
	if (mc == NULL)
		return ENOMEM;

	mc->mirror = (data[6] & ROM_MIRROR_MASK) == ROM_MIRROR_V ? 0 : 1;

	m->cpuread = mmc3_cpuread;
	m->cpuwrite = mmc3_cpuwrite;
	m->ppuread = mmc3_ppuread;
	m->ppuwrite = mmc3_ppuwrite;
	m->priv = mc;

	return 0;
}

MAPPER_DECL(mmc3, "MMC3", mmc3_init);
