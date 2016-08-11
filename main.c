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

#include <sys/mman.h>
#include <sys/stat.h>
#include <assert.h>
#include <err.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cpu.h"
#include "ppu.h"
#include "io.h"

#include "sdl.h"

#define	NES_RAM_SIZE		2048
#define	NES_ROM_SIZE		32768

/* Memory map */
#define	NES_RAM_MINADDR_A	0x0000
#define	NES_RAM_MAXADDR_A	0x07FF
#define	NES_RAM_MINADDR_B	0x0800
#define	NES_RAM_MAXADDR_B	0x0FFF
#define	NES_RAM_MINADDR_C	0x1000
#define	NES_RAM_MAXADDR_C	0x17FF
#define	NES_RAM_MINADDR_D	0x1800
#define	NES_RAM_MAXADDR_D	0x1FFF
#define	NES_PPU_MINADDR		0x2000
#define	NES_PPU_MAXADDR		0x3FFF
#define	NES_PPU_OAMDMAADDR	0x4014
#define	NES_APU_MINADDR		0x4000
#define	NES_APU_MAXADDR		0x4015
#define	NES_IO_CONTROLLER_1	0x4016
#define	NES_IO_CONTROLLER_2	0x4017
#define	NES_ROM_MINADDR		0x8000
#define	NES_ROM_MAXADDR		0xFFFF

static uint8_t			*nes_ram;
static uint8_t			*nes_rom;
static uint8_t			*nes_chr;
static uint16_t			nes_chrsize;
static struct ppu_context	*nes_ppu;
static struct io_context	*nes_io;
static uint8_t			nes_flags6;

static struct cpu_context c;
static struct ppu_context p;
static struct io_context io;

static uint8_t
nes_read8(uint16_t addr)
{
//	printf("  [%s] addr=%04X\n", __func__, addr);

	/* 2KB internal RAM */
	if (addr >= NES_RAM_MINADDR_A && addr <= NES_RAM_MAXADDR_A)
		return nes_ram[addr - NES_RAM_MINADDR_A];

	/* Mirrors of internal RAM */
	if (addr >= NES_RAM_MINADDR_B && addr <= NES_RAM_MAXADDR_B)
		return nes_ram[addr - NES_RAM_MINADDR_B];
	if (addr >= NES_RAM_MINADDR_C && addr <= NES_RAM_MAXADDR_C)
		return nes_ram[addr - NES_RAM_MINADDR_C];
	if (addr >= NES_RAM_MINADDR_D && addr <= NES_RAM_MAXADDR_D)
		return nes_ram[addr - NES_RAM_MINADDR_D];

	/* PRG ROM */
	if (addr >= NES_ROM_MINADDR && addr <= NES_ROM_MAXADDR)
		return nes_rom[addr - NES_ROM_MINADDR];

	/* PPU */
	if (addr >= NES_PPU_MINADDR && addr <= NES_PPU_MAXADDR)
		return ppu_read(nes_ppu, addr);

	/* APU */
	if (addr >= NES_APU_MINADDR && addr <= NES_APU_MAXADDR) {
		//printf("TODO APU read $%04X\n", addr);
		return 0;
	}

	/* Controller */
	if (addr == NES_IO_CONTROLLER_1 || addr == NES_IO_CONTROLLER_2) {
		int index = addr - NES_IO_CONTROLLER_1;
		uint8_t shift = nes_io->shift[index];
		uint8_t val = (nes_io->state[index] >> shift) & 1;
		if (nes_io->shift[index] == 0)
			nes_io->shift[index] = 7;
		else
			nes_io->shift[index]--;
		return val;
	}

	printf("[%s] addr %04X not mapped\n", __func__, addr);
	abort();
}

static void
nes_write8(uint16_t addr, uint8_t val)
{

#if 0
	if (addr >= 0x100 && addr <= 0x1ff)
		printf("STACK write addr $%04X val $%02X\n", addr, val);
#endif

#if 0
	if (addr >= 0 && addr < 0x100)
		printf("ZEROPAGE write addr $%04X val $%02X\n", addr, val);
#endif

	/* 2KB internal RAM */
	if (addr >= NES_RAM_MINADDR_A && addr <= NES_RAM_MAXADDR_A) {
		nes_ram[addr - NES_RAM_MINADDR_A] = val;
		return;
	}

	/* Mirrors of internal RAM */
	if (addr >= NES_RAM_MINADDR_B && addr <= NES_RAM_MAXADDR_B) {
		nes_ram[addr - NES_RAM_MINADDR_B] = val;
		return;
	}
	if (addr >= NES_RAM_MINADDR_C && addr <= NES_RAM_MAXADDR_C) {
		nes_ram[addr - NES_RAM_MINADDR_C] = val;
		return;
	}
	if (addr >= NES_RAM_MINADDR_D && addr <= NES_RAM_MAXADDR_D) {
		nes_ram[addr - NES_RAM_MINADDR_D] = val;
		return;
	}

	/* PRG ROM */
	if (addr >= NES_ROM_MINADDR && addr <= NES_ROM_MAXADDR) {
		/* XXX */
		nes_rom[addr - NES_ROM_MINADDR] = val;
		printf("[%s] write to ROM ?? $%04X ($%02X)\n", __func__, addr, val);
		return;
	}

	/* PPU */
	if (addr >= NES_PPU_MINADDR && addr <= NES_PPU_MAXADDR) {
		ppu_write(nes_ppu, addr, val);
		return;
	}
	if (addr == NES_PPU_OAMDMAADDR) {
		for (int i = 0; i < PPU_OAM_SIZE; i++) {
			nes_ppu->oam[nes_ppu->oamaddr] = nes_read8(((uint16_t)val << 8) + i);
			nes_ppu->oamaddr = (nes_ppu->oamaddr + 1) & (PPU_OAM_SIZE - 1);
		}
		return;
	}

	/* APU */
	if (addr >= NES_APU_MINADDR && addr <= NES_APU_MAXADDR) {
		//printf("TODO APU write $%04X\n", addr);
		return;
	}
	if (addr == NES_IO_CONTROLLER_2) {
		/* Writes to $4017 are for APU frame counter register */
		//printf("TODO APU write $%04X\n", addr);
		return;
	}

	/* Controller */
	if (addr == NES_IO_CONTROLLER_1) {
		if ((val & 1) == 0) {
			/* Clearing S (strobe) resets controller shift registers */
			nes_io->shift[0] = nes_io->shift[1] = 7;
		}
		return;
	}

	printf("  [%s] addr %04X not mapped\n", __func__, addr);
	abort();
}

static void
usage(const char *pn)
{
	fprintf(stderr, "usage: %s rom.bin\n", pn);
	exit(EXIT_FAILURE);
}

static int
load_rom(const char *path)
{
	struct stat st;
	uint8_t *data;
	int rom_fd, prg_size, chr_size;
	off_t prg_offset, chr_offset;

	rom_fd = open(path, O_RDONLY);
	if (rom_fd == -1)
		err(EXIT_FAILURE, "Failed to open %s", path);
	if (fstat(rom_fd, &st) == -1)
		err(EXIT_FAILURE, "Failed to fstat %s", path);

	data = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, rom_fd, 0);
	if (data == MAP_FAILED)
		err(EXIT_FAILURE, "Failed to mmap %s", path);

	if (memcmp(data, "NES\x1a", 4) != 0)
		errx(EXIT_FAILURE, "Bad iNES header");

	prg_offset = 16;
	prg_size = 16384 * data[4];

	chr_offset = prg_offset + prg_size;
	chr_size = 8192 * data[5];

	/* PRG ROM must be either 16 or 32 KB */
	switch (prg_size) {
	case NES_ROM_SIZE:
		memcpy(nes_rom, data + prg_offset, prg_size);
		break;
	case NES_ROM_SIZE / 2:
		memcpy(nes_rom, data + prg_offset, prg_size);
		memcpy(nes_rom + (NES_ROM_SIZE / 2), data + prg_offset, prg_size);
		break;
	default:
		errx(EXIT_FAILURE, "Invalid PRG ROM size %d", prg_size);
	}

	/* CHR ROM */
	nes_chr = data + chr_offset;
	nes_chrsize = chr_size;

	/* Flags */
	nes_flags6 = data[6];

	return 0;
}

int
main(int argc, char *argv[])
{
	if (argc != 2)
		usage(argv[0]);

	if (sdl_init(basename(argv[1])) != 0)
		return EXIT_FAILURE;

	/* Allocate 2KB internal RAM */
	nes_ram = calloc(1, NES_RAM_SIZE);
	assert(nes_ram != NULL);

	/* Allocate 32KB for ROM */
	nes_rom = calloc(1, NES_ROM_SIZE);
	assert(nes_rom != NULL);

	/* Load ROM */
	if (load_rom(argv[1]) != 0)
		errx(EXIT_FAILURE, "Couldn't load ROM %s", argv[1]);

	c.read8 = nes_read8;
	c.write8 = nes_write8;
	cpu_init(&c);

	nes_ppu = &p;
	ppu_init(&p, &c, nes_flags6);
	/* Copy CHR-ROM to start of VRAM */
	memcpy(&p.vram[0], nes_chr, nes_chrsize);
	p.draw = sdl_draw;

	nes_io = &io;
	memset(&io, 0, sizeof(io));

	for (;;) {
		cpu_step(&c);
		if (ppu_step(&p) == 1) {
			if (sdl_poll(&io) != 0)
				break;
		}
	}

	return 0;
}
