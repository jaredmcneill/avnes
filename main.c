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

#include "avnes.h"
#include "rom.h"

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

#define	NES_ROM_MINADDR		0x4020
#define	NES_ROM_MAXADDR		0xFFFF

struct avnes_context avnes;

static uint8_t
nes_cpuread8(uint16_t addr)
{
//	printf("  [%s] addr=%04X\n", __func__, addr);

	/* 2KB internal RAM */
	if (addr >= NES_RAM_MINADDR_A && addr <= NES_RAM_MAXADDR_D) {
		return avnes.ram[addr & NES_RAM_MAXADDR_A];
	}

	/* PPU registers */
	if (addr >= NES_PPU_MINADDR && addr <= NES_PPU_MAXADDR) {
		return ppu_read(&avnes.p, addr);
	}

	/* APU */
	if (addr >= NES_APU_MINADDR && addr <= NES_APU_MAXADDR) {
		return apu_read(&avnes.a, addr);
	}

	/* Controller */
	if (addr == NES_IO_CONTROLLER_1 || addr == NES_IO_CONTROLLER_2) {
		int index = addr - NES_IO_CONTROLLER_1;
		uint8_t shift = avnes.io.shift[index];
		uint8_t val = (avnes.io.state[index] >> shift) & 1;
		if (avnes.io.shift[index] == 0)
			avnes.io.shift[index] = 7;
		else
			avnes.io.shift[index]--;
		return val;
	}

	/* Cartridge space */
	if (addr >= NES_ROM_MINADDR && addr <= NES_ROM_MAXADDR) {
		return avnes.m.cpuread(&avnes, addr);
	}

	printf("[%s] addr %04X not mapped\n", __func__, addr);
	abort();
}

static void
nes_cpuwrite8(uint16_t addr, uint8_t val)
{

	/* 2KB internal RAM */
	if (addr >= NES_RAM_MINADDR_A && addr <= NES_RAM_MAXADDR_D) {
		avnes.ram[addr & NES_RAM_MAXADDR_A] = val;
		return;
	}

	/* PPU registers */
	if (addr >= NES_PPU_MINADDR && addr <= NES_PPU_MAXADDR) {
		ppu_write(&avnes.p, addr, val);
		return;
	}
	if (addr == NES_PPU_OAMDMAADDR) {
		for (int i = 0; i < PPU_OAM_SIZE; i++) {
			avnes.p.oam[avnes.p.oamaddr] = nes_cpuread8(((uint16_t)val << 8) + i);
			avnes.p.oamaddr = (avnes.p.oamaddr + 1) & (PPU_OAM_SIZE - 1);
		}
		return;
	}

	/* APU */
	if (addr >= NES_APU_MINADDR && addr <= NES_APU_MAXADDR) {
		apu_write(&avnes.a, addr, val);
		return;
	}
	if (addr == NES_IO_CONTROLLER_2) {
		/* Writes to $4017 are for APU frame counter register */
		apu_write(&avnes.a, addr, val);
		return;
	}

	/* Controller */
	if (addr == NES_IO_CONTROLLER_1) {
		if ((val & 1) == 0) {
			/* Clearing S (strobe) resets controller shift registers */
			avnes.io.shift[0] = avnes.io.shift[1] = 7;
		}
		return;
	}

	/* Cartridge space */
	if (addr >= NES_ROM_MINADDR && addr <= NES_ROM_MAXADDR) {
		avnes.m.cpuwrite(&avnes, addr, val);
		return;
	}

	printf("  [%s] addr %04X not mapped\n", __func__, addr);
	abort();
}

static uint8_t
nes_ppuread8(uint16_t addr)
{
	assert (addr <= 0x3FFF);

	if (addr >= 0x3F00 && addr <= 0x3FFF) {
		switch (addr) {
		case 0x3F10:
		case 0x3F14:
		case 0x3F18:
		case 0x3F1C:
			addr -= 0x10;
			break;
		}
		return avnes.paletteram[addr & (AVNES_PALETTERAM_SIZE - 1)];
	}

	return avnes.m.ppuread(&avnes, addr);
}

static void
nes_ppuwrite8(uint16_t addr, uint8_t val)
{
	if (addr >= 0x3F00 && addr <= 0x3FFF) {
		switch (addr) {
		case 0x3F10:
		case 0x3F14:
		case 0x3F18:
		case 0x3F1C:
			addr -= 0x10;
			break;
		}
		avnes.paletteram[addr & (AVNES_PALETTERAM_SIZE - 1)] = val;
		return;
	}

	avnes.m.ppuwrite(&avnes, addr, val);
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
	int rom_fd;

	rom_fd = open(path, O_RDONLY);
	if (rom_fd == -1)
		err(EXIT_FAILURE, "Failed to open %s", path);
	if (fstat(rom_fd, &st) == -1)
		err(EXIT_FAILURE, "Failed to fstat %s", path);

	avnes.rom_data = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, rom_fd, 0);
	if (avnes.rom_data == MAP_FAILED)
		err(EXIT_FAILURE, "Failed to mmap %s", path);

	if (memcmp(avnes.rom_data, "NES\x1a", 4) != 0)
		errx(EXIT_FAILURE, "Bad iNES header");

	avnes.prg_start = ROM_PRG_START(avnes.rom_data);
	avnes.prg_len = ROM_PRG_LENGTH(avnes.rom_data);
	avnes.chr_start = ROM_CHR_START(avnes.rom_data);
	avnes.chr_len = ROM_CHR_LENGTH(avnes.rom_data);

	return mapper_init(&avnes.m, avnes.rom_data, avnes.rom_datalen);
}

int
main(int argc, char *argv[])
{
#ifdef CPU_TIMING_DEBUG
	uint64_t lf_cputicks = 0;
#endif

	if (argc != 2)
		usage(argv[0]);

	/* Load ROM */
	if (load_rom(argv[1]) != 0)
		errx(EXIT_FAILURE, "Couldn't load ROM %s", argv[1]);

	if (sdl_init(basename(argv[1])) != 0)
		return EXIT_FAILURE;

	memset(&avnes.c, 0, sizeof(avnes.c));
	avnes.c.read8 = nes_cpuread8;
	avnes.c.write8 = nes_cpuwrite8;
	cpu_init(&avnes.c);

	memset(&avnes.a, 0, sizeof(avnes.a));
	avnes.a.c = &avnes.c;
	avnes.a.play = sdl_play;
	avnes.a.read8 = nes_cpuread8;
	avnes.a.write8 = nes_cpuwrite8;
	apu_init(&avnes.a);

	memset(&avnes.p, 0, sizeof(avnes.p));
	avnes.p.c = &avnes.c;
	avnes.p.draw = sdl_draw;
	avnes.p.read8 = nes_ppuread8;
	avnes.p.write8 = nes_ppuwrite8;
	ppu_init(&avnes.p);

	memset(&avnes.io, 0, sizeof(avnes.io));

	for (;;) {
		int frame_complete = 0;

		cpu_step(&avnes.c);
		apu_step(&avnes.a);

		for (int i = 0; i < 3; i++) {
			int status = ppu_step(&avnes.p);
#ifdef CPU_TIMING_DEBUG
			if (status && (avnes.p.frame % 60) == 0) {
				printf("CPU frequency: %lu Hz\n", avnes.c.ticks - lf_cputicks);
				lf_cputicks = avnes.c.ticks;
			}
#endif
			frame_complete += status;
		}

		if (frame_complete) {
			int pending;
			while ((pending = sdl_poll(&avnes.io)) == 0)
				;
			if (pending == 1)
				break;
		}
	}

	return 0;
}
