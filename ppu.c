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

#include <sys/time.h>
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "ppu.h"

#define	PPU_REG_MIN	0x2000
#define	PPU_REG_MAX	0x3FFF
#define	PPU_REG_MASK	0x7

#define	REG_PPUCTRL	0
#define	 PPUCTRL_V	(1 << 7)	/* Vertical blank NMI enable */
#define	 PPUCTRL_H	(1 << 5)	/* Sprite size (0: 8x8; 1: 8x16) */
#define	 PPUCTRL_B	(1 << 4)	/* Background palette table address */
#define	 PPUCTRL_S	(1 << 3)	/* Sprite pattern table address for 8x8 sprites */
#define	 PPUCTRL_I	(1 << 2)	/* VRAM address increment per CPU read/write of PPUDATA */
#define	 PPUCTRL_N	(3 << 0)	/* Base nametable address */
#define	 PPUCTRL_N_X	(1 << 0)	/* Add 256 to the X scroll position */
#define	 PPUCTRL_N_Y	(2 << 0)	/* Add 240 to the Y scroll position */
#define	REG_PPUMASK	1
#define	 PPUMASK_s	(1 << 4)	/* Show sprites */
#define	 PPUMASK_b	(1 << 3)	/* Show background */
#define	 PPUMASK_m	(1 << 1)	/* Show background in leftmost 8 pixels */
#define	REG_PPUSTATUS	2
#define	 PPUSTATUS_V	(1 << 7)	/* Vertical blank */
#define	 PPUSTATUS_S	(1 << 6)	/* Sprite 0 Hit */
#define	REG_OAMADDR	3
#define	REG_OAMDATA	4
#define	REG_PPUSCROLL	5
#define	REG_PPUADDR	6
#define	REG_PPUDATA	7

static const struct timespec PPU_VBLANK_INTERVAL = {
	.tv_sec = 0,
	.tv_nsec = 1000000000 / 60000 * 1001
};

int
ppu_init(struct ppu_context *p)
{
	p->latch_addr = 2;
	p->latch_scroll = 2;

	clock_gettime(CLOCK_MONOTONIC, &p->next_vblank);
	timespecadd(&p->next_vblank, &PPU_VBLANK_INTERVAL);

	return 0;
}

uint8_t
ppu_read(struct ppu_context *p, uint16_t addr)
{
	uint16_t vaddr;
	uint8_t reg, val;
	int incr;

	if (addr < PPU_REG_MIN || addr > PPU_REG_MAX)
		assert("ppu: address out of range");

	reg = (addr - PPU_REG_MIN) & PPU_REG_MASK;

	//printf("  [%s] reg %d %02X\n", __func__, reg, p->regs[reg]);

	val = p->regs[reg];

	switch (reg) {
	case REG_PPUSTATUS:
		/* Clear vblank flag */
		p->regs[reg] &= ~PPUSTATUS_V;

		/* Clear scroll and addr latches */
		p->latch_scroll = 2;
		p->latch_addr = 2;
		break;
	case REG_PPUDATA:
		vaddr = p->vramaddr & (PPU_MEMMAP_SIZE - 1);

		if (vaddr <= 0x3eff) {
			/* Emulate PPUDATA read buffer (post-fetch) */
			val = p->ppudata;
			p->ppudata = p->read8(vaddr);
		} else
			val = p->read8(vaddr);

		incr = (p->regs[REG_PPUCTRL] & PPUCTRL_I) ? 32 : 1;
		p->vramaddr = (p->vramaddr + incr) & (PPU_MEMMAP_SIZE - 1);
		break;
	case REG_PPUSCROLL:
		break;
	}

	return val;
}

void
ppu_write(struct ppu_context *p, uint16_t addr, uint8_t val)
{
	uint16_t vaddr;
	uint8_t reg;
	int incr;

	if (addr < PPU_REG_MIN || addr > PPU_REG_MAX)
		assert("ppu: address out of range");

	reg = (addr - PPU_REG_MIN) & PPU_REG_MASK;

	//printf("  [%s] reg %d %02X -> %02X\n", __func__, reg, p->regs[reg], val);

	p->regs[reg] = val;

	switch (reg) {
	case REG_PPUADDR:
		if (p->latch_addr == 0)
			p->latch_addr = 2;
		if (p->latch_addr > 0) {
			//printf("PPU vramaddr %04X ", p->vramaddr);
			p->vramaddr = (p->vramaddr << 8) | val;
			//printf("-> %04X ", p->vramaddr);
			--p->latch_addr;
		}
		if (p->latch_addr == 0) {
			p->vramaddr &= (PPU_MEMMAP_SIZE - 1);
			//printf("new PPUADDR $%04X\n", p->vramaddr);
#if 0
			if (p->vramaddr < 0x2000 || p->vramaddr > 0x3fff)
				assert("Strange VRAM address" == NULL);
#endif
		}
		break;
	case REG_PPUDATA:
		p->vramaddr &= (PPU_MEMMAP_SIZE - 1);
		//printf("PPUDATA addr $%04X\n", p->vramaddr);
		vaddr = p->vramaddr & (PPU_MEMMAP_SIZE - 1);

		p->write8(vaddr, val);
#if 0
		if (vaddr >= 0x3f00)
			printf("PPU Palette write $%04X : $%02X\n", vaddr, val);
#endif
		incr = (p->regs[REG_PPUCTRL] & PPUCTRL_I) ? 32 : 1;
		p->vramaddr = (p->vramaddr + incr) & (PPU_MEMMAP_SIZE - 1);
		break;
	case REG_OAMADDR:
		p->oamaddr = val;
		break;
	case REG_OAMDATA:
		p->oam[p->oamaddr] = val;
		p->oamaddr = (p->oamaddr + 1) & (PPU_OAM_SIZE - 1);
		break;
	case REG_PPUSCROLL:
		if (p->latch_scroll == 0)
			p->latch_scroll = 2;
		if (p->latch_scroll > 0) {
			p->scroll = (p->scroll << 8) | val;
			--p->latch_scroll;
		}
		break;
	}
}

static uint8_t
ppu_get_cs_for_bgpixel(uint8_t attr, unsigned int x, unsigned int y)
{
	if ((x & 0xf) < 8 && (y & 0xf) < 8)
		return (attr >> 0) & 0x3;	/* top left */
	else if ((x & 0xf) < 8 && (y & 0xf) >= 8)
		return (attr >> 4) & 0x3;	/* bottom left */
	else if ((x & 0xf) >= 8 && (y & 0xf) < 8)
		return (attr >> 2) & 0x3;	/* top right */
	else
		return (attr >> 6) & 0x3;	/* bottom right */
}

static void
ppu_put_pixel(struct ppu_context *p, unsigned int x, unsigned int y)
{

	/* Rendering control flags */
	const int show_background = (p->regs[REG_PPUMASK] & PPUMASK_b) != 0;
	const int show_sprites = (p->regs[REG_PPUMASK] & PPUMASK_s) != 0;

	/* X/Y scrolling */
	const unsigned int xscroll = ((p->scroll >> 8) & 0xff);
	const unsigned int yscroll = (p->scroll & 0xff);

	/* Base nametable address */
	uint16_t nt_start = 0x2000 + (p->regs[REG_PPUCTRL] & PPUCTRL_N) * 0x400;
	if (x + xscroll > 0xff)
		nt_start ^= 0x400;
	if (y + yscroll > 0xff)
		nt_start ^= 0x800;

	/* Attribute table starts at the end of the nametable */
	const uint16_t attr_start = nt_start + 0x3c0;
	/* Palette address */
	const uint16_t pal_start = 0x3f00;

	if (show_background) {
		/* Background pattern table address */
		const uint16_t pat_start = (p->regs[REG_PPUCTRL] & PPUCTRL_B) ? 0x1000 : 0x0000;

		uint8_t xrel = (x + xscroll) & 0xff;
		uint8_t yrel = (y + yscroll) & 0xff;
	
		/* Offset of nametable entry */
		uint16_t nt_off = ((yrel / 8) * 32) + (xrel / 8);
		/* Offset of attribute table entry */
		uint16_t attr_off = ((yrel / 32) * 8) + (xrel / 32);

		/* Nametable entry */
		const uint8_t nt = p->read8(nt_start + nt_off);

		/* Offset of pattern table entry (low) */
		const uint16_t pat_off = (uint16_t)nt * 16 + (yrel & 7);

		/* Pattern table entry */
		const uint8_t pat_l = p->read8(pat_start + pat_off);
		const uint8_t pat_h = p->read8(pat_start + pat_off + 8);
		/* Bit in pattern table */
		const int bit = 7 - (xrel & 7);
		/* Palette entry */
		const uint8_t pal = ((pat_l & (1 << bit)) ? 1 : 0) |
				    ((pat_h & (1 << bit)) ? 2 : 0);

		p->pixels[y][x].pal = pal;

		if (pal) {
			p->pixels[y][x].priority = PPU_PRIO_BG;
			/* Attribute table entry */
			const uint8_t attr = p->read8(attr_start + attr_off);
			/* Colour set */
			const uint8_t cs = ppu_get_cs_for_bgpixel(attr, x + xscroll, y + yscroll);

			p->pixels[y][x].c = p->read8(pal_start + (cs * 4) + pal);
		} else {
			/* Default background colour */
			p->pixels[y][x].priority = PPU_PRIO_NONE;
			p->pixels[y][x].c = p->read8(pal_start);
		}

		if (x < 8 && (p->regs[REG_PPUMASK] & PPUMASK_m) == 0) {
			/* Hide background in leftmost 8 pixels */
			p->pixels[y][x].priority = PPU_PRIO_NONE;
			p->pixels[y][x].pal = 0;
			p->pixels[y][x].c = p->read8(pal_start);
		}
	} else {
		p->pixels[y][x].priority = PPU_PRIO_NONE;
		p->pixels[y][x].pal = 0;
		p->pixels[y][x].c = p->read8(pal_start);
	}

	if (show_sprites) {
		/* Sprite height (8 or 16 pixels) */
		const uint16_t sprite_height = (p->regs[REG_PPUCTRL] & PPUCTRL_H) ? 16 : 8;

		for (int n = 0; n < 64; n++) {
			const uint8_t sprite_y = p->oam[n * 4 + 0] + 1;
			const uint8_t sprite_x = p->oam[n * 4 + 3];

			if (sprite_y == 0 || sprite_y >= 0xf0)
				continue;

			if (x >= sprite_x && x < sprite_x + 8 && y >= sprite_y && y < sprite_y + sprite_height) {
				const unsigned int xrel = x - sprite_x;
				const unsigned int yrel = y - sprite_y;
				const uint8_t sprite_tile = p->oam[n * 4 + 1];
				const uint8_t sprite_attr = p->oam[n * 4 + 2];

				/* Horizontal flip flag */
				const int flip_h = (sprite_attr & 0x40) != 0;
				/* Vertical flip flag */
				const int flip_v = (sprite_attr & 0x80) != 0;
				/* Priority */
				const int priority = (sprite_attr & 0x20) != 0 ? PPU_PRIO_BEHIND : PPU_PRIO_FRONT;

				/* Sprite pattern table address */
				const uint16_t pat_start = sprite_height == 8 ?
				    ((p->regs[REG_PPUCTRL] & PPUCTRL_S) ? 0x1000 : 0x0000) :
				    (sprite_tile & 1) << 12;

				/* Sprite tile start */
				const uint16_t sprite_tile_start = sprite_height == 8 ?
				    sprite_tile : (sprite_tile & 0xfe);

				/* Offset of pattern table entry (low) */
				const uint16_t pat_off = sprite_tile_start * 16 +
				    (flip_v ? ((sprite_height - 1) - (yrel & (sprite_height - 1))) : (yrel & (sprite_height - 1)));

				/* Pattern table entry */
				const uint8_t pat_l = p->read8(pat_start + pat_off);
				const uint8_t pat_h = p->read8(pat_start + pat_off + 8);
				/* Bit in pattern table */
				const int bit = flip_h ? (xrel & 7) : 7 - (xrel & 7);
				/* Palette entry */
				const uint8_t pal = ((pat_l & (1 << bit)) ? 1 : 0) |
						    ((pat_h & (1 << bit)) ? 2 : 0);

				/*
				 * When a nonzero pixel of sprite 0 overlaps a nonzero background pixel,
				 * set the Sprite 0 Hit flag in PPUSTATUS
				 */
				if (n == 0 && x < 255 && p->pixels[y][x].pal != 0 && pal != 0) {
					p->regs[REG_PPUSTATUS] |= PPUSTATUS_S;
				}

				/* Skip pixels in front of this one. If sprites overlap, the lower numbered sprite wins. */
				if (priority <= p->pixels[y][x].priority || pal == 0)
					continue;

				/* Colour set */
				const uint8_t cs = (sprite_attr & 0x3) + 4;

				p->pixels[y][x].priority = priority;
				p->pixels[y][x].pal = pal;
				p->pixels[y][x].c = p->read8(pal_start + (cs * 4) + pal);

				/* Emulate sprite priority quirk; visible sprites with the lowest OAM index win regardless of front/back priority */
				break;
			}
		}
	}
}

int
ppu_step(struct ppu_context *p)
{
	struct timespec ts, when;

	if (p->draw_cycles > 0) {
		for (int i = 0; i < 3; i++) {
			unsigned int off = (PPU_WIDTH * PPU_HEIGHT) - p->draw_cycles;
			unsigned int y = off / PPU_WIDTH;
			unsigned int x = off - (y * PPU_WIDTH);
			ppu_put_pixel(p, x, y);

			if (--p->draw_cycles == 0) {
				if (p->draw)
					p->draw(p);

				if (!p->clear_s0 && (p->regs[REG_PPUSTATUS] & PPUSTATUS_S) != 0)
					p->clear_s0 = 1;
				else if (p->clear_s0) {
					/* Clear Sprite 0 Hit */
					p->regs[REG_PPUSTATUS] &= ~PPUSTATUS_S;
				}

				/* Set vblank flag */
				p->regs[REG_PPUSTATUS] |= PPUSTATUS_V;

				return 1;
			}
		}
		return 0;
	}

	clock_gettime(CLOCK_MONOTONIC, &ts);
	if (timespeccmp(&ts, &p->next_vblank, >=) && p->draw_cycles == 0) {
		p->draw_cycles = PPU_WIDTH * PPU_HEIGHT;

#if 0
		printf("Sprites:");
		for (int n = 0; n < 64; n++) {
			const uint8_t sprite_y = p->oam[n * 4 + 0] + 1;
			const uint8_t sprite_x = p->oam[n * 4 + 3];

			if (sprite_y == 0 || sprite_y >= 0xf0)
				break;
			printf(" %02X@%dx%d", n, sprite_x, sprite_y);
		}
		printf("\n");
#endif

		if (p->regs[REG_PPUCTRL] & PPUCTRL_V)
			cpu_nmi(p->c);

#if 0
		/* X/Y scrolling */
		const unsigned int xscroll = ((p->scroll >> 8) & 0xff) +
		    ((p->regs[REG_PPUCTRL] & PPUCTRL_N_X) ? PPU_WIDTH : 0);
		const unsigned int yscroll = (p->scroll & 0xff) +
		    ((p->regs[REG_PPUCTRL] & PPUCTRL_N_Y) ? PPU_HEIGHT : 0);
		printf("Scroll X=%d Y=%d\n", xscroll, yscroll);
#endif

		clock_gettime(CLOCK_MONOTONIC, &p->next_vblank);
		timespecadd(&p->next_vblank, &PPU_VBLANK_INTERVAL);
	}

	return 0;
}
