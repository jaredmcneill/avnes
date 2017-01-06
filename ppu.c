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

#include "avnes.h"
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

#define	PPU_TILE_ADDR(v)	(0x2000 | ((v) & 0x0fff))
#define	PPU_ATTR_ADDR(v)	(0x23c0 | ((v) & 0x0c00) | (((v) >> 4) & 0x38) | (((v) >> 2) & 0x07))

static uint16_t
ppu_incr_x(struct ppu_context *p)
{
	uint16_t v = p->reg_v;

	if ((v & 0x001f) == 31) {
		v &= ~0x001f;
		v ^= 0x0400;
	} else {
		v += 1;
	}

#ifdef PPU_DEBUG
	printf("PPU INCX $%04X\n", v);
#endif
	return v;
}

static uint16_t
ppu_incr_y(struct ppu_context *p)
{
	uint16_t v = p->reg_v;

	if ((v & 0x7000) != 0x7000) {
		v += 0x1000;
	} else {
		v &= ~0x7000;
		uint16_t y = (v & 0x03e0) >> 5;
		if (y == 29) {
			y = 0;
			v ^= 0x0800;
		} else if (y == 31) {
			y = 0;
		} else {
			y += 1;
		}
		v = (v & ~0x03e0) | (y << 5);
	}

#ifdef PPU_DEBUG
	printf("PPU INCY $%04X\n", v);
#endif
	return v;
}

int
ppu_init(struct ppu_context *p)
{
	p->frame = 0;
	p->frame_ticks = PPU_TICKS_PER_FRAME;

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

	switch (reg) {
	case REG_PPUSTATUS:
		/* Get current state */
		val = p->regs[reg];

		/* Clear vblank flag */
		p->regs[reg] &= ~PPUSTATUS_V;

		/* Clear write toggle bit */
		p->reg_w = 0;

		break;
	case REG_PPUDATA:
		vaddr = p->reg_v & (PPU_MEMMAP_SIZE - 1);

		if (vaddr <= 0x3eff) {
			/* Emulate PPUDATA read buffer (post-fetch) */
			val = p->ppudata;
			p->ppudata = p->read8(vaddr);
		} else
			val = p->read8(vaddr);

		incr = (p->regs[REG_PPUCTRL] & PPUCTRL_I) ? 32 : 1;
		p->reg_v = (p->reg_v + incr) & (PPU_MEMMAP_SIZE - 1);
#ifdef PPU_DEBUG
		printf("PPUDATA r $%04X\n", p->reg_v);
#endif
		break;

	default:
		val = p->regs[reg];
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

	switch (reg) {
	case REG_PPUSCROLL:
		if (p->reg_w == 0) {
			/* First write */
			p->reg_t &= ~0x1f;
			p->reg_t |= (val >> 3);
			p->reg_x = (val & 7);
		} else {
			/* Second write */
			p->reg_t &= ~(0x1f << 5);
			p->reg_t |= ((uint16_t)val >> 3) << 5;
			p->reg_t &= ~(0x7 << 12);
			p->reg_t |= ((uint16_t)val & 0x7) << 12;
		}
		p->reg_w ^= 1;
		p->regs[reg] = val;
		break;

	case REG_PPUADDR:
		if (p->reg_w == 0) {
			/* First write */
			p->reg_t &= ~(0x3f << 8);
			p->reg_t |= ((uint16_t)val & 0x3f) << 8;
			p->reg_t &= ~(1 << 14);
		} else {
			/* Second write */
			p->reg_t &= ~0xff;
			p->reg_t |= val;
			p->reg_v = p->reg_t;
#ifdef PPU_DEBUG
			printf("PPUADDR $%04X\n", p->reg_v);
#endif
		}
		p->reg_w ^= 1;
		p->regs[reg] = val;
		break;

	case REG_PPUCTRL:
		p->regs[reg] = val;
		p->reg_t &= ~(0x3 << 10);
		p->reg_t |= ((uint16_t)val & 0x3) << 10;

		break;

	case REG_PPUDATA:
		vaddr = p->reg_v & (PPU_MEMMAP_SIZE - 1);

		p->write8(vaddr, val);

		incr = (p->regs[REG_PPUCTRL] & PPUCTRL_I) ? 32 : 1;
		p->reg_v = (p->reg_v + incr) & (PPU_MEMMAP_SIZE - 1);

#ifdef PPU_DEBUG
		const unsigned int tick = PPU_TICKS_PER_FRAME - p->frame_ticks;
		const int scanline = (tick / 341) - 1;
		const unsigned int scanline_cycle = tick % 341;

		printf("PPUDATA w $%04X (tick %d scanline %d dot %d)\n", p->reg_v, tick, scanline, scanline_cycle);
#endif
		break;

	case REG_OAMADDR:
		p->oamaddr = val;
		p->regs[reg] = val;
		break;

	case REG_OAMDATA:
		p->oam[p->oamaddr] = val;
		p->oamaddr = (p->oamaddr + 1) & (PPU_OAM_SIZE - 1);
		p->regs[reg] = val;
		break;

	case REG_PPUSTATUS:
		break;

	default:
		p->regs[reg] = val;
		break;

	}
}

static uint8_t
ppu_get_cs_for_bgpixel(uint8_t attr, unsigned int x, unsigned int y)
{
	if ((x & 0x1f) < 16 && (y & 0x1f) < 16)
		return (attr >> 0) & 0x3;	/* top left */
	else if ((x & 0x1f) < 16 && (y & 0x1f) >= 16)
		return (attr >> 4) & 0x3;	/* bottom left */
	else if ((x & 0x1f) >= 16 && (y & 0x1f) < 16)
		return (attr >> 2) & 0x3;	/* top right */
	else
		return (attr >> 6) & 0x3;	/* bottom right */
}

static void
ppu_get_sprites(struct ppu_context *p, unsigned int y)
{
	/* Rendering control flags */
	const int show_sprites = (p->regs[REG_PPUMASK] & PPUMASK_s) != 0;

	/* Sprite height (8 or 16 pixels) */
	const uint16_t sprite_height = (p->regs[REG_PPUCTRL] & PPUCTRL_H) ? 16 : 8;

	p->scanline_num_sprites = 0;

	for (int n = 0; n < 64 && p->scanline_num_sprites < 8; n++) {
		const uint8_t sprite_y = p->oam[n * 4 + 0] + 1;

		if (sprite_y == 0 || sprite_y >= 0xf0)
			continue;

		if (y >= sprite_y && y < sprite_y + sprite_height)
			p->scanline_sprites[p->scanline_num_sprites++] = n;
	}
}

static void
ppu_put_pixel(struct ppu_context *p, unsigned int x, unsigned int y)
{

	/* Rendering control flags */
	const int show_background = (p->regs[REG_PPUMASK] & PPUMASK_b) != 0;
	const int show_sprites = (p->regs[REG_PPUMASK] & PPUMASK_s) != 0;

	/* Palette address */
	const uint16_t pal_start = 0x3f00;

	if (show_background) {
		/* Fine Y scroll */
		const uint8_t fine_y = p->reg_v >> 12;
		const uint8_t fine_x = p->reg_x;

		/* X/Y scroll positions */
		const int xscroll = x + p->reg_x;
		const int yscroll = (((p->reg_v >> 5) & 0x1f) << 3) | fine_y;

		/* Colour set */
		const uint8_t cs = (p->attr >> 30) & 3;

		/* Palette entry */
		const int bit = 1 << (15 - p->reg_x);
		const uint8_t pal = ((p->tile_l & bit) ? 1 : 0) |
				    ((p->tile_h & bit) ? 2 : 0);

		p->tile_l <<= 1;
		p->tile_h <<= 1;
		p->attr <<= 2;

		if (x <= 0xff) {
			p->pixels[y][x].pal = pal;
			p->pixels[y][x].has_sprite = 0;

			if (pal) {
				p->pixels[y][x].priority = PPU_PRIO_BG;

				p->pixels[y][x].c = p->read8(pal_start + (cs * 4) + pal) & 0x3f;
			} else {
				/* Default background colour */
				p->pixels[y][x].priority = PPU_PRIO_NONE;
				p->pixels[y][x].c = p->read8(pal_start) & 0x3f;
			}

			if (x < 8 && (p->regs[REG_PPUMASK] & PPUMASK_m) == 0) {
				/* Hide background in leftmost 8 pixels */
				p->pixels[y][x].priority = PPU_PRIO_NONE;
				p->pixels[y][x].pal = 0;
				p->pixels[y][x].c = p->read8(pal_start) & 0x3f;
			}
		}
	} else {
		if (x <= 0xff) {
			p->pixels[y][x].priority = PPU_PRIO_NONE;
			p->pixels[y][x].pal = 0;
			p->pixels[y][x].c = p->read8(pal_start) & 0x3f;
			p->pixels[y][x].has_sprite = 0;
		}
	}

	if (x > 0xff)
		return;

	if (show_sprites) {
		/* Sprite height (8 or 16 pixels) */
		const uint16_t sprite_height = (p->regs[REG_PPUCTRL] & PPUCTRL_H) ? 16 : 8;

		for (int i = 0; i < p->scanline_num_sprites; i++) {
			const int n = p->scanline_sprites[i];
			const uint8_t sprite_y = p->oam[n * 4 + 0] + 1;
			const uint8_t sprite_x = p->oam[n * 4 + 3];

			if (sprite_y == 0 || sprite_y >= 0xf0)
				continue;

			if (x < sprite_x || x >= sprite_x + 8)
				continue;

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
			uint16_t pat_off = sprite_tile_start * 16 +
			    (flip_v ? ((sprite_height - 1) - (yrel & (sprite_height - 1))) : (yrel & (sprite_height - 1)));
			if (yrel >= 8)
				pat_off += 8;
			if (sprite_height == 16 && flip_v) {
				if (yrel >= 8)
					pat_off -= 8;
				else
					pat_off += 8;
			}

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
			if (n == 0 && p->pixels[y][x].pal != 0 && pal != 0) {
				if (p->sprite0_hit == 0) {
					p->regs[REG_PPUSTATUS] |= PPUSTATUS_S;
					p->sprite0_hit = 1;
				}
			}

			/* Non-transparent back-priority sprites with a lower sprite index have a higher priority */
			if (p->pixels[y][x].has_sprite)
				continue;
			if (pal)
				p->pixels[y][x].has_sprite = 1;

			/* Skip pixels in front of this one. If sprites overlap, the lower numbered sprite wins. */
			if (priority <= p->pixels[y][x].priority || pal == 0)
				continue;

			/* Colour set */
			const uint8_t cs = (sprite_attr & 0x3) + 4;

			p->pixels[y][x].priority = priority;
			p->pixels[y][x].pal = pal;
			p->pixels[y][x].c = p->read8(pal_start + (cs * 4) + pal) & 0x3f;

			/* Emulate sprite priority quirk; visible sprites with the lowest OAM index win regardless of front/back priority */
			break;
		}
	}
}

static void
ppu_fetch(struct ppu_context *p)
{
	/* Nametable entry */
	const uint8_t nt = p->read8(PPU_TILE_ADDR(p->reg_v));

	/* Fine Y scroll */
	const uint8_t fine_y = p->reg_v >> 12;
	const uint8_t fine_x = p->reg_x;

	/* X/Y scroll positions */
	const int xscroll = ((p->reg_v & 0x1f) << 3) | fine_x;
	const int yscroll = (((p->reg_v >> 5) & 0x1f) << 3) | fine_y;

	/* Attribute table entry */
	uint16_t attr = 0;
	uint8_t curattr = p->read8(PPU_ATTR_ADDR(p->reg_v));
	for (int i = 0; i < 8; i++) {
		if (i == (8 - p->reg_x))
			curattr = p->read8(PPU_ATTR_ADDR(ppu_incr_x(p)));
		attr <<= 2;
		attr |= ppu_get_cs_for_bgpixel(curattr, xscroll + i, yscroll);
	}
	p->attr |= attr;

	/* Pattern table start offset */
	const uint16_t pat_start = (p->regs[REG_PPUCTRL] & PPUCTRL_B) ? 0x1000 : 0x0000;

	/* Offset of pattern table entry (low) */
	const uint16_t pat_off = (uint16_t)nt * 16 + (p->reg_v >> 12);
			
	/* Pattern table entry */
	p->tile_l |= p->read8(pat_start + pat_off);
	p->tile_h |= p->read8(pat_start + pat_off + 8);

	p->reg_v = ppu_incr_x(p);
}

int
ppu_step(struct ppu_context *p)
{
	struct timespec ts;
	int ret = 0;

	if (p->frame_ticks == 0) {
#if 0
		if (p->draw)
			p->draw(p);
#else
		extern void sdl_draw(struct ppu_context *);
		sdl_draw(p);
#endif

		p->frame_ticks = PPU_TICKS_PER_FRAME;
		if ((p->frame & 1) != 0 || (p->regs[REG_PPUMASK] & PPUMASK_b) != 0) {
			/* Pre-render scanline -1, cycle 0 is skipped for odd PPU frames and when the BG is disabled */
			p->frame_ticks -= 1;
		}

		p->frame++;

		ret = 1;
	} else {
		const unsigned int tick = PPU_TICKS_PER_FRAME - p->frame_ticks;
		const int scanline = (tick / 341) - 1;
		const unsigned int scanline_cycle = tick % 341;

		/* Rendering control flags */
		const int show_background = (p->regs[REG_PPUMASK] & PPUMASK_b) != 0;
		const int show_sprites = (p->regs[REG_PPUMASK] & PPUMASK_s) != 0;
		const int render_enable = show_background || show_sprites;

		if (render_enable && scanline == -1) {
			/* Pre-render scanline */
			if (scanline_cycle == 1) {
				/* Clear VBlank, Sprite 0 Hit */
				p->regs[REG_PPUSTATUS] &= ~(PPUSTATUS_S | PPUSTATUS_V);
				p->sprite0_hit = 0;
			} else if (scanline_cycle >= 280 && scanline_cycle <= 304) {
				/* Reload vertical scroll position */
				p->reg_v &= ~0x7be0;
				p->reg_v |= (p->reg_t & 0x7be0);
#ifdef PPU_DEBUG
				printf("PPU V   $%04X\n", p->reg_v);
#endif
			}
		} else if (render_enable && scanline >= 0 && scanline <= 239) {
			/* Visible scanlines */
			if (scanline_cycle == 0) {
				/* Idle cycle */
				ppu_get_sprites(p, scanline);
			} else if (scanline_cycle >= 1 && scanline_cycle <= 256) {
				/* Fetch cycle */
				ppu_put_pixel(p, scanline_cycle - 1, scanline);
			} else if (scanline_cycle >= 257 && scanline_cycle <= 320) {
				/* Tile data for sprites on the next scanline are fetched (XXX) */
			} else if (scanline_cycle >= 321 && scanline_cycle <= 336) {
				/* First two tiles for the next scanline are fetched (XXX) */
			} else if (scanline_cycle >= 337 && scanline_cycle <= 340) {
				/* Two nametable bytes are fetched for unknown purposes (XXX) */
			}
		} else if (scanline == 240) {
			/* Post-render scanline */
		} else if (scanline >= 241 && scanline <= 260) {
			/* Vertical blanking lines */
			if (scanline == 241 && scanline_cycle == 1) {
				/* Set VBlank flag on second tick of scanline 241 */
				p->regs[REG_PPUSTATUS] |= PPUSTATUS_V;

				/* VBlank NMI */
				if (p->regs[REG_PPUCTRL] & PPUCTRL_V) {
					cpu_nmi(p->c);
				}
			}
		}

		if (render_enable && scanline >= -1 && scanline <= 239) {
			if (scanline_cycle > 0 && (scanline_cycle & 7) == 0 && scanline_cycle < 256) {
				ppu_fetch(p);
			}
			if (scanline_cycle == 256) {
				p->reg_v = ppu_incr_y(p);
			}
			if (scanline_cycle == 257) {
				/* Reload horizontal scroll position */
				p->reg_v &= ~0x41f;
				p->reg_v |= (p->reg_t & 0x41f);
#ifdef PPU_DEBUG
				printf("PPU H   $%04X\n", p->reg_v);
#endif
			}

			if (scanline_cycle == 321) {
				p->tile_l = p->tile_h = 0;
				ppu_fetch(p);
				p->tile_l <<= 8;
				p->tile_h <<= 8;
				p->attr <<= 16;
			} else if (scanline_cycle == 329) {
				ppu_fetch(p);
			}
		}

		--p->frame_ticks;
	}

	return ret;
}
