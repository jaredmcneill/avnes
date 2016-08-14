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

#ifndef _PPU_H
#define _PPU_H

#include <time.h>
#include "cpu.h"

#define timespeccmp(tvp, uvp, cmp)                                      \
        (((tvp)->tv_sec == (uvp)->tv_sec) ?                             \
            ((tvp)->tv_nsec cmp (uvp)->tv_nsec) :                       \
            ((tvp)->tv_sec cmp (uvp)->tv_sec))

#define timespecadd(vvp, uvp)                                           \
        do {                                                            \
                (vvp)->tv_sec += (uvp)->tv_sec;                         \
                (vvp)->tv_nsec += (uvp)->tv_nsec;                       \
                if ((vvp)->tv_nsec >= 1000000000) {                     \
                        (vvp)->tv_sec++;                                \
                        (vvp)->tv_nsec -= 1000000000;                   \
                }                                                       \
        } while (0)

#define timespecsub(vvp, uvp)                                           \
        do {                                                            \
                (vvp)->tv_sec -= (uvp)->tv_sec;                         \
                (vvp)->tv_nsec -= (uvp)->tv_nsec;                       \
                if ((vvp)->tv_nsec < 0) {                               \
                        (vvp)->tv_sec--;                                \
                        (vvp)->tv_nsec += 1000000000;                   \
                }                                                       \
        } while (0)

#define	PPU_NREG	8
#define	PPU_OAM_SIZE	0x100
#define	PPU_MEMMAP_SIZE	0x4000

#define	PPU_WIDTH	256
#define	PPU_HEIGHT	240

#define	PPU_TICKS_PER_FRAME	89342

struct ppu_pixel {
	uint8_t priority;
#define	PPU_PRIO_NONE		0
#define	PPU_PRIO_BEHIND		1
#define	PPU_PRIO_BG		2
#define	PPU_PRIO_FRONT		3
	uint8_t pal;	/* palette entry */
	uint8_t c;	/* colour value */
};

struct ppu_context {
	struct cpu_context *c;
	uint8_t	regs[PPU_NREG];
	uint16_t vramaddr;
	uint8_t oam[PPU_OAM_SIZE];
	uint8_t oamaddr;
	uint16_t scroll;

	uint8_t ppudata;

	uint8_t flags;
	/* Flags line up with "Flags 6" from iNES header */
#define	PPU_F_MIRROR_MASK	0x09
#define	PPU_F_MIRROR_H		0x00
#define	PPU_F_MIRROR_V		0x01
#define	PPU_F_MIRROR_NONE	0x08

	int latch_scroll;
	int latch_addr;

	struct timespec next_vblank;

	struct ppu_pixel pixels[PPU_HEIGHT][PPU_WIDTH];

	unsigned int frame;
	unsigned int frame_ticks;
	uint64_t ticks;

	int sprite0_hit;
	int xscroll;
	int yscroll;

	void (*draw)(struct ppu_context *);
	uint8_t (*read8)(uint16_t);
	void (*write8)(uint16_t, uint8_t);
};

int	ppu_init(struct ppu_context *);
uint8_t	ppu_read(struct ppu_context *, uint16_t);
void	ppu_write(struct ppu_context *, uint16_t, uint8_t);
int	ppu_step(struct ppu_context *);

#endif /* !_PPU_H */
