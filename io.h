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

#ifndef _IO_H
#define _IO_H

#define	IO_BUTTON_A		(1 << 7)
#define	IO_BUTTON_B		(1 << 6)
#define	IO_BUTTON_SELECT	(1 << 5)
#define	IO_BUTTON_START		(1 << 4)
#define	IO_BUTTON_UP		(1 << 3)
#define	IO_BUTTON_DOWN		(1 << 2)
#define	IO_BUTTON_LEFT		(1 << 1)
#define	IO_BUTTON_RIGHT		(1 << 0)

#define	IO_CONTROLLER_1		0
#define	IO_CONTROLLER_2		1
#define	IO_NCONTROLLER		2

struct io_context {
	uint8_t	state[IO_NCONTROLLER];
	uint8_t shift[IO_NCONTROLLER];
};

#endif /* !_IO_H */
