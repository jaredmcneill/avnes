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

#ifndef _ROM_H
#define _ROM_H

#define	ROM_HEADER_LENGTH	16
#define	ROM_TRAINER_LENGTH	512

#define	ROM_HAS_TRAINER(data)	((data[6] & 0x04) != 0)
#define	ROM_PRG_START(data)	(ROM_HEADER_LENGTH + (ROM_HAS_TRAINER(data) ? ROM_TRAINER_LENGTH : 0))
#define	ROM_PRG_LENGTH(data)	(data[4] * 0x4000)
#define	ROM_CHR_START(data)	(ROM_PRG_START(data) + ROM_PRG_LENGTH(data))
#define	ROM_CHR_LENGTH(data)	(data[5] * 0x2000)

#define	ROM_MIRROR_MASK		0x9
#define	ROM_MIRROR_H		0x0
#define	ROM_MIRROR_V		0x1

#endif /* !_ROM_H */
