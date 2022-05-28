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

#include <errno.h>
#include <stdio.h>

#include "avnes.h"
#include "mapper.h"

extern struct mapper_impl MAPPER_IMPL(axrom);
extern struct mapper_impl MAPPER_IMPL(cnrom);
extern struct mapper_impl MAPPER_IMPL(colordreams);
extern struct mapper_impl MAPPER_IMPL(gxrom);
extern struct mapper_impl MAPPER_IMPL(mmc1);
extern struct mapper_impl MAPPER_IMPL(mmc2);
extern struct mapper_impl MAPPER_IMPL(mmc3);
extern struct mapper_impl MAPPER_IMPL(nrom);
extern struct mapper_impl MAPPER_IMPL(unrom);

static struct mapper_impl *mappers[] = {
	[0] = &MAPPER_IMPL(nrom),
	[1] = &MAPPER_IMPL(mmc1),
	[2] = &MAPPER_IMPL(unrom),
	[3] = &MAPPER_IMPL(cnrom),
	[4] = &MAPPER_IMPL(mmc3),
	[7] = &MAPPER_IMPL(axrom),
	[9] = &MAPPER_IMPL(mmc2),
	[11] = &MAPPER_IMPL(colordreams),
	[66] = &MAPPER_IMPL(gxrom),
};

int
mapper_init(struct mapper_context *m, const uint8_t *data, size_t datalen)
{
	uint8_t mapper_number = (data[6] >> 4) | (data[7] & 0xf0);
	struct mapper_impl *impl;
	int error;

	if (mapper_number >= nitems(mappers) || mappers[mapper_number] == NULL) {
		printf("Mapper #%d not supported\n", mapper_number);
		return ENOENT;
	}

	impl = mappers[mapper_number];
	error = impl->init(m, data, datalen);
	if (error != 0) {
		printf("Mapper #%d (%s) init failed: %d\n", mapper_number, impl->descr, error);
		return error;
	}

	printf("Using mapper #%d (%s)\n", mapper_number, impl->descr);

	return 0;
}
