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
#include <stdio.h>
#include <errno.h>

#include <SDL.h>

#include "io.h"
#include "ppu.h"
#include "sdl.h"

static uint8_t ntscpalette_pal[] = {
  0x52, 0x52, 0x52, 0x01, 0x1a, 0x51, 0x0f, 0x0f, 0x65, 0x23, 0x06, 0x63,
  0x36, 0x03, 0x4b, 0x40, 0x04, 0x26, 0x3f, 0x09, 0x04, 0x32, 0x13, 0x00,
  0x1f, 0x20, 0x00, 0x0b, 0x2a, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x2e, 0x0a,
  0x00, 0x26, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xa0, 0xa0, 0xa0, 0x1e, 0x4a, 0x9d, 0x38, 0x37, 0xbc, 0x58, 0x28, 0xb8,
  0x75, 0x21, 0x94, 0x84, 0x23, 0x5c, 0x82, 0x2e, 0x24, 0x6f, 0x3f, 0x00,
  0x51, 0x52, 0x00, 0x31, 0x63, 0x00, 0x1a, 0x6b, 0x05, 0x0e, 0x69, 0x2e,
  0x10, 0x5c, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xfe, 0xff, 0xff, 0x69, 0x9e, 0xfc, 0x89, 0x87, 0xff, 0xae, 0x76, 0xff,
  0xce, 0x6d, 0xf1, 0xe0, 0x70, 0xb2, 0xde, 0x7c, 0x70, 0xc8, 0x91, 0x3e,
  0xa6, 0xa7, 0x25, 0x81, 0xba, 0x28, 0x63, 0xc4, 0x46, 0x54, 0xc1, 0x7d,
  0x56, 0xb3, 0xc0, 0x3c, 0x3c, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xfe, 0xff, 0xff, 0xbe, 0xd6, 0xfd, 0xcc, 0xcc, 0xff, 0xdd, 0xc4, 0xff,
  0xea, 0xc0, 0xf9, 0xf2, 0xc1, 0xdf, 0xf1, 0xc7, 0xc2, 0xe8, 0xd0, 0xaa,
  0xd9, 0xda, 0x9d, 0xc9, 0xe2, 0x9e, 0xbc, 0xe6, 0xae, 0xb4, 0xe5, 0xc7,
  0xb5, 0xdf, 0xe4, 0xa9, 0xa9, 0xa9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static unsigned int ntscpalette_pal_len = 192;

static uint8_t joymap[256];

static SDL_Joystick *joy;
static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_Texture *texture;
static uint8_t fbmem[PPU_WIDTH * PPU_HEIGHT * 4];
static int fullscreen = 0;

int
sdl_init(const char *filename)
{
	char *window_title;
	int window_width, window_height;
	int error;

	/* Emulate 8:7 PAR */
	window_width = PPU_WIDTH * 2 * 8 / 7;
	window_height = PPU_HEIGHT * 2;

	error = SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK);
	if (error != 0) {
		fprintf(stderr, "Couldn't init SDL: %s\n", SDL_GetError());
		return ENXIO;
	}
	atexit(SDL_Quit);

	if (SDL_NumJoysticks() > 0) {
		joy = SDL_JoystickOpen(0);
		if (joy) {
			printf("Using Joystick 0 (%s) for controller port 1\n", SDL_JoystickName(joy));

			for (int i = 0; i < sizeof(joymap); i++)
				joymap[i] = 0x00;
			joymap[0] = IO_BUTTON_START;
			joymap[1] = IO_BUTTON_SELECT;
			joymap[7] = IO_BUTTON_B;
			joymap[8] = IO_BUTTON_A;
		}
	}

	error = asprintf(&window_title, "avnes - %s", filename);
	if (error == -1) {
		fprintf(stderr, "Couldn't allocate memory for window title\n");
		return ENOMEM;
	}

	window = SDL_CreateWindow(window_title,
	    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
	    window_width, window_height,
	    SDL_WINDOW_SHOWN);
	if (!window) {
		fprintf(stderr, "Couldn't create SDL window: %s\n", SDL_GetError());
		return ENXIO;
	}

	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (!renderer) {
		fprintf(stderr, "Couldn't create SDL renderer: %s\n", SDL_GetError());
		return ENXIO;
	}
	if (SDL_RenderSetLogicalSize(renderer, window_width, window_height) != 0) {
		fprintf(stderr, "Couldn't set SDL renderer logical resolution: %s\n", SDL_GetError());
		return ENXIO;
	}

	texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, PPU_WIDTH, PPU_HEIGHT);
	if (!texture) {
		fprintf(stderr, "Couldn't create SDL texture: %s\n", SDL_GetError());
		return ENXIO;
	}

	return 0;
}

void
sdl_draw(struct ppu_context *p)
{
	uint8_t *fb = fbmem;

	for (unsigned int y = 0; y < PPU_HEIGHT; y++) {
		for (unsigned int x = 0; x < PPU_WIDTH; x++) {
			struct ppu_pixel *pix = &p->pixels[y][x];

			*fb++ = ntscpalette_pal[(pix->c * 3) + 2];
			*fb++ = ntscpalette_pal[(pix->c * 3) + 1];
			*fb++ = ntscpalette_pal[(pix->c * 3) + 0];
			*fb++ = 0xff;
		}
	}

	SDL_UpdateTexture(texture, NULL, fbmem, PPU_WIDTH * 4);

	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, NULL, NULL);
	SDL_RenderPresent(renderer);
}

static uint8_t
sdl_keytobutton(struct io_context *io, SDL_KeyboardEvent *key)
{
	uint8_t btn = 0;

	switch (key->keysym.sym) {
	case SDLK_a:
		btn = IO_BUTTON_A;
		break;
	case SDLK_b:
		btn = IO_BUTTON_B;
		break;
	case SDLK_BACKSPACE:
		btn = IO_BUTTON_SELECT;
		break;
	case SDLK_RETURN:
		btn = IO_BUTTON_START;
		break;
	case SDLK_UP:
		btn = IO_BUTTON_UP;
		break;
	case SDLK_DOWN:
		btn = IO_BUTTON_DOWN;
		break;
	case SDLK_LEFT:
		btn = IO_BUTTON_LEFT;
		break;
	case SDLK_RIGHT:
		btn = IO_BUTTON_RIGHT;
		break;
	}

	return btn;
}

int
sdl_poll(struct io_context *io)
{
	SDL_Event event;
	uint8_t btn;
	int pending;

	pending = SDL_PollEvent(&event);
	if (!pending)
		return -1;

	switch (event.type) {
	case SDL_QUIT:
		printf("Shutting down\n");
		return 1;

	case SDL_KEYDOWN:
	case SDL_KEYUP:

		if (event.key.keysym.sym == SDLK_ESCAPE)
			return 1;

		if (event.key.keysym.sym == SDLK_f && event.key.state == SDL_RELEASED) {
			fullscreen ^= 1;
			SDL_SetWindowFullscreen(window, fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
			return 0;
		}

		btn = sdl_keytobutton(io, &event.key);

		if (event.key.state == SDL_PRESSED)
			io->state[0] |= btn;
		else if (event.key.state == SDL_RELEASED)
			io->state[0] &= ~btn;

		return 0;

	case SDL_JOYBUTTONDOWN:
	case SDL_JOYBUTTONUP:
		btn = joymap[event.jbutton.button];
		if (btn) {
			if (event.jbutton.state == SDL_PRESSED)
				io->state[0] |= btn;
			else
				io->state[0] &= ~btn;
		}

		return 0;

	case SDL_JOYAXISMOTION:
		switch (event.jaxis.axis) {
		case 0:
			io->state[0] &= ~(IO_BUTTON_LEFT|IO_BUTTON_RIGHT);
			if (event.jaxis.value == -32768)
				io->state[0] |= IO_BUTTON_LEFT;
			else if (event.jaxis.value == 32767)
				io->state[0] |= IO_BUTTON_RIGHT;
			break;
		case 1:
			io->state[0] &= ~(IO_BUTTON_UP|IO_BUTTON_DOWN);
			if (event.jaxis.value == -32768)
				io->state[0] |= IO_BUTTON_DOWN;
			else if (event.jaxis.value == 32767)
				io->state[0] |= IO_BUTTON_UP;
			break;
		}
		return 0;

	}

	return 0;
}
