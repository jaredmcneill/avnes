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
#include "apu.h"
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
static const unsigned int ntscpalette_pal_len = 192;

static uint8_t joymap[256];

#define	SDL_NUM_CONTROLLERS	2

static SDL_GameController *controller[SDL_NUM_CONTROLLERS];
static SDL_JoystickID controller_id[SDL_NUM_CONTROLLERS];
static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_Texture *texture;
static uint8_t fbmem[PPU_WIDTH * PPU_HEIGHT * 4];
static int fullscreen = 0;
static int overscan = 0;
static SDL_Rect rect_overscan = { .x = 8, .y = 8, .w = PPU_WIDTH - 16, .h = PPU_HEIGHT - 16 };

static SDL_AudioDeviceID audiodev;
static float audio_pulse_table[31];
static const unsigned int audio_pulse_table_len = 31;
static float audio_tnd_table[203];
static const unsigned int audio_tnd_table_len = 203;

static void
sdl_init_audio(void)
{
	SDL_AudioSpec want, have;

	SDL_memset(&want, 0, sizeof(want));
	//want.freq = 48000;
	want.freq = 1789773;
	want.format = AUDIO_F32;
	want.channels = 1;
	want.samples = 32768;
	want.callback = NULL;

	audiodev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
	if (audiodev == 0) {
		SDL_Log("Failed to open audio: %s", SDL_GetError());
		return;
	}

	SDL_PauseAudioDevice(audiodev, 0);
}

int
sdl_init(const char *filename)
{
	char *window_title;
	int window_width, window_height;
	int controller_index, i, error;

	/* Emulate 8:7 PAR */
	window_width = PPU_WIDTH * 2 * 8 / 7;
	window_height = PPU_HEIGHT * 2;

	error = SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO|SDL_INIT_GAMECONTROLLER);
	if (error != 0) {
		fprintf(stderr, "Couldn't init SDL: %s\n", SDL_GetError());
		return ENXIO;
	}
	atexit(SDL_Quit);

	sdl_init_audio();

	for (i = 0; i < audio_pulse_table_len; i++)
		audio_pulse_table[i] = 95.52 / (8128.0 / i + 100);
	for (i = 0; i < audio_tnd_table_len; i++)
		audio_tnd_table[i] = 163.67 / (24329.0 / i + 100);

	for (i = 0; i < sizeof(joymap); i++)
		joymap[i] = 0;
	/* Button mapping. Yes, B is A... */
	joymap[SDL_CONTROLLER_BUTTON_A] = IO_BUTTON_B;
	joymap[SDL_CONTROLLER_BUTTON_B] = IO_BUTTON_A;
	joymap[SDL_CONTROLLER_BUTTON_BACK] = IO_BUTTON_SELECT;
	joymap[SDL_CONTROLLER_BUTTON_START] = IO_BUTTON_START;

	if (SDL_GameControllerAddMappingsFromFile("gamecontrollerdb.txt") == -1)
		fprintf(stderr, "Couldn't load game controller mappings from gamecontrollerdb.txt: %s\n", SDL_GetError());

	controller_index = 0;
	for (i = 0; i < SDL_NumJoysticks() && controller_index < SDL_NUM_CONTROLLERS; i++) {
		if (!SDL_IsGameController(i)) {
			char buf[33];
			SDL_JoystickGetGUIDString(SDL_JoystickGetDeviceGUID(i), buf, sizeof(buf));
			printf("Joystick #%d (%s) is not a game controller\n", i, buf);
			continue;
		}
		controller[controller_index] = SDL_GameControllerOpen(i);
		if (controller[controller_index] == NULL) {
			fprintf(stderr, "Couldn't open game controller %i: %s\n", i, SDL_GetError());
			continue;
		}
		controller_id[controller_index] = i;
		controller_index++;
	}
	for (i = 0; i < SDL_NUM_CONTROLLERS; i++)
		printf("Gamepad #%d: %s\n", i + 1, controller[i] ? SDL_GameControllerName(controller[i]) : "<none>");

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
	SDL_Rect *src;

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

	src = overscan ? &rect_overscan : NULL;

	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, src, NULL);
	SDL_RenderPresent(renderer);
}

void
sdl_play(struct apu_context *a)
{
	float sample, pulse_out, tnd_out;
	uint8_t pulse1, pulse2, triangle, noise, dmc;

	sample = 0.0;
	pulse1 = pulse2 = triangle = noise = dmc = 0;

	if (a->status.pulse_enable[0] && a->pulse[0].timer >= 8) {
		pulse1 = a->pulse[0].seqval ? a->pulse[0].vol_env_div_period : 0;
	}
	if (a->status.pulse_enable[1] && a->pulse[1].timer >= 8) {
		pulse2 = a->pulse[1].seqval ? a->pulse[1].vol_env_div_period : 0;
	}
	pulse_out = audio_pulse_table[pulse1 + pulse2];

	if (a->status.triangle_enable && a->triangle.timer >= 8)
		triangle = a->triangle.seqval;
	tnd_out = audio_tnd_table[3 * triangle + 2 * noise + dmc];

	sample = pulse_out + tnd_out;

	SDL_QueueAudio(audiodev, &sample, sizeof(sample));
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

static int
sdl_get_controller(SDL_JoystickID id)
{
	int i;

	for (i = 0; i < SDL_NUM_CONTROLLERS; i++) {
		if (controller[i] == NULL)
			continue;
		if (controller_id[i] == id)
			return i;
	}

	return -1;
}

int
sdl_poll(struct io_context *io)
{
	SDL_Event event;
	uint8_t btn;
	int pending, index;

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
			SDL_ShowCursor(fullscreen ? SDL_DISABLE : SDL_ENABLE);
			SDL_SetWindowFullscreen(window, fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
			return 0;
		}

		if (event.key.keysym.sym == SDLK_o && event.key.state == SDL_RELEASED) {
			overscan ^= 1;
			return 0;
		}

		btn = sdl_keytobutton(io, &event.key);

		if (event.key.state == SDL_PRESSED)
			io->state[0] |= btn;
		else if (event.key.state == SDL_RELEASED)
			io->state[0] &= ~btn;

		return 0;

	case SDL_CONTROLLERBUTTONDOWN:
	case SDL_CONTROLLERBUTTONUP:
		index = sdl_get_controller(event.caxis.which);
		if (index == -1)
			return 0;

		btn = joymap[event.cbutton.button];
		if (btn) {
			if (event.cbutton.state == SDL_PRESSED)
				io->state[index] |= btn;
			else
				io->state[index] &= ~btn;
		}

		return 0;

	case SDL_CONTROLLERAXISMOTION:
		index = sdl_get_controller(event.caxis.which);
		if (index == -1)
			return 0;

		switch (event.caxis.axis) {
		case SDL_CONTROLLER_AXIS_LEFTX:
			io->state[index] &= ~(IO_BUTTON_LEFT|IO_BUTTON_RIGHT);
			if (event.caxis.value <= -16384)
				io->state[index] |= IO_BUTTON_LEFT;
			else if (event.caxis.value >= 16387)
				io->state[index] |= IO_BUTTON_RIGHT;
			break;
		case SDL_CONTROLLER_AXIS_LEFTY:
			io->state[index] &= ~(IO_BUTTON_UP|IO_BUTTON_DOWN);
			if (event.caxis.value <= -16384)
				io->state[index] |= IO_BUTTON_UP;
			else if (event.caxis.value >= 16387)
				io->state[index] |= IO_BUTTON_DOWN;
			break;
		}
		return 0;

	}

	return 0;
}
