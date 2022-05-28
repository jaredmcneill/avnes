PROG=	avnes
OBJS=	cpu.o ppu.o apu.o sdl.o main.o mapper.o \
	mapper_axrom.o \
	mapper_cnrom.o \
	mapper_colordreams.o \
	mapper_gxrom.o \
	mapper_mmc1.o \
	mapper_mmc2.o \
	mapper_mmc3.o \
	mapper_nrom.o \
	mapper_unrom.o

CC=	cc

SDL_CFLAGS=	$(shell pkg-config --cflags sdl2)
SDL_LIBS=	$(shell pkg-config --libs sdl2)

OPSYS=		$(shell uname -s)

ifeq ($(OPSYS),Linux)
OS_CFLAGS+=	-D_GNU_SOURCE
endif

CFLAGS=		$(SDL_CFLAGS) $(OS_CFLAGS) -g -O2
LIBS=		$(SDL_LIBS)

all: $(PROG)

$(PROG): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(PROG) $(LIBS)

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o *.core $(PROG)
