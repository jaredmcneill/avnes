PROG=	avnes
OBJS=	cpu.o ppu.o sdl.o main.o mapper.o \
	mapper_cnrom.o \
	mapper_mmc1.o \
	mapper_mmc2.o \
	mapper_mmc3.o \
	mapper_nrom.o \
	mapper_unrom.o

CC=	cc

SDL_CFLAGS!=	pkg-config --cflags sdl2
SDL_LIBS!=	pkg-config --libs sdl2

CFLAGS=	$(SDL_CFLAGS) -g -O2
LIBS=	$(SDL_LIBS)

all: $(PROG)

$(PROG): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(PROG) $(LIBS)

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o *.core $(PROG)
