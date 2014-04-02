# Makefile to build the SDL tests

srcdir  = .

CC      = gcc
EXE	= 
CFLAGS  = -g -O2 -I/usr/include/SDL2 -D_REENTRANT -DHAVE_OPENGL -std=c99
LIBS	= -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lSDL2 -lpthread -lm

TARGETS = \
	fg-haptic$(EXE) \
	test-haptic$(EXE)

all: $(TARGETS)

fg-haptic$(EXE): $(srcdir)/fg-haptic.c
	$(CC) -o $@ $? $(CFLAGS) $(LIBS)

test-haptic$(EXE): $(srcdir)/test-haptic.c
	$(CC) -o $@ $? $(CFLAGS) $(LIBS)

clean:
	rm -f $(TARGETS)

distclean: clean
	rm -f Makefile
	rm -f config.status config.cache config.log
	rm -rf $(srcdir)/autom4te*
