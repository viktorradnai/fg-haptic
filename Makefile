# Makefile to build the SDL tests

srcdir  = .

# Parameters tosupport different library versions
SDLVER = sdl2
EXE	= 

PKGS	= ${SDLVER}

# Compiler parameters etc
CC      = gcc
CFLAGS  = -g -O2 -Wall -std=c99 $(foreach pkg,$(PKGS),$(shell pkg-config --cflags $(pkg)))
DEFS	= -D_POSIX_C_SOURCE -DSDL_${SDLVER}
LIBS	= -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lpthread -lm $(foreach pkg,$(PKGS),$(shell pkg-config --libs $(pkg)))

TARGETS = \
	fg-haptic$(EXE) \
	test-haptic$(EXE)

all: $(TARGETS)

fg-haptic$(EXE): $(srcdir)/fg-haptic.c
	$(CC) -o $@ $? $(DEFS) $(CFLAGS) $(LIBS)

test-haptic$(EXE): $(srcdir)/test-haptic.c
	$(CC) -o $@ $? $(DEFS) $(CFLAGS) $(LIBS)

clean:
	rm -f $(TARGETS)

distclean: clean
	rm -f Makefile
	rm -f config.status config.cache config.log
	rm -rf $(srcdir)/autom4te*
