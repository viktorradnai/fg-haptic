# Makefile to build the SDL tests

srcdir  = .

CC      = gcc
EXE	= 
CFLAGS  = -g -O2 -I/usr/local/include/SDL -D_REENTRANT -DHAVE_OPENGL -std=c99
LIBS	=  -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lSDL -lpthread

TARGETS = \
	testhaptic$(EXE) \
	haptic$(EXE)

all: $(TARGETS)

testhaptic$(EXE): $(srcdir)/testhaptic.c
	$(CC) -o $@ $? $(CFLAGS) $(LIBS)

haptic$(EXE): $(srcdir)/haptic.c
	$(CC) -o $@ $? $(CFLAGS) $(LIBS)

clean:
	rm -f $(TARGETS)

distclean: clean
	rm -f Makefile
	rm -f config.status config.cache config.log
	rm -rf $(srcdir)/autom4te*
