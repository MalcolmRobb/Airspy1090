#
# When building a package or installing otherwise in the system, make
# sure that the variable PREFIX is defined, e.g. make PREFIX=/usr/local
#
PROGNAME=airspy1090

ifdef PREFIX
BINDIR=$(PREFIX)/bin
SHAREDIR=$(PREFIX)/share/$(PROGNAME)
EXTRACFLAGS=-DHTMLPATH=\"$(SHAREDIR)\"
endif

ASMFLAGS=-al -march=armv7-a
CFLAGS=-O2 -mcpu=cortex-a53 -g -Wall -W `pkg-config --cflags libusb-1.0`
LIBS=`pkg-config --libs libusb-1.0` -lpthread -lm
ASM=as
CC=gcc

all: airspy1090

%.o: %.c
	$(CC) $(CFLAGS) $(EXTRACFLAGS) -c $<

%.o: %.s
	$(ASM) $(ASMFLAGS) $< -o $(@) > $^.lst

airspy1090: airspy1090.o airspy.o anet.o interactive.o mode_ac.o mode_s.o net_io.o demodulate.o detectModeS.o detectModeA.o decode.o
	$(CC) -g -o airspy1090 airspy1090.o airspy.o anet.o interactive.o mode_ac.o mode_s.o net_io.o demodulate.o detectModeS.o detectModeA.o decode.o $(LIBS) $(LDFLAGS)

clean:
	rm -f *.o airspy1090
