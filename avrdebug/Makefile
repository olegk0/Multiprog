# Name: Makefile
# Project: AVR Programmer
# Author: Christian Starkjohann
# Creation Date: 2006-07-10
# Tabsize: 4
# Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
# License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)


CC              = gcc
LIBUSB_CONFIG   = libusb-config
# Make sure that libusb-config is in the search path or specify a full path.
# On Windows, there is no libusb-config and you must configure the options
# below manually. See examples.

USBFLAGS        = `$(LIBUSB_CONFIG) --cflags`
# On Windows replace `$(LIBUSB_CONFIG) --cflags` with appropriate "-I..."
# option to ensure that usb.h is found

USBLIBS         = `$(LIBUSB_CONFIG) --libs`
# You may need "-framework CoreFoundation" on Mac OS X and Darwin.
#USBLIBS        = -L/usr/local/libusb/lib/gcc -lusb
# On Windows use somthing similar to the line above.

# EXE_SUFFIX =	# can be overridden from outside

CFLAGS          = $(USBFLAGS) -O -Wall -I.
LIBS            = $(USBLIBS)

PROGRAM = avrdebug$(EXE_SUFFIX)
# PROGRAMHID may be defined by Makefile.windows which includes this Makefile

all: $(PROGRAM) $(PROGRAMHID)

.c.o:
	$(CC) $(CFLAGS) -c $<

$(PROGRAM): avrdebug.o
	$(CC) -o $(PROGRAM) avrdebug.o $(LIBS)

strip: all
	strip $(PROGRAM) $(PROGRAMHID)

clean:
	rm -f *.o
	rm -f avrdebug avrdebug.exe avrdebug-hid.exe
