Airspy1090 README
===

Airspy 1090 is a Mode A/C/S decoder specifically designed for Airspy R2 devices.

The main features are:

* Robust decoding of weak messages.
* Network support: TCP30003 stream (MSG5...), Raw packets, HTTP.
* Ability to decode DF11, DF17, DF18 messages.
* Ability to decode DF formats like DF0, DF4, DF5, DF16, DF20 and DF21
  where the checksum is xored with the ICAO address by brute forcing the
  checksum field using recently seen ICAO addresses.
* Decode raw samples from file (using --ifile command line switch).
* Interactive command-line-interfae mode where aircrafts currently detected
  are shown as a list refreshing as more data arrives.
* CPR coordinates decoding and track calculation from velocity.
* TCP server streaming to connected clients

Installation
---

Type "make".

Normal usage
---

To capture traffic directly from your Airspy device and show the captured traffic
on standard output, just run the program without options at all:

    ./airspy1090

To run the program in interactive mode:

    ./airspy1090 --interactive

In interactive mode it is possible to have a less information dense but more
"arcade style" output, where the screen is refreshed every second displaying
all the recently seen aircrafts with some additional information such as
altitude and flight number, extracted from the received Mode S packets.

Using files as source of data
---

To decode data from file, use:

    ./airspy1090 --ifile /path/to/binfile

Additional options
---

airspy1090 can be called with other command line options to set a different
gain, frequency, and so forth. For a list of options use:

    ./airspy1090 --help

Everything is not documented here should be obvious, and for most users calling
it without arguments at all is the best thing to do.

Reliability
---

By default airspy1090 checks for decoding errors using the 24-bit CRC checksum,
where available. Messages with errors are discarded. CRC bit fixing is not, and
will not be supported. EVER! It introduces too many false decodes.

Network server features
---

When the program is started, it starts listening for clients connections on ports
 30002, 30003, 30005 and 30006 (you can change any of the ports if you want, see 
--help output).

Port 30002
---

Connected clients are served with raw data as it arrives from the device
(or from file if --ifile is used) in the raw format similar to the following:

    *8D451E8B99019699C00B0A81F36E;

Every entry is separated by a simple newline (LF character, hex 0x0A).

Port 30003
---

Connected clients are served with messages in SBS1 (BaseStation) format,
similar to:

    MSG,4,,,738065,,,,,,,,420,179,,,0,,0,0,0,0
    MSG,3,,,738065,,,,,,,35000,,,34.81609,34.07810,,,0,0,0,0

This can be used to feed data to various sharing sites without the need to use another decoder.

Port 30005
---

Connected clients are served with Beast style data as it arrives from the device
(or from file if --ifile is used).

Port 30006
---

Connected clients are served with AVR style data as it arrives from the device
(or from file if --ifile is used).

How this program works?
---

The code is commented reasonably well, but non the less I realise it can be daunting. If
you want anything explaining drop me an email, or find me on one of the message boards.

I have tested the code using Microsoft Visual Studio C++ 6.0, Microsoft Visual Studio C++ 
2019, and on an RPi3B+ (Raspbian 8.3.0-6+rpi1) using gcc 8.3.0. 

There are some warnings when building for the RPi, but I've left them in to remind me 
there is inline assembler in there. The Microsoft 2019 package can compile both x86(32bit) 
and x64(64 bit) versions to run on the Windows platform.

The code started off as a branch of dump1090, but it soon became apparant that the increased
data rate from an Airspy(20 MSPS @ 12 bit) over that from an RTL-SDR dongle (2MSPS @ 8 bit)
means that a single threaded program couldn't work on an RPi type device - even an RPi4 
doesn't have the grunt to deal with the data rate.

Therefore airspy1090 was rewritten dump1090 by braking it down onto 4 basic modules, all 
running in their own separate thread. The threads Are :

1. The IF Demodulator (demodulate.c). 
This takes the raw (20MSPS @ 12 bit) samples from Airspy (libusb), and demodulates them from 
what is effectivly a 5MHz Intermediate Frequency to a baseband Audio Frequency. This is done
in the classic AM demodulation way - High pass filter the IF to remove any DC component, 
Rectify, and then Lowpass filter to AM.

The RPi doesn't have the grunt to implement a proper FIR filter, so a simple 5 tap box filter
is used for the HPF, and a 10 tap box filter is used for the LPF. The function is available
in both standard C and ARMv7 inline assembler. The C version uses about 50% of a single core 
in an RPi3B+, whilst the Assembler uses about 40%.

Once a whole buffer is processeed from IF to AF it is placed the AF Fifo so that a separate 
thread can continue processing, and the IF thread can deal with the next block.

2. Mode S Detection (detectModeS.c)
This module takes the AF buffers (from demodulate.c) and extracts any ModeS frames from the 
data. If ModeA decoding is enabled, it also 'kicks' the ModeA thread so that it processes
the same buffer of data.

The ModeS decoding is the most CPU intensive part of the program. There are 3 versions of the
decoder - one written in C, one written in Armv7 assembler using SIMD instructions, and a third
written in assembler using NEON instructions. Unfortunatley, the only version that works on 
an RPi3B+ is the NEON assembler version - and it still uses about 90% of a single CPU core. The
SIMD version uses about 130% of a single core, and the C version about 250%. 

Decoded ModeS frames are then placed into the ModeS FIFO queue of the decoder.

3. Mode A Detection (detectModeA.c)
If Mode A decoding is enabled (--modeac on the command line) then the ModeS detector thread
kicks the ModeA thread into life. The ModeA thread scans the AF data buffer for potential 
ModeA signals. The ModeA detection code is also available in three forms - Standard C, 
SIMD and NEON. 

The NEON version takes about 50% of a single RPi3B+ core, and the C version about 95%. Since
NEON is required for ModeS to work, you may as well use NEON for mode A too.

Decoded ModeA frames are then placed into the ModeA FIFO queue of the decoder.

4. Final Decoding (decode.c)
This module takes the data from the ModeA and ModeS decoder fifos, and sorts them into time order
before decoding the required parameters and passing them to the output and display functions.

Contributing
---

Please don't request display changes or updates. The program is intended as a decoder only. If you
want to display things in a different way, then please investigate programs like View1090, or 
better still Planeplotter. Other plane tracking software is available.

Credits
---
airspy1090 was written by Malcolm Robb <support@attavionics.com> and is
released under the BSD three clause license.
dump1090 was written by Salvatore Sanfilippo <antirez@gmail.com> and is
released under the BSD three clause license.
