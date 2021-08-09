// Airspy1090, a Mode 1/2/3/A/C/S messages decoder for Airspy devices.
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
// Copyright (C) 2021 by Malcolm Robb <support@attavionics.com>
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  *  Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//  *  Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#include "airspy1090.h"
//
// ============================= Utility functions ==========================
//
void sigintHandler(int dummy) {
    MODES_NOTUSED(dummy);
    signal(SIGINT, SIG_DFL);  // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
    pthread_cond_signal (&Modes.IF_cond);
    pthread_cond_signal (&Modes.AF_cond);
}
//
// =============================== Terminal handling ========================
//
#ifndef _WIN32
// Get the number of rows after the terminal changes size.
int getTermRows() { 
    struct winsize w; 
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w); 
    return (w.ws_row); 
} 

// Handle resizing terminal
void sigWinchCallback() {
    signal(SIGWINCH, SIG_IGN);
    Modes.interactive_rows = getTermRows();
    interactiveShowData();
    signal(SIGWINCH, sigWinchCallback); 
}
#else 
int getTermRows() { return MODES_INTERACTIVE_ROWS;}
#endif
//
// =============================== Initialization ===========================
//
void modesInitConfig(void) {
    // Default everything to zero/NULL
    memset(&Modes, 0, sizeof(Modes));

    // Now initialise things that should not be 0/NULL to their defaults
    Modes.gain                    = MODES_MAX_GAIN;
    Modes.freq                    = MODES_DEFAULT_FREQ;
    Modes.net_heartbeat_rate      = MODES_NET_HEARTBEAT_RATE;
    Modes.net_output_sbs_port     = MODES_NET_OUTPUT_SBS_PORT;
    Modes.net_output_avr_port     = MODES_NET_OUTPUT_AVR_PORT;
    Modes.net_output_raw_port     = MODES_NET_OUTPUT_RAW_PORT;
    Modes.net_output_beast_port   = MODES_NET_OUTPUT_BEAST_PORT;
    Modes.interactive_rows        = getTermRows();
    Modes.interactive_delete_ttl  = MODES_INTERACTIVE_DELETE_TTL;
    Modes.interactive_display_ttl = MODES_INTERACTIVE_DISPLAY_TTL;
    Modes.fUserLat                = MODES_USER_LATITUDE_DFLT;
    Modes.fUserLon                = MODES_USER_LONGITUDE_DFLT;
}
//
//=========================================================================
//
void modesInit(void) {
    int i;
	pthread_attr_t attr;

    // Allocate the various buffers used by Modes
    if ( ((Modes.icao_cache  = (pICAOCache)  malloc(sizeof(tICAOCache)  * MODES_ICAO_CACHE_LEN))                             == NULL) ||
         ((Modes.ModeA_cache = (pModeACache) malloc(sizeof(tModeACache) * MODEA_CACHE_LEN))                                  == NULL) ||
         ((Modes.pFileData   = (uint16_t *)  malloc((MODES_ASYNC_BUF_SAMPLES * sizeof(uint16_t)))                          ) == NULL) ||
         ((Modes.uIF_iDelay  = (uint16_t *)  malloc((MODES_ASYNC_BUF_SAMPLES+MODES_LONG_FRAME_SAMPLES) * sizeof(uint16_t)) ) == NULL) ||
         ((Modes.uAF_iDelay  = (uint16_t *)  malloc((MODES_ASYNC_BUF_SAMPLES+MODES_LONG_FRAME_SAMPLES) * sizeof(uint16_t)) ) == NULL) ||
         ((Modes.uAF_oDelay  = (uint16_t *)  malloc((                        MODES_LONG_FRAME_SAMPLES) * sizeof(uint16_t)) ) == NULL) ||
         ((Modes.beastOut    = (char     *)  malloc(MODES_RAWOUT_BUF_SIZE)                                                 ) == NULL) ||
         ((Modes.rawOut      = (char     *)  malloc(MODES_RAWOUT_BUF_SIZE)                                                 ) == NULL) ||
         ((Modes.avrOut      = (char     *)  malloc(MODES_RAWOUT_BUF_SIZE)                                                 ) == NULL) ) 
    {
        fprintf(stderr, "Out of memory allocating data buffer.\n");
        exit(1);
    }

    // Clear the buffers that have just been allocated, just in-case
    memset(Modes.icao_cache, 0,   sizeof(tICAOCache) * MODES_ICAO_CACHE_LEN);
    memset(Modes.ModeA_cache, 0,  sizeof(tModeACache) * MODEA_CACHE_LEN);
    memset(Modes.pFileData,127,   MODES_ASYNC_BUF_SIZE);
    memset(Modes.uIF_iDelay, 0, ((MODES_ASYNC_BUF_SAMPLES+MODES_LONG_FRAME_SAMPLES) * sizeof(uint16_t)));
    memset(Modes.uAF_iDelay, 0, ((MODES_ASYNC_BUF_SAMPLES+MODES_LONG_FRAME_SAMPLES) * sizeof(uint16_t)));
    memset(Modes.uAF_oDelay, 0, ((                        MODES_LONG_FRAME_SAMPLES) * sizeof(uint16_t)));

    for (i = 0; i < MODES_ASYNC_BUF_NUMBER; i++) {
        if ( ((Modes.IF_Fifo[i].pFifo = (uint16_t*)malloc((MODES_ASYNC_BUF_SAMPLES)                            * sizeof(uint16_t))) == NULL) ||
             ((Modes.AF_Fifo[i].pFifo = (uint16_t*)malloc((MODES_ASYNC_BUF_SAMPLES + MODES_LONG_FRAME_SAMPLES) * sizeof(uint16_t))) == NULL) )
        {
            fprintf(stderr, "Out of memory allocating data buffer.\n");
            exit(1);
        }
        memset(Modes.IF_Fifo[i].pFifo, 0, (MODES_ASYNC_BUF_SAMPLES                )            * sizeof(uint16_t));
        memset(Modes.AF_Fifo[i].pFifo, 0, (MODES_ASYNC_BUF_SAMPLES + MODES_LONG_FRAME_SAMPLES) * sizeof(uint16_t));
    }

    for (i = 0; i < MODES_DECODE_BUF_NUMBER; i++) {
        if ( ((Modes.pModeS_Buffer[i] = (tMessageRaw*) malloc(sizeof(tMessageRaw))) == NULL) ||
             ((Modes.pModeA_Buffer[i] = (tMessageRaw*) malloc(sizeof(tMessageRaw))) == NULL) )
        {
            fprintf(stderr, "Out of memory allocating data buffer.\n");
            exit(1);
        }
        memset(Modes.pModeS_Buffer[i], 0, sizeof(tMessageRaw));
        memset(Modes.pModeA_Buffer[i], 0, sizeof(tMessageRaw));
    }

    // Validate the users Lat/Lon home location inputs
    if ( (Modes.fUserLat >   90.0)  // Latitude must be -90 to +90
      || (Modes.fUserLat <  -90.0)  // and 
      || (Modes.fUserLon >  360.0)  // Longitude must be -180 to +360
      || (Modes.fUserLon < -180.0) ) {
        Modes.fUserLat = Modes.fUserLon = 0.0;
    } else if (Modes.fUserLon > 180.0) { // If Longitude is +180 to +360, make it -180 to 0
        Modes.fUserLon -= 360.0;
    }
    // If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the 
    // Atlantic ocean off the west coast of Africa. This is unlikely to be correct. 
    // Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian 
    // is at 0.0 Lon,so we must check for either fLat or fLon being non zero not both. 
    // Testing the flag at runtime will be much quicker than ((fLon != 0.0) || (fLat != 0.0))
    Modes.bUserFlags &= ~MODES_USER_LATLON_VALID;
    if ((Modes.fUserLat != 0.0) || (Modes.fUserLon != 0.0)) {
        Modes.bUserFlags |= MODES_USER_LATLON_VALID;
    }

    // Limit the maximum requested raw output size to less than one Ethernet Block 
    if (Modes.net_output_raw_size > (MODES_RAWOUT_BUF_FLUSH))
      {Modes.net_output_raw_size = MODES_RAWOUT_BUF_FLUSH;}
    if (Modes.net_output_raw_rate > (MODES_RAWOUT_BUF_RATE))
      {Modes.net_output_raw_rate = MODES_RAWOUT_BUF_RATE;}
    if (Modes.net_sndbuf_size > (MODES_NET_SNDBUF_MAX))
      {Modes.net_sndbuf_size = MODES_NET_SNDBUF_MAX;}

    // Initialise the Block Timers to something half sensible
    ftime(&Modes.stSystemTimeBlk);

    // Initialise the Threads and control mutex's and condex's
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	if (   pthread_mutex_init(&Modes.IF_mutex, NULL)
		|| pthread_mutex_lock(&Modes.IF_mutex)
		|| pthread_cond_init (&Modes.IF_cond,  NULL)
		|| pthread_create    (&Modes.IF_thread, &attr, IF_threadproc, NULL)) {
		return;
	}

	if (   pthread_mutex_init(&Modes.AF_mutex, NULL)
		|| pthread_mutex_lock(&Modes.AF_mutex)
		|| pthread_cond_init (&Modes.AF_cond, NULL)
		|| pthread_create    (&Modes.AF_thread, &attr, ModeS_threadproc, NULL)) {
		return;
	}

	if (   pthread_mutex_init(&Modes.ModeA_Go_mutex, NULL)
		|| pthread_mutex_lock(&Modes.ModeA_Go_mutex)
        || pthread_mutex_init(&Modes.ModeA_Done_mutex, NULL)
		|| pthread_mutex_lock(&Modes.ModeA_Done_mutex)
		|| pthread_cond_init(&Modes.ModeA_Go_cond, NULL)
		|| pthread_cond_init(&Modes.ModeA_Done_cond, NULL)
		|| pthread_create(&Modes.ModeA_thread, &attr, ModeA_threadproc, NULL)) {
		return;
	}

	if (   pthread_mutex_init(&Modes.Decode_mutex, NULL)
		|| pthread_mutex_lock(&Modes.Decode_mutex)
		|| pthread_cond_init (&Modes.Decode_cond, NULL)
		|| pthread_create    (&Modes.Decode_thread, &attr, Decode_threadproc, NULL)) {
		return;
	}

	pthread_attr_destroy(&attr);
}
//
//=========================================================================
//
void modesShutdown(void) {

    Modes.exit = 1;

	// Set all the condexs to allow the threads to run and terminate
	pthread_cond_signal(&Modes.IF_cond);
	pthread_cond_signal(&Modes.AF_cond);
	pthread_cond_signal(&Modes.ModeA_Go_cond);
	pthread_cond_signal(&Modes.ModeA_Done_cond);
	pthread_cond_signal(&Modes.Decode_cond);

	// Wait for threads to terminate
	pthread_join(Modes.IF_thread, NULL);
	pthread_join(Modes.AF_thread, NULL);
	pthread_join(Modes.ModeA_thread, NULL);
	pthread_join(Modes.Decode_thread, NULL);

	// Destroy the condexs and mutexs
	pthread_cond_destroy (&Modes.IF_cond);
	pthread_mutex_destroy(&Modes.IF_mutex);
	pthread_cond_destroy (&Modes.AF_cond);
	pthread_mutex_destroy(&Modes.AF_mutex);
	pthread_cond_destroy (&Modes.ModeA_Go_cond);
	pthread_mutex_destroy(&Modes.ModeA_Go_mutex);
	pthread_cond_destroy (&Modes.ModeA_Done_cond);
	pthread_mutex_destroy(&Modes.ModeA_Done_mutex);
	pthread_cond_destroy (&Modes.Decode_cond);
	pthread_mutex_destroy(&Modes.Decode_mutex);

}
//
// ================================ Main ====================================
//
void showHelp(void) {
    printf(
"-----------------------------------------------------------------------------\n"
"|                Airspy1090 Mode 1/2/3/A/C/S Receiver    Ver : " MODES_AIRSPY1090_VERSION " |\n"
"-----------------------------------------------------------------------------\n"
"--device-index <index>   Select RTL device (default: 0)\n"
"--gain <db>              Set gain (default: max gain. Use -10 for auto-gain)\n"
"--enable-agc             Enable the Automatic Gain Control (default: off)\n"
"--freq <hz>              Set frequency (default: 1090 Mhz)\n"
"--ifile <filename>       Read data from file (use '-' for stdin)\n"
"--interactive            Interactive mode refreshing data on screen\n"
"--interactive-rows <num> Max number of rows in interactive mode (default: 15)\n"
"--interactive-ttl <sec>  Remove from list if idle for <sec> (default: 60)\n"
"--modeac                 Enable decoding of SSR Modes 3/A & 3/C\n"
"--net-beast              TCP raw output in Beast binary format\n"
"--net-bind-address <ip>  IP address to bind to (default: Any; Use 127.0.0.1 for private)\n"
"--net-ro-port <port>     TCP raw output listen port (default: 30002)\n"
"--net-sbs-port <port>    TCP BaseStation output listen port (default: 30003)\n"
"--net-bo-port <port>     TCP Beast output listen port (default: 30005)\n"
"--net-avr-port <port>    TCP AVR output listen port (default: 30006)\n"
"--net-ro-size <size>     TCP raw output minimum size (default: 0)\n"
"--net-ro-rate <rate>     TCP raw output memory flush rate (default: 0)\n"
"--net-heartbeat <rate>   TCP heartbeat rate in seconds (default: 60 sec; 0 to disable)\n"
"--net-buffer <n>         TCP buffer size 64Kb * (2^n) (default: n=0, 64Kb)\n"
"--lat <latitude>         Reference/receiver latitude for surface posn (opt)\n"
"--lon <longitude>        Reference/receiver longitude for surface posn (opt)\n"
"--quiet                  Disable output to stdout. Use for daemon applications\n"
"--help                   Show this help\n"
"\n"
    );
}

#ifdef _WIN32
void showCopyright(void) {
    time_t llTime = time(NULL) + 1;

    printf(
"-----------------------------------------------------------------------------\n"
"|                Airspy1090 Mode 1/2/3/A/C/S Receiver    Ver : " MODES_AIRSPY1090_VERSION " |\n"
"-----------------------------------------------------------------------------\n"
"\n"
" Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>\n"
" Copyright (C) 2014 by Malcolm Robb <support@attavionics.com>\n"
" Copyright (C) 2021 by Malcolm Robb <support@attavionics.com>\n"
"\n"
" All rights reserved.\n"
"\n"
" THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n"
" ""AS IS"" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n"
" LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR\n"
" A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT\n"
" HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,\n"
" SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT\n"
" LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,\n"
" DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY\n"
" THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT\n"
" (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE\n"
" OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n"
"\n"
" For further details refer to <https://github.com/MalcolmRobb/Airspy1090>\n" 
"\n"
    );

  // delay for 1 second to give the user a chance to read the copyright
  while (llTime >= time(NULL)) {}
}
#endif

//
//=========================================================================
//
// This function is called a few times every second by main in order to
// perform tasks we need to do continuously, like accepting new clients
// from the net, refreshing the screen in interactive mode, and so forth
//
void backgroundTasks(void) {

	netReadFromClients();

    // If Modes.aircrafts is not NULL, remove any stale aircraft
    if (Modes.pAircraft) {
        interactiveRemoveStaleAircrafts();
    }

    // Refresh screen when in interactive mode
    if (Modes.interactive) {
        interactiveShowData();
    }
}
//
//=========================================================================
//
int verbose_device_search(char *s)
{
    UNUSED (s);
/*
	int i, device_count, device, offset;
	char *s2;
	char vendor[256], product[256], serial[256];
	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		return -1;
	}
	fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	}
	fprintf(stderr, "\n");
	// does string look like raw id number
	device = (int)strtol(s, &s2, 0);
	if (s2[0] == '\0' && device >= 0 && device < device_count) {
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	// does string exact match a serial
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strcmp(s, serial) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	// does string prefix match a serial
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strncmp(s, serial, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	// does string suffix match a serial
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		offset = strlen(serial) - strlen(s);
		if (offset < 0) {
			continue;}
		if (strncmp(s, serial+offset, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	fprintf(stderr, "No matching devices found.\n"); */
	return -1;
}
//
//=========================================================================
//
int main(int argc, char **argv) {
    int j;

    // Set sane defaults
    modesInitConfig();
    signal(SIGINT, sigintHandler); // Define Ctrl/C handler (exit program)

    // Parse the command line options
    for (j = 1; j < argc; j++) {
        int more = j+1 < argc; // There are more arguments

        if (!strcmp(argv[j],"--device-index") && more) {
            Modes.dev_index = verbose_device_search(argv[++j]);
        } else if (!strcmp(argv[j],"--gain") && more) {
            Modes.gain = (int) (atof(argv[++j])*10); // Gain is in tens of DBs
        } else if (!strcmp(argv[j],"--enable-agc")) {
            Modes.enable_agc++;
        } else if (!strcmp(argv[j],"--freq") && more) {
            Modes.freq = (int) strtoll(argv[++j],NULL,10);
        } else if (!strcmp(argv[j],"--ifile") && more) {
            Modes.filename = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--modeac")) {
            Modes.mode_ac = 1;
        } else if (!strcmp(argv[j],"--net-beast")) {
            Modes.beast = 1;
        } else if (!strcmp(argv[j],"--net-heartbeat") && more) {
            Modes.net_heartbeat_rate = atoi(argv[++j]) * 15;
        } else if (!strcmp(argv[j],"--net-ro-size") && more) {
            Modes.net_output_raw_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ro-rate") && more) {
            Modes.net_output_raw_rate = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ro-port") && more) {
            if (Modes.beast) // Required for legacy backward compatibility
                {Modes.net_output_beast_port = atoi(argv[++j]);;}
            else
                {Modes.net_output_raw_port = atoi(argv[++j]);}
        } else if (!strcmp(argv[j],"--net-avr-port") && more) {
            Modes.net_output_avr_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bo-port") && more) {
            Modes.net_output_beast_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bind-address") && more) {
            Modes.net_bind_address = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-sbs-port") && more) {
            Modes.net_output_sbs_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-buffer") && more) {
            Modes.net_sndbuf_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--interactive")) {
            Modes.interactive = 1;
        } else if (!strcmp(argv[j],"--interactive-rows") && more) {
            Modes.interactive_rows = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--interactive-ttl") && more) {
            Modes.interactive_display_ttl = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--lat") && more) {
            Modes.fUserLat = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--lon") && more) {
            Modes.fUserLon = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--help")) {
            showHelp();
            exit(0);
        } else if (!strcmp(argv[j],"--quiet")) {
            Modes.quiet = 1;
        } else {
            fprintf(stderr,
                "Unknown or not enough arguments for option '%s'.\n\n",
                argv[j]);
            showHelp();
            exit(1);
        }
    }

#ifdef _WIN32
    // Try to comply with the Copyright license conditions for binary distribution
    if (!Modes.quiet) {showCopyright();}
#endif

#ifndef _WIN32
    // Setup for SIGWINCH for handling lines
    if (Modes.interactive) {signal(SIGWINCH, sigWinchCallback);}
#endif

    // Initialization
    modesInit();

    if (Modes.filename == NULL) {
        airspy_InitDevice();
    } else {
        if ((Modes.fd = open(Modes.filename,
#ifdef _WIN32
                                    (O_RDONLY | O_BINARY)
#else
                                    (O_RDONLY)
#endif
                                    )) == -1) {
            perror("Opening data file");
            exit(1);
        }
    }

    netInit();
/*
    {
        FILE* hFile;
        if ((hFile = fopen("airspy1090_u17.txt", "w"))) {
            fclose(hFile);
        }
    }
*/
    // Start reading Airspy data or File Data 
    if (Modes.filename == NULL) {
 	    airspy_start_rx(Modes.dev);
    } else {
        airspy_start_replay();
    }

    while (!Modes.exit) {
        backgroundTasks();
        usleep(100000);
    }

    if (Modes.filename == NULL) {
		airspy_close(Modes.dev);
    }

    modesShutdown();

#ifdef _WIN32
	return (0);
#else
	pthread_exit(0);
#endif
}
//
//=========================================================================
//
