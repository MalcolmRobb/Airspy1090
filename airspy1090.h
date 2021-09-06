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
#ifndef __AIRSPY1090_H
#define __AIRSPY1090_H

// File Version number
// ====================
// Format is : MajorVer.MinorVer.DayMonth.Year"
// MajorVer changes only with significant changes
// MinorVer changes when additional features are added, but not for bug fixes (range 00-99)
// DayDate & Year changes for all changes, including for bug fixes. It represent the release date of the update
//
#define MODES_AIRSPY1090_VERSION     "1.0.0709.21"

// ============================= Include files ==========================

#ifndef _WIN32
//  #define _ARMSIMD (1)
    #define _ARMNEON (1)
    #define _ARMASM  (1)

    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include <pthread.h>
    #include <stdint.h>
    #include <stdbool.h>
    #include <errno.h>
    #include <unistd.h>
    #include <math.h>
    #include <sys/time.h>
    #include <sys/timeb.h>
    #include <signal.h>
    #include <fcntl.h>
    #include <ctype.h>
    #include <sys/stat.h>
    #include <sys/ioctl.h>

typedef union _LARGE_INTEGER {
    struct {
        uint32_t LowPart;
        int32_t  HighPart;
    };
    struct {
        uint32_t LowPart;
        int32_t  HighPart;
    } u;
    int64_t QuadPart;
} LARGE_INTEGER;

#else
    #include "winstubs.h" //Put everything Windows specific in here
#endif

#include "airspy.h"
#include "./libusb/include/libusb.h"

#include "anet.h"

typedef struct airspy_device {
    libusb_context          *usb_context;
    libusb_device_handle    *usb_device;
    struct libusb_transfer **transfers;
    volatile int             streaming;
} airspy_device_t;

static const uint16_t airspy_usb_vid = 0x1d50;
static const uint16_t airspy_usb_pid = 0x60a1;

#define UNUSED(V) ((void) V)
#define TRUE  1
#define FALSE 0

// ============================= #defines ===============================
//
// If you have a valid coaa.h, these values will come from it. If not,
// then you can enter your own values in the #else section here
//
#ifdef USER_LATITUDE
    #define MODES_USER_LATITUDE_DFLT   (USER_LATITUDE)
    #define MODES_USER_LONGITUDE_DFLT  (USER_LONGITUDE)
#else
    #define MODES_USER_LATITUDE_DFLT   (0.0)
    #define MODES_USER_LONGITUDE_DFLT  (0.0)
#endif

#define MODES_DEFAULT_RATE         20000000
#define MODES_DEFAULT_FREQ         1090000000
#define MODES_DEFAULT_WIDTH        1000
#define MODES_DEFAULT_HEIGHT       700
#define MODES_ASYNC_BUF_NUMBER     (16)
#define MODES_ASYNC_BUF_SIZE       (262144)                          // 262144;
#define MODES_ASYNC_BUF_SAMPLES    (MODES_ASYNC_BUF_SIZE / 2)        // 131072; // Number of samples in a MODES_ASYNC_USB_SIZE block
#define MODES_AUTO_GAIN            -100                              // Use automatic gain
#define MODES_MAX_GAIN             21                                // Use max available gain
#define MODES_MSG_ENCODER_ERRS     3                                 // Maximum number of encoding errors

// When changing, change also fixBitErrors() and modesInitErrorTable() !!
#define MODES_MAX_BITERRORS        2                          // Global max for fixable bit erros

#define MODEAC_MSG_SAMPLES       (25 * 2)                     // include up to the SPI bit
#define MODEAC_MSG_BYTES          2
#define MODEAC_MSG_SQUELCH_LEVEL  0x03FF                      // Average signal strength limit
#define MODEAC_MSG_FLAG          (1<<0)
#define MODEAC_MSG_MODES_HIT     (1<<1)
#define MODEAC_MSG_MODEA_HIT     (1<<2)
#define MODEAC_MSG_MODEC_HIT     (1<<3)
#define MODEAC_MSG_MODEA_ONLY    (1<<4)
#define MODEAC_MSG_MODEC_OLD     (1<<5)

#define MODES_DECODE_BUF_NUMBER  (65536)
#define MODES_DECODE_FIFO_NUMBER (16384)

#define MODES_PREAMBLE_BITS     (8)
#define MODES_PREAMBLE_SAMPLES  (MODES_PREAMBLE_BITS     * 20)
#define MODES_PREAMBLE_SIZE     (MODES_PREAMBLE_SAMPLES  * sizeof(uint16_t))
#define MODES_LONG_MSG_BYTES     14
#define MODES_SHORT_MSG_BYTES    7
#define MODES_LONG_MSG_BITS     (MODES_LONG_MSG_BYTES    * 8)
#define MODES_SHORT_MSG_BITS    (MODES_SHORT_MSG_BYTES   * 8)
#define MODES_LONG_MSG_SAMPLES  (MODES_LONG_MSG_BITS     * 20)
#define MODES_SHORT_MSG_SAMPLES (MODES_SHORT_MSG_BITS    * 20)
//#define MODES_LONG_MSG_SIZE     (MODES_LONG_MSG_SAMPLES  * sizeof(uint16_t))
//#define MODES_SHORT_MSG_SIZE    (MODES_SHORT_MSG_SAMPLES * sizeof(uint16_t))

#define MODES_LONG_FRAME_BITS     (MODES_PREAMBLE_BITS    + MODES_LONG_MSG_BITS)
#define MODES_SHORT_FRAME_SAMPLES (MODES_PREAMBLE_SAMPLES + MODES_SHORT_MSG_SAMPLES)
#define MODES_LONG_FRAME_SAMPLES  (MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES)
//#define MODES_SHORT_FRAME_SIZE    (MODES_SHORT_FRAME_SAMPLES * sizeof(uint16_t))
//#define MODES_LONG_FRAME_SIZE     (MODES_LONG_FRAME_SAMPLES * sizeof(uint16_t))

#define MODES_RAWOUT_BUF_SIZE   (1500)
//#define MODES_RAWOUT_BUF_SIZE   (15000)
#define MODES_RAWOUT_BUF_FLUSH  (MODES_RAWOUT_BUF_SIZE - 200)
#define MODES_RAWOUT_BUF_RATE   (1000)            // 1000 * 64mS = 1 Min approx

#define MODES_ICAO_CACHE_LEN 1024 // Power of two required
#define MODES_ICAO_CACHE_TTL (60 * MODES_DEFAULT_RATE) // Time to live of cached addresses
#define MODES_UNIT_FEET   0
#define MODES_UNIT_METERS 1

#define MODEA_CACHE_LEN     (4096)
#define MODEA_CACHE_MIN_PRF (20000)                    // Minimum ModeA PRF 1mS
#define MODEA_CACHE_MAX_PRF (2000000)                  // Maximum ModeA PRF 100mS
#define MODEA_CACHE_CNT_PRF (3)
#define MODEA_CACHE_REV_PRF (2)
#define MODEA_CACHE_TTL_PRF (15 * MODES_DEFAULT_RATE)  // Time to live of cached ModeA

#define MODES_USER_LATLON_VALID (1<<0)

#define MODES_ACFLAGS_LATLON_VALID   (1<<0)  // Aircraft Lat/Lon is decoded
#define MODES_ACFLAGS_ALTITUDE_VALID (1<<1)  // Aircraft altitude is known
#define MODES_ACFLAGS_HEADING_VALID  (1<<2)  // Aircraft heading is known
#define MODES_ACFLAGS_SPEED_VALID    (1<<3)  // Aircraft speed is known
#define MODES_ACFLAGS_VERTRATE_VALID (1<<4)  // Aircraft vertical rate is known
#define MODES_ACFLAGS_SQUAWK_VALID   (1<<5)  // Aircraft Mode A Squawk is known
#define MODES_ACFLAGS_CALLSIGN_VALID (1<<6)  // Aircraft Callsign Identity
#define MODES_ACFLAGS_EWSPEED_VALID  (1<<7)  // Aircraft East West Speed is known
#define MODES_ACFLAGS_NSSPEED_VALID  (1<<8)  // Aircraft North South Speed is known
#define MODES_ACFLAGS_AOG            (1<<9)  // Aircraft is On the Ground
#define MODES_ACFLAGS_LLEVEN_VALID   (1<<10) // Aircraft Even Lot/Lon is known
#define MODES_ACFLAGS_LLODD_VALID    (1<<11) // Aircraft Odd Lot/Lon is known
#define MODES_ACFLAGS_AOG_VALID      (1<<12) // MODES_ACFLAGS_AOG is valid
#define MODES_ACFLAGS_FS_VALID       (1<<13) // Aircraft Flight Status is known
#define MODES_ACFLAGS_NSEWSPD_VALID  (1<<14) // Aircraft EW and NS Speed is known
#define MODES_ACFLAGS_LATLON_REL_OK  (1<<15) // Indicates it's OK to do a relative CPR

#define MODES_ACFLAGS_LLEITHER_VALID (MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID)
#define MODES_ACFLAGS_LLBOTH_VALID   (MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID)
#define MODES_ACFLAGS_AOG_GROUND     (MODES_ACFLAGS_AOG_VALID    | MODES_ACFLAGS_AOG)
#define MODES_INTERACTIVE_REFRESH_TIME 250      // Milliseconds
#define MODES_INTERACTIVE_ROWS          80      // Rows on screen
#define MODES_INTERACTIVE_DELETE_TTL   300      // Delete from the list after 300 seconds
#define MODES_INTERACTIVE_DISPLAY_TTL   60      // Delete from display after 60 seconds

#define MODES_NET_HEARTBEAT_RATE       9155     // Each block is approx 6.5mS - default is > 1 min

#define MODES_NET_SERVICES_NUM          4
//#define MODES_NET_INPUT_RAW_PORT    30001
#define MODES_NET_OUTPUT_RAW_PORT   30002
#define MODES_NET_OUTPUT_SBS_PORT   30003
//#define MODES_NET_INPUT_BEAST_PORT  30004
#define MODES_NET_OUTPUT_BEAST_PORT 30005
#define MODES_NET_OUTPUT_AVR_PORT   30006
#define MODES_CLIENT_BUF_SIZE  1024
#define MODES_NET_SNDBUF_SIZE (1024*64)
#define MODES_NET_SNDBUF_MAX  (7)

#define MODES_NOTUSED(V) ((void) V)

#define kDFValid (0x00370831) // One bit per 32 possible DF's. Set bits 0,4,5,11,16,17,18,20,21

//======================== structure declarations =========================

typedef struct stICAOCache {
    uint64_t     lTime;
    uint32_t     uAddr;
    uint32_t     uPadding;
} tICAOCache, *pICAOCache;

typedef struct stModeACache {
    uint64_t     lTime;
    uint32_t     uRevCount;
    uint32_t     uPrfCount;
} tModeACache, *pModeACache;

// Structure used to describe a networking client
typedef struct stClient {
    struct stClient*  next;              // Pointer to next client
    int    fd;                           // File descriptor
    int    service;                      // TCP port the client is connected to
    int    buflen;                       // Amount of data on buffer
    char   buf[MODES_CLIENT_BUF_SIZE+1]; // Read buffer
} tClient;

// Structure used to describe an aircraft in iteractive mode
typedef struct stAircraft {
    uint32_t      addr;           // ICAO address
    char          flight[16];     // Flight number
    unsigned char signalLevel[8]; // Last 8 Signal Amplitudes
    int           altitude;       // Altitude
    int           speed;          // Velocity
    int           track;          // Angle of flight
    int           vert_rate;      // Vertical rate.
    time_t        seen;           // Time at which the last packet was received
    time_t        seenLatLon;     // Time at which the last lat long was calculated
    uint64_t      timestamp;      // Timestamp at which the last packet was received
    uint64_t      timestampLatLon;// Timestamp at which the last lat long was calculated
    long          messages;       // Number of Mode S messages received
    uint32_t      uModeA;         // Squawk
    int           modeC;          // Altitude
    long          modeAcount;     // Mode A Squawk hit Count
    long          modeCcount;     // Mode C Altitude hit Count
    int           modeACflags;    // Flags for mode A/C recognition

    // Encoded latitude and longitude as extracted by odd and even CPR encoded messages
    int           odd_cprlat;
    int           odd_cprlon;
    int           even_cprlat;
    int           even_cprlon;
    uint64_t      odd_cprtime;
    uint64_t      even_cprtime;
    double        lat, lon;       // Coordinated obtained from CPR encoded data
    int           bFlags;         // Flags related to valid fields in this structure
    struct stAircraft *next;      // Next aircraft in our linked list
} tAircraft;

// The struct we use to store information about a decoded message.
// This structure is used in some assembly language functions under some
// architectures, so DO NOT meddle with the order or size of the structure
// elements unless you want to re-write the assembler too.
// The struct we use to store basic additional information about a decoded message.
typedef struct stMessageRaw {
    // Generic fields
    uint32_t      signalLevel32;                  // 00-03 Signal Amplitude
    uint32_t      uMsgBits;                       // 04-07 Number of bits in message 
    unsigned char ucMsg[16];                      // 08-23 Binary message.
    uint64_t      lTime;                          // 24-31 Timestamp of the message
    uint32_t      uCrc;                           // 32-35 Message CRC
    uint32_t      uAddr;                          // 36-39 ICAO Address from bytes 1 2 and 3
    uint32_t      uMsgType;                       // 40-43 Downlink format #
    uint32_t      nCount;                         // 44-47
    unsigned char spare[12];                      // 48-59
    struct stMessageRaw * pNext;                  // 60-63
} tMessageRaw;

// The struct we use to store fully decoded information about a decoded message.
typedef struct stModesMessage {
    // Raw Message 
    tMessageRaw   rm;

    // Decoded fields
    unsigned char signalLevel;  // Signal Amplitude

    // DF 11
    int    ca;                  // Responder capabilities
    int    iid;

    // DF 17, DF 18
    int    metype;              // Extended squitter message type.
    int    mesub;               // Extended squitter message subtype.
    int    heading;             // Reported by aircraft, or computed from from EW and NS velocity
    int    raw_latitude;        // Non decoded latitude.
    int    raw_longitude;       // Non decoded longitude.
    double fLat;                // Coordinates obtained from CPR encoded data if/when decoded
    double fLon;                // Coordinates obtained from CPR encoded data if/when decoded
    char   flight[16];          // 8 chars flight number.
    int    ew_velocity;         // E/W velocity.
    int    ns_velocity;         // N/S velocity.
    int    vert_rate;           // Vertical rate.
    int    velocity;            // Reported by aircraft, or computed from from EW and NS velocity

    // DF4, DF5, DF20, DF21
    int  fs;                    // Flight status for DF4,5,20,21
    uint32_t uModeA;            // 13 bits identity (Squawk).

    // Fields used by multiple message types.
    int  altitude;
    int  unit; 
    int  bFlags;                // Flags related to fields in this structure
} tModesMessage;

typedef struct stFifo16 {
    uint16_t* pFifo; // Pointer to the data in this FIFO entry
    uint32_t  uLost; // Count of the buffers lost after this buffer
    uint64_t  lTime; // Time of the start of the buffer
} tFifo16;

// Program global state
struct {                               // Internal state
    pthread_t             reader_thread;

    pthread_t             IF_thread;
    pthread_mutex_t       IF_mutex;        // Mutex to synchronize buffer access
    pthread_cond_t        IF_cond;         // Conditional variable associated
    tFifo16               IF_Fifo[MODES_ASYNC_BUF_NUMBER]; // Raw IQ sample buffers from libusb
    volatile uint32_t     uIF_In;          // RF Fifo input pointer
    volatile uint32_t     uIF_Out;         // RF Fifo output pointer
    uint16_t*             uIF_iDelay;
    uint16_t              iIF_Out;

    pthread_t             AF_thread;
    pthread_mutex_t       AF_mutex;        // Mutex to synchronize buffer access
    pthread_cond_t        AF_cond;         // Conditional variable associated
    tFifo16               AF_Fifo[MODES_ASYNC_BUF_NUMBER]; // Filtered samples ready for ModeA/C/S processing
    volatile uint32_t     uAF_In;          // Fifo input pointer
    volatile uint32_t     uAF_Out;         // Fifo output pointer
    uint16_t*             uAF_iDelay;
    uint16_t*             uAF_oDelay;
    uint16_t              iAF_Out;

    pthread_t             ModeA_thread;
    pthread_mutex_t       ModeA_Go_mutex;  // Mutex to synchronize buffer access
    pthread_cond_t        ModeA_Go_cond;   // Conditional variable associated
    pthread_mutex_t       ModeA_Done_mutex;// Mutex to synchronize buffer access
    pthread_cond_t        ModeA_Done_cond; // Conditional variable associated
    tMessageRaw*          pModeA_List;
    tMessageRaw*          pModeA_Buffer[MODES_DECODE_BUF_NUMBER];
    volatile uint32_t     uModeA_Buffer;
    uint32_t              uModeATimeout;

    pthread_t             Decode_thread;
    pthread_mutex_t       Decode_mutex;      // Mutex to synchronize buffer access
    pthread_cond_t        Decode_cond;       // Conditional variable associated
    tMessageRaw*          Decode_ModeS_Fifo[MODES_DECODE_FIFO_NUMBER]; // Decoded Frames ready for processing
    volatile uint32_t     uDecode_ModeS_In;  // Fifo input pointer
    volatile uint32_t     uDecode_ModeS_Out; // Fifo output pointer
    tMessageRaw*          Decode_ModeA_Fifo[MODES_DECODE_FIFO_NUMBER]; // Decoded Frames ready for processing
    volatile uint32_t     uDecode_ModeA_In;  // Fifo input pointer
    volatile uint32_t     uDecode_ModeA_Out; // Fifo output pointer
    tMessageRaw*          pModeS_List;
    tMessageRaw*          pModeS_Buffer[MODES_DECODE_BUF_NUMBER];
    volatile uint32_t     uModeS_Buffer;
    uint32_t              uModeSTimeout;

    pthread_t             Replay_thread;
    uint16_t             *pFileData;       // Raw IQ samples buffer (from a File)
    uint64_t              timestampBlk;    // Timestamp of the start of the current block
    struct timeb          stSystemTimeBlk; // System time when RTL passed us currently processing this block
    int                   fd;              // --ifile option file descriptor
    tICAOCache           *icao_cache;      // Recently seen ICAO addresses cache
    tModeACache          *ModeA_cache;     // Recently seen ModeA cache
    volatile int          exit;            // Exit from the main loop when true

    // Device Control
    int                   dev_index;
    airspy_device_t*      dev;
    int                   gain;            // Linearity gain setting
    int                   vgagain;         // Vga gain override (-1 = unused)
    int                   lnagain;         // Lna gain override (-1 = unused)
    int                   mixergain;       // mixer gain override (-1 = unused)
    int                   bias_t;          // Bias_t ( 1=enabled, 0 = disabled)
    int                   freq;            // Tuner frequency (= 1090Mhz)

    // Networking
    char            aneterr[ANET_ERR_LEN];
    tClient        *clients;         // Our clients

    unsigned int    sbsConnections;  // Number of currently open SBS connections
    int             sbsSocket;       // SBS output listening socket

    unsigned int    avrConnections;  // Number of currently open AVR ASCII connections
    int             avrSocket;       // AVR output listening socket
    char           *avrOut;          // Buffer for building AVR output data
    int             avrOutUsed;      // How much of the buffer is currently used

    unsigned int    rawConnections;  // Number of currently open RAW ASCII connections
    int             rawSocket;       // RAW Ascii output listening socket
    char           *rawOut;          // Buffer for building RAW ascii output data
    int             rawOutUsed;      // How much of the buffer is currently used

    unsigned int    beastConnections;// Number of currently open beast binary connections
    int             beastSocket;     // Beast Binary output listening socket
    char           *beastOut;        // Buffer for building beast binary output data
    int             beastOutUsed;    // How much of the buffer is currently used

#ifdef _WIN32
    WSADATA         wsaData;         // Windows socket initialisation
#endif

    // Configuration
    char     *filename;              // Input form file, --ifile option
    int       mode_ac;               // Enable decoding of SSR Modes A & C
    int       mode_ac_prf_min;       // Minimum mode A/C PRF rate (in ms)
    int       mode_ac_prf_max;       // Maximum mode A/C PRF rate (in ms)
    uint32_t  mode_ac_prf_count;     // Min mode A/C PRF count
    uint32_t  mode_ac_prf_revs;      // Min mode A/C REV count
    int       mode_ac_prf_ttl;       // Mode A/C Time to Live

    int   net_heartbeat_count;       // TCP heartbeat counter
    int   net_heartbeat_rate;        // TCP heartbeat rate
    int   net_output_raw_size;       // Minimum Size of the output raw data
    int   net_output_raw_rate;       // Rate (in 64mS increments) of output raw data
    int   net_output_raw_rate_count; // Rate (in 64mS increments) of output raw data
    int   net_output_sbs_port;       // SBS output TCP port
    int   net_output_avr_port;       // AVR output TCP port
    int   net_output_raw_port;       // Raw output TCP port
    int   net_output_beast_port;     // Beast output TCP port
    char *net_bind_address;          // Bind address
    int   net_sndbuf_size;           // TCP output buffer size (64Kb * 2^n)
    int   quiet;                     // Suppress stdout
    int   interactive;               // Interactive mode
    int   interactive_rows;          // Interactive mode: max number of rows
    int   interactive_display_ttl;   // Interactive mode: TTL display
    int   interactive_delete_ttl;    // Interactive mode: TTL before deletion

    // User details
    double fUserLat;                // Users receiver/antenna lat/lon needed for initial surface location
    double fUserLon;                // Users receiver/antenna lat/lon needed for initial surface location
    int    bUserFlags;              // Flags relating to the user details

    // Interactive mode
    tAircraft       *pAircraft;
    uint64_t         interactive_last_update; // Last screen update in milliseconds
    time_t           last_cleanup_time;       // Last cleanup time in seconds
} Modes;

// ======================== function declarations =========================

#ifdef __cplusplus
extern "C" {
#endif

//
// Functions exported from airspy.c
//
void airspy_InitDevice(void);
void airspy_start_replay(void);

//
// Functions exported from Demodulate.c
//
void* IF_threadproc(void* arg);

//
// Functions exported from detectModeS.c
//
void* ModeS_threadproc(void* arg);

//
// Functions exported from detectModeA.c
//
void* ModeA_threadproc(void* arg);

//
// Functions exported from decode.c
//
void* Decode_threadproc(void* arg);

//
// Functions exported from mode_ac.c
//
uint32_t validateModeA (tMessageRaw   *rm);
void updateModeA       (tModesMessage *mm);
void updateModeC       (tModesMessage *mm);
void decodeModeA       (tModesMessage *mm);
int  ModeAToModeC      (uint32_t      uModeA);

//
// Functions exported from mode_s.c
//
void decodeModeS       (tModesMessage *mm);
int  decodeCPR         (tAircraft *a, int fflag, int surface);
int  decodeCPRrelative (tAircraft *a, int fflag, int surface);

//
// Functions exported from interactive.c
//
tAircraft* interactiveReceiveData(tModesMessage *mm);
void  interactiveShowData(void);
void  interactiveListData(tModesMessage* mm);
void  interactiveRemoveStaleAircrafts(void);

//
// Functions exported from net_io.c
//
void netInit            (void);
void netReadFromClients (void);
void netSendAllClients  (int service, void *msg, int len);
void netQueueOutput     (tModesMessage *mm);
void netFlushClients    (uint64_t lTime);

#ifdef __cplusplus
}
#endif

#endif // __AIRSPY1090_H
