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
// ============================= Networking =============================
//
// Note: here we disregard any kind of good coding practice in favor of
// extreme simplicity, that is:
//
// 1) We only rely on the kernel buffers for our I/O without any kind of
//    user space buffering.
// 2) We don't register any kind of event handler, from time to time a
//    function gets called and we accept new connections. All the rest is
//    handled via non-blocking I/O and manually polling clients to see if
//    they have something new to share with us when reading is needed.
//
//=========================================================================
//
// Networking "stack" initialization
//
typedef struct stService {
	char *descr;
	int *socket;
	int port;
	int enabled;
} tService;

tService services[MODES_NET_SERVICES_NUM];

void netInit(void) {
    int j;

	tService svc[MODES_NET_SERVICES_NUM] = {
		{"AVR TCP output",         &Modes.avrSocket,   Modes.net_output_avr_port,   1},
		{"RAW TCP output",         &Modes.rawSocket,   Modes.net_output_raw_port,   1},
		{"Beast TCP output",       &Modes.beastSocket, Modes.net_output_beast_port, 1},
		{"Basestation TCP output", &Modes.sbsSocket,   Modes.net_output_sbs_port,   1}
	};

	memcpy(&services, &svc, sizeof(svc));//services = svc;

    Modes.clients = NULL;

#ifdef _WIN32
    if ( (!Modes.wsaData.wVersion) 
      && (!Modes.wsaData.wHighVersion) ) {
      // Try to start the windows socket support
      if (WSAStartup(MAKEWORD(2,1),&Modes.wsaData) != 0) 
        {
        fprintf(stderr, "WSAStartup returned Error\n");
        }
      }
#endif

    for (j = 0; j < MODES_NET_SERVICES_NUM; j++) {
		services[j].enabled = (services[j].port != 0);
		if (services[j].enabled) {
			int s = anetTcpServer(Modes.aneterr, services[j].port, Modes.net_bind_address);
			if (s == -1) {
				fprintf(stderr, "Error opening the listening port %d (%s): %s\n",
					services[j].port, services[j].descr, Modes.aneterr);
				exit(1);
			}
			anetNonBlock(Modes.aneterr, s);
			*services[j].socket = s;
		}
    }

#ifndef _WIN32
    signal(SIGPIPE, SIG_IGN);
#endif
}
//
//=========================================================================
//
// This function gets called from time to time when the decoding thread is
// awakened by new data arriving. This usually happens a few times every second
//
tClient * netAcceptClients(void) {
    int fd, port;
    unsigned int j;
    tClient *c;

    for (j = 0; j < MODES_NET_SERVICES_NUM; j++) {
		if (services[j].enabled) {
			fd = anetTcpAccept(Modes.aneterr, *services[j].socket, NULL, &port);
			if (fd == -1) continue;

			anetNonBlock(Modes.aneterr, fd);
			c = (tClient *) malloc(sizeof(*c));
            if (!c) continue;

            c->service    = *services[j].socket;
			c->next       = Modes.clients;
			c->fd         = fd;
			c->buflen     = 0;
			Modes.clients = c;
			anetSetSendBuffer(Modes.aneterr,fd, (MODES_NET_SNDBUF_SIZE << Modes.net_sndbuf_size));

			if (*services[j].socket == Modes.sbsSocket)   Modes.sbsConnections++;
			if (*services[j].socket == Modes.avrSocket)   Modes.avrConnections++;
			if (*services[j].socket == Modes.beastSocket) Modes.beastConnections++;
			if (*services[j].socket == Modes.rawSocket)   Modes.rawConnections++;

			j--; // Try again with the same listening port
		}
    }
    return Modes.clients;
}
//
//=========================================================================
//
// On error free the client, collect the structure, adjust maxfd if needed.
//
void netFreeClient(tClient *c) {

    // Unhook this client from the linked list of clients
    tClient *p = Modes.clients;
    if (p) {
        if (p == c) {
            Modes.clients = c->next;
        } else {
            while ((p) && (p->next != c)) {
                p = p->next;
            }
            if (p) {
                p->next = c->next;
            }
        }
    }

    free(c);
}
//
//=========================================================================
//
// Close the client connection and mark it as closed
//
void netCloseClient(tClient *c) {
	close(c->fd);
    if        (c->service == Modes.sbsSocket) {
        if (Modes.sbsConnections) Modes.sbsConnections--;
    } else if (c->service == Modes.avrSocket) {
        if (Modes.avrConnections) Modes.avrConnections--;
    } else if (c->service == Modes.beastSocket) {
        if (Modes.beastConnections) Modes.beastConnections--;
    } else if (c->service == Modes.rawSocket) {
        if (Modes.rawConnections) Modes.rawConnections--;
    }

    c->fd = -1;
}
//
//=========================================================================
//
// Send the specified message to all clients listening for a given service
//
void netSendAllClients(int service, void *msg, int len) {
    tClient *c = Modes.clients;

    while (c) {
        // Read next before servicing client incase the service routine deletes the client! 
        tClient *next = c->next;

        if (c->fd != -1) {
            if (c->service == service) {
#ifndef _WIN32
                int nwritten = write(c->fd, msg, len);
#else
                int nwritten = send(c->fd, msg, len, 0 );
#endif
                if (nwritten != len) {
                    netCloseClient(c);
                }
            }
        } else {
            netFreeClient(c);
        }
        c = next;
    }
}
//
//=========================================================================
//
// Write raw output in Beast Binary format with Timestamp to TCP clients
//
void netSendBeastOutput(tModesMessage *mm) {
    char *p = &Modes.beastOut[Modes.beastOutUsed];
    int  msgLen = mm->rm.uMsgBits / 8;
    uint64_t lTimeStamp = mm->rm.lTime;
    char ch;
    int  j;
    int  iOutLen = msgLen + 9; // Escape, msgtype, timestamp, sigLevel, msg

    // Output the escape character
    *p++ = 0x1a;

    // Output the message type, and correct the timestamp for message type
    if      (msgLen == MODES_SHORT_MSG_BYTES)
        {*p++ = '2';}
    else if (msgLen == MODES_LONG_MSG_BYTES)
        {*p++ = '3';}
    else if (msgLen == MODEAC_MSG_BYTES)
        {*p++ = '1'; lTimeStamp += (377 * 20);}
    else
        {return;}

    // Output the Timestamp
    *p++ = (ch = (unsigned char)((lTimeStamp >> 40) & 0xFF));
    if (0x1A == ch) { *p++ = ch; iOutLen++; }
    *p++ = (ch = (unsigned char)((lTimeStamp >> 32) & 0xFF));
    if (0x1A == ch) { *p++ = ch; iOutLen++; }
    *p++ = (ch = (unsigned char)((lTimeStamp >> 24) & 0xFF));
    if (0x1A == ch) { *p++ = ch; iOutLen++; }
    *p++ = (ch = (unsigned char)((lTimeStamp >> 16) & 0xFF));
    if (0x1A == ch) { *p++ = ch; iOutLen++; }
    *p++ = (ch = (unsigned char)((lTimeStamp >>  8) & 0xFF));
    if (0x1A == ch) { *p++ = ch; iOutLen++; }
    *p++ = (ch = (unsigned char)((lTimeStamp >>  0) & 0xFF));
    if (0x1A == ch) { *p++ = ch; iOutLen++; }

    // Output the signal level
    *p++ = (ch = mm->signalLevel);
    if (0x1A == ch) {*p++ = ch; iOutLen++;}

    // Output the message bytes
    for (j = 0; j < msgLen; j++) {
        *p++ = (ch = mm->rm.ucMsg[j]);
        if (0x1A == ch) {*p++ = ch; iOutLen++;} 
    }

    // Increase the Beast buffer count to include this message 
    Modes.beastOutUsed +=  iOutLen;

    // If the buffer now contains enough data to be worth sending, send it
    if (Modes.beastOutUsed >= Modes.net_output_raw_size) {
        netSendAllClients(Modes.beastSocket, Modes.beastOut, Modes.beastOutUsed);
        Modes.beastOutUsed = 0;
        Modes.net_output_raw_rate_count = 0;
    }
}
//
//=========================================================================
//
// Write RAW ASCII output (with timestamp) to TCP clients
//
void netSendRawOutput(tModesMessage *mm) {
    char *p = &Modes.rawOut[Modes.rawOutUsed];
    int  msgLen = mm->rm.uMsgBits / 8;
    int j;
    unsigned char * pTimeStamp;

    *p++ = '@';
    pTimeStamp = (unsigned char *) &mm->rm.lTime;
    for (j = 5; j >= 0; j--) {
        sprintf(p, "%02X", pTimeStamp[j]);
        p += 2;
    }
    Modes.rawOutUsed += 12; // additional 12 characters for timestamp

    for (j = 0; j < msgLen; j++) {
        sprintf(p, "%02X", mm->rm.ucMsg[j]);
        p += 2;
    }

    *p++ = ';';
    *p++ = '\n';

    Modes.rawOutUsed += ((msgLen*2) + 3);
    if (Modes.rawOutUsed >= Modes.net_output_raw_size) {
        netSendAllClients(Modes.rawSocket, Modes.rawOut, Modes.rawOutUsed);
        Modes.rawOutUsed = 0;
        Modes.net_output_raw_rate_count = 0;
    }
}
//
//=========================================================================
//
// Write AVR ASCII (no timestamp) output to TCP clients
//
void netSendAVROutput(tModesMessage *mm) {
    char *p = &Modes.avrOut[Modes.avrOutUsed];
    int  msgLen = mm->rm.uMsgBits / 8;
    int j;

    *p++ = '*';
    for (j = 0; j < msgLen; j++) {
        sprintf(p, "%02X", mm->rm.ucMsg[j]);
        p += 2;
    }

    *p++ = ';';
    *p++ = '\n';

    Modes.avrOutUsed += ((msgLen*2) + 3);
    if (Modes.avrOutUsed >= Modes.net_output_raw_size) {
        netSendAllClients(Modes.avrSocket, Modes.avrOut, Modes.avrOutUsed);
        Modes.avrOutUsed = 0;
        Modes.net_output_raw_rate_count = 0;
    }
}
//
//=========================================================================
//
// Write SBS output to TCP clients
// The message structure mm->bFlags tells us what has been updated by this message
//
void netSendSBSOutput(tModesMessage *mm) {
    char msg[256], *p = msg;
    uint32_t     offset;
    struct timeb epocTime_receive, epocTime_now;
    struct tm    stTime_receive, stTime_now;
    int          msgType;

    //
    // SBS BS style output checked against the following reference
    // http://www.homepages.mcb.net/bones/SBS/Article/Barebones42_Socket_Data.htm - seems comprehensive
    //

    // Decide on the basic SBS Message Type
    if        ((mm->rm.uMsgType ==  4) || (mm->rm.uMsgType == 20)) {
        msgType = 5;
    } else if ((mm->rm.uMsgType ==  5) || (mm->rm.uMsgType == 21)) {
        msgType = 6;
    } else if ((mm->rm.uMsgType ==  0) || (mm->rm.uMsgType == 16)) {
        msgType = 7;
    } else if  (mm->rm.uMsgType == 11) {
        msgType = 8;
    } else if ((mm->rm.uMsgType != 17) && (mm->rm.uMsgType != 18)) {
        return;
    } else if ((mm->metype >= 1) && (mm->metype <=  4)) {
        msgType = 1;
    } else if ((mm->metype >= 5) && (mm->metype <=  8)) {
        if (mm->bFlags & MODES_ACFLAGS_LATLON_VALID)
            {msgType = 2;}
        else
            {msgType = 7;}
    } else if ((mm->metype >= 9) && (mm->metype <= 18)) {
        if (mm->bFlags & MODES_ACFLAGS_LATLON_VALID)
            {msgType = 3;}
        else
            {msgType = 7;}
    } else if (mm->metype !=  19) {
        return;
    } else if ((mm->mesub == 1) || (mm->mesub == 2)) {
        msgType = 4;
    } else {
        return;
    }

    // Fields 1 to 6 : SBS message type and ICAO address of the aircraft and some other stuff
    p += sprintf(p, "MSG,%d,111,11111,%06X,111111,", msgType, mm->rm.uAddr); 

    // Find current system time
    ftime(&epocTime_now);                                         // get the current system time & date
    stTime_now = *localtime(&epocTime_now.time);

    // Find message reception time
    if (mm->rm.lTime) {                                           // Make sure the records' timestamp is valid before using it
        epocTime_receive = Modes.stSystemTimeBlk;                 // This is the time of the start of the Block we're processing
        offset   = (int) (mm->rm.lTime - Modes.timestampBlk);     // This is the time (in 12Mhz ticks) into the Block
        offset   = offset / 12000;                                // convert to milliseconds
        epocTime_receive.millitm += (uint16_t) offset;            // add on the offset time to the Block start time
        if (epocTime_receive.millitm > 999) {                     // if we've caused an overflow into the next second...
            epocTime_receive.millitm -= 1000;
            epocTime_receive.time ++;                             //    ..correct the overflow
        }
        stTime_receive = *localtime(&epocTime_receive.time);
    } else {
        epocTime_receive = epocTime_now;                          // We don't have a usable reception time; use the current system time
        stTime_receive = stTime_now;
    }

    // Fields 7 & 8 are the message reception time and date
    p += sprintf(p, "%04d/%02d/%02d,", (stTime_receive.tm_year+1900),(stTime_receive.tm_mon+1), stTime_receive.tm_mday);
    p += sprintf(p, "%02d:%02d:%02d.%03d,", stTime_receive.tm_hour, stTime_receive.tm_min, stTime_receive.tm_sec, epocTime_receive.millitm);

    // Fields 9 & 10 are the current time and date
    p += sprintf(p, "%04d/%02d/%02d,", (stTime_now.tm_year+1900),(stTime_now.tm_mon+1), stTime_now.tm_mday);
    p += sprintf(p, "%02d:%02d:%02d.%03d", stTime_now.tm_hour, stTime_now.tm_min, stTime_now.tm_sec, epocTime_now.millitm);

    // Field 11 is the callsign (if we have it)
    if (mm->bFlags & MODES_ACFLAGS_CALLSIGN_VALID) {p += sprintf(p, ",%s", mm->flight);}
    else                                           {p += sprintf(p, ",");}

    // Field 12 is the altitude (if we have it) - force to zero if we're on the ground
    if ((mm->bFlags & MODES_ACFLAGS_AOG_GROUND) == MODES_ACFLAGS_AOG_GROUND) {
        p += sprintf(p, ",0");
    } else if (mm->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
        p += sprintf(p, ",%d", mm->altitude);
    } else {
        p += sprintf(p, ",");
    }

    // Field 13 is the ground Speed (if we have it)
    if (mm->bFlags & MODES_ACFLAGS_SPEED_VALID) {
        p += sprintf(p, ",%d", mm->velocity);
    } else {
        p += sprintf(p, ","); 
    }

    // Field 14 is the ground Heading (if we have it)       
    if (mm->bFlags & MODES_ACFLAGS_HEADING_VALID) {
        p += sprintf(p, ",%d", mm->heading);
    } else {
        p += sprintf(p, ",");
    }

    // Fields 15 and 16 are the Lat/Lon (if we have it)
    if (mm->bFlags & MODES_ACFLAGS_LATLON_VALID) {p += sprintf(p, ",%1.5f,%1.5f", mm->fLat, mm->fLon);}
    else                                         {p += sprintf(p, ",,");}

    // Field 17 is the VerticalRate (if we have it)
    if (mm->bFlags & MODES_ACFLAGS_VERTRATE_VALID) {p += sprintf(p, ",%d", mm->vert_rate);}
    else                                           {p += sprintf(p, ",");}

    // Field 18 is  the Squawk (if we have it)
    if (mm->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {p += sprintf(p, ",%x", mm->uModeA);}
    else                                         {p += sprintf(p, ",");}

    // Field 19 is the Squawk Changing Alert flag (if we have it)
    if (mm->bFlags & MODES_ACFLAGS_FS_VALID) {
        if ((mm->fs >= 2) && (mm->fs <= 4)) {
            p += sprintf(p, ",-1");
        } else {
            p += sprintf(p, ",0");
        }
    } else {
        p += sprintf(p, ",");
    }

    // Field 20 is the Squawk Emergency flag (if we have it)
    if (mm->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {
        if ((mm->uModeA == 0x7500) || (mm->uModeA == 0x7600) || (mm->uModeA == 0x7700)) {
            p += sprintf(p, ",-1");
        } else {
            p += sprintf(p, ",0");
        }
    } else {
        p += sprintf(p, ",");
    }

    // Field 21 is the Squawk Ident flag (if we have it)
    if (mm->bFlags & MODES_ACFLAGS_FS_VALID) {
        if ((mm->fs >= 4) && (mm->fs <= 5)) {
            p += sprintf(p, ",-1");
        } else {
            p += sprintf(p, ",0");
        }
    } else {
        p += sprintf(p, ",");
    }

    // Field 22 is the OnTheGround flag (if we have it)
    if (mm->bFlags & MODES_ACFLAGS_AOG_VALID) {
        if (mm->bFlags & MODES_ACFLAGS_AOG) {
            p += sprintf(p, ",-1");
        } else {
            p += sprintf(p, ",0");
        }
    } else {
        p += sprintf(p, ",");
    }

    p += sprintf(p, "\r\n");
    netSendAllClients(Modes.sbsSocket, msg, (int) (p-msg));
}
//
//=========================================================================
//
void netQueueOutput(tModesMessage *mm) {
    if (Modes.sbsConnections)   {netSendSBSOutput(mm);}
    if (Modes.beastConnections) {netSendBeastOutput(mm);}
    if (Modes.rawConnections)   {netSendRawOutput(mm);}
    if (Modes.avrConnections)   {netSendAVROutput(mm);}
}
//
//=========================================================================
//
// Read data from clients. This function actually delegates a lower-level
// function that depends on the kind of service (raw, http, ...).
//
void netReadFromClients(void) {

    tClient *c = netAcceptClients();

    while (c) {
        // Read next before servicing client incase the service routine deletes the client! 
        tClient *next = c->next;

        if (c->fd < 0) {
            netFreeClient(c);
        }
        c = next;
    }
}
//
//=========================================================================
//
void netFlushClients(uint64_t lTime) {

    //Send any remaining partial raw buffers now
    if (Modes.avrOutUsed || Modes.beastOutUsed || Modes.rawOutUsed) {
        Modes.net_output_raw_rate_count++;
        if (Modes.net_output_raw_rate_count > Modes.net_output_raw_rate) {
            if (Modes.avrOutUsed) {
                netSendAllClients(Modes.avrSocket, Modes.avrOut, Modes.avrOutUsed);
                Modes.avrOutUsed = 0;
            }
            if (Modes.rawOutUsed) {
                netSendAllClients(Modes.rawSocket, Modes.rawOut, Modes.rawOutUsed);
                Modes.rawOutUsed = 0;
            }
            if (Modes.beastOutUsed) {
                netSendAllClients(Modes.beastSocket, Modes.beastOut, Modes.beastOutUsed);
                Modes.beastOutUsed = 0;
            }
            Modes.net_output_raw_rate_count = 0;
        }
    }
    // Nothing to send, so increase the heartbeat
    else if ((Modes.net_heartbeat_rate)
        && ((++Modes.net_heartbeat_count) > Modes.net_heartbeat_rate)) {
        //
        // We haven't received any Mode A/C/S messages for some time. To try and keep any TCP
        // links alive, send a null frame. This will help stop any routers discarding our TCP 
        // link which will cause an un-recoverable link error if/when a real frame arrives.   
        //
        // Fudge up a null message
        tModesMessage mm;
        memset(&mm, 0, sizeof(mm));

        mm.rm.uMsgBits = MODES_SHORT_MSG_BITS;
        mm.rm.lTime    = lTime;

        // Feed output clients
        netQueueOutput(&mm);

        // Reset the heartbeat counter
        Modes.net_heartbeat_count = 0;
    }
}
//
// =============================== Network IO ===========================
//
