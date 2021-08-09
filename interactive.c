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
static uint64_t mstime(void) {
    struct timeval tv;
    uint64_t mst;

    gettimeofday(&tv, NULL);
    mst = ((uint64_t)tv.tv_sec)*1000;
    mst += tv.tv_usec/1000;
    return mst;
}
//
//========================= Interactive mode ===============================
//
// Return a new aircraft structure for the interactive mode linked list
// of aircraft
//
tAircraft *interactiveCreateAircraft(tModesMessage *mm) {
    tAircraft *a = (tAircraft *) malloc(sizeof(*a));

    if (!a) { return (NULL);}

    // Default everything to zero/NULL
    memset(a, 0, sizeof(*a));

    // Now initialise things that should not be 0/NULL to their defaults
    a->addr = mm->rm.uAddr;
    a->lat  = a->lon = 0.0;
    memset(a->signalLevel, mm->signalLevel, 8); // First time, initialise everything
                                                // to the first signal strength

    // mm->msgtype 32 is used to represent Mode A/C. These values can never change, so 
    // set them once here during initialisation, and don't bother to set them every 
    // time this ModeA/C is received again in the future
    if (mm->rm.uMsgType == 32) {
        int modeC      = ModeAToModeC(mm->uModeA | mm->fs);
        a->modeACflags = MODEAC_MSG_FLAG;
        if (modeC < -12) {
            a->modeACflags |= MODEAC_MSG_MODEA_ONLY;
        } else {
            mm->altitude = modeC * 100;
            mm->bFlags  |= MODES_ACFLAGS_ALTITUDE_VALID;
        }
    }
    return (a);
}
//
//=========================================================================
//
// Return the aircraft with the specified address, or NULL if no aircraft
// exists with this address.
//
tAircraft *interactiveFindAircraft(uint32_t addr) {
    tAircraft *a = Modes.pAircraft;

    while(a) {
        if (a->addr == addr) return (a);
        a = a->next;
    }
    return (NULL);
}
//
//=========================================================================
//
// We have received a Mode A or C response. 
//
// Search through the list of known Mode-S aircraft and tag them if this Mode A/C 
// matches their known Mode S Squawks or Altitudes(+/- 50feet).
//
// A Mode S equipped aircraft may also respond to Mode A and Mode C SSR interrogations.
// We can't tell if this is a Mode A or C, so scan through the entire aircraft list
// looking for matches on Mode A (squawk) and Mode C (altitude). Flag in the Mode S
// records that we have had a potential Mode A or Mode C response from this aircraft. 
//
// If an aircraft responds to Mode A then it's highly likely to be responding to mode C 
// too, and vice verca. Therefore, once the mode S record is tagged with both a Mode A
// and a Mode C flag, we can be fairly confident that this Mode A/C frame relates to that
// Mode S aircraft.
//
// Mode C's are more likely to clash than Mode A's; There could be several aircraft 
// cruising at FL370, but it's less likely (though not impossible) that there are two 
// aircraft on the same squawk. Therefore, give precidence to Mode A record matches
//
// Note : It's theoretically possible for an aircraft to have the same value for Mode A 
// and Mode C. Therefore we have to check BOTH A AND C for EVERY S.
//
void interactiveUpdateAircraftModeA(tAircraft *a) {
    tAircraft *b = Modes.pAircraft;

    while(b) {
        if ((b->modeACflags & MODEAC_MSG_FLAG) == 0) {// skip any fudged ICAO records 

            // If both (a) and (b) have valid squawks...
            if ((a->bFlags & b->bFlags) & MODES_ACFLAGS_SQUAWK_VALID) {
                // ...check for Mode-A == Mode-S Squawk matches
                if (a->uModeA == b->uModeA) { // If a 'real' Mode-S ICAO exists using this Mode-A Squawk
                    b->modeAcount   = a->messages;
                    b->modeACflags |= MODEAC_MSG_MODEA_HIT;
                    a->modeACflags |= MODEAC_MSG_MODEA_HIT;
                    if ( (b->modeAcount > 0) &&
                       ( (b->modeCcount > 1)
                      || (a->modeACflags & MODEAC_MSG_MODEA_ONLY)) ) // Allow Mode-A only matches if this Mode-A is invalid Mode-C
                        {a->modeACflags |= MODEAC_MSG_MODES_HIT;}    // flag this ModeA/C probably belongs to a known Mode S                    
                }
            }

            // If both (a) and (b) have valid altitudes...
            if ((a->bFlags & b->bFlags) & MODES_ACFLAGS_ALTITUDE_VALID) {
                // ... check for Mode-C == Mode-S Altitude matches
                if (  (a->modeC     == b->modeC    )     // If a 'real' Mode-S ICAO exists at this Mode-C Altitude
                   || (a->modeC     == b->modeC + 1)     //          or this Mode-C - 100 ft
                   || (a->modeC + 1 == b->modeC    ) ) { //          or this Mode-C + 100 ft
                    b->modeCcount   = a->messages;
                    b->modeACflags |= MODEAC_MSG_MODEC_HIT;
                    a->modeACflags |= MODEAC_MSG_MODEC_HIT;
                    if ( (b->modeAcount > 0) &&
                         (b->modeCcount > 1) )
                        {a->modeACflags |= (MODEAC_MSG_MODES_HIT | MODEAC_MSG_MODEC_OLD);} // flag this ModeA/C probably belongs to a known Mode S                    
                }
            }
        }
        b = b->next;
    }
}
//
//=========================================================================
//
void interactiveUpdateAircraftModeS() {
    tAircraft *a = Modes.pAircraft;

    while(a) {
        int flags = a->modeACflags;
        if (flags & MODEAC_MSG_FLAG) { // find any fudged ICAO records

            // clear the current A,C and S hit bits ready for this attempt
            a->modeACflags = flags & ~(MODEAC_MSG_MODEA_HIT | MODEAC_MSG_MODEC_HIT | MODEAC_MSG_MODES_HIT);

            interactiveUpdateAircraftModeA(a);  // and attempt to match them with Mode-S
        }
        a = a->next;
    }
}
//
//=========================================================================
//
// Receive new messages and populate the interactive mode with more info
//
tAircraft *interactiveReceiveData(tModesMessage *mm) {
    tAircraft* a;

    // Lookup our aircraft or create a new one
    a = interactiveFindAircraft(mm->rm.uAddr);
    if (!a) {                              // If it's a currently unknown aircraft....
        a = interactiveCreateAircraft(mm); // ., create a new record for it,

        if (!a) { return (NULL); }         // If we failed to create a new aircraft return now

        a->next = Modes.pAircraft;         // .. and put it at the head of the list
        Modes.pAircraft = a;
    }

    a->signalLevel[a->messages & 7] = mm->signalLevel;// replace the 8th oldest signal strength
    a->seen      = time(NULL);
    a->timestamp = mm->rm.lTime;
    a->messages++;

    // If a (new) CALLSIGN has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_CALLSIGN_VALID) {
        memcpy(a->flight, mm->flight, sizeof(a->flight));
    }

    // If a (new) ALTITUDE has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
        if ( (a->modeCcount)                   // if we've a modeCcount already
          && (a->altitude  != mm->altitude ) ) // and Altitude has changed
//        && (a->modeC     != mm->modeC + 1)   // and Altitude not changed by +100 feet
//        && (a->modeC + 1 != mm->modeC    ) ) // and Altitude not changes by -100 feet
            {
            a->modeCcount   = 0;               //....zero the hit count
            a->modeACflags &= ~MODEAC_MSG_MODEC_HIT;
            }
        a->altitude = mm->altitude;
        a->modeC    = (mm->altitude + 49) / 100;
    }

    // If a (new) SQUAWK has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {
        if (a->uModeA != mm->uModeA) {
            a->modeAcount   = 0; // Squawk has changed, so zero the hit count
            a->modeACflags &= ~MODEAC_MSG_MODEA_HIT;
        }
        a->uModeA = mm->uModeA;
    }

    // If a (new) HEADING has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_HEADING_VALID) {
        a->track = mm->heading;
    }

    // If a (new) SPEED has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_SPEED_VALID) {
        a->speed = mm->velocity;
    }

    // If a (new) Vertical Descent rate has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_VERTRATE_VALID) {
        a->vert_rate = mm->vert_rate;
    }

    // if the Aircraft has landed or taken off since the last message, clear the even/odd CPR flags
    if ((mm->bFlags & MODES_ACFLAGS_AOG_VALID) && ((a->bFlags ^ mm->bFlags) & MODES_ACFLAGS_AOG)) {
        a->bFlags &= ~(MODES_ACFLAGS_LLBOTH_VALID | MODES_ACFLAGS_AOG);
    }

    // If we've got a new cprlat or cprlon
    if (mm->bFlags & MODES_ACFLAGS_LLEITHER_VALID) {
        int location_ok = 0;

        if (mm->bFlags & MODES_ACFLAGS_LLODD_VALID) {
            a->odd_cprlat  = mm->raw_latitude;
            a->odd_cprlon  = mm->raw_longitude;
            a->odd_cprtime = mstime();
        } else {
            a->even_cprlat  = mm->raw_latitude;
            a->even_cprlon  = mm->raw_longitude;
            a->even_cprtime = mstime();
        }

        // If we have enough recent data, try global CPR
        if (((mm->bFlags | a->bFlags) & MODES_ACFLAGS_LLEITHER_VALID) == MODES_ACFLAGS_LLBOTH_VALID && abs((int)(a->even_cprtime - a->odd_cprtime)) <= 10000) {
            if (decodeCPR(a, (mm->bFlags & MODES_ACFLAGS_LLODD_VALID), (mm->bFlags & MODES_ACFLAGS_AOG)) == 0) {
                location_ok = 1;
            }
        }

        // Otherwise try relative CPR.
        if (!location_ok && decodeCPRrelative(a, (mm->bFlags & MODES_ACFLAGS_LLODD_VALID), (mm->bFlags & MODES_ACFLAGS_AOG)) == 0) {
            location_ok = 1;
        }

        //If we sucessfully decoded, back copy the results to mm so that we can print them in list output
        if (location_ok) {
            mm->bFlags |= MODES_ACFLAGS_LATLON_VALID;
            mm->fLat    = a->lat;
            mm->fLon    = a->lon;
        }
    }

    // Update the aircrafts a->bFlags to reflect the newly received mm->bFlags;
    a->bFlags |= mm->bFlags;

    if (mm->rm.uMsgType == 32) {
        int flags = a->modeACflags;
        if ((flags & (MODEAC_MSG_MODEC_HIT | MODEAC_MSG_MODEC_OLD)) == MODEAC_MSG_MODEC_OLD) {
            //
            // This Mode-C doesn't currently hit any known Mode-S, but it used to because MODEAC_MSG_MODEC_OLD is
            // set  So the aircraft it used to match has either changed altitude, or gone out of our receiver range
            //
            // We've now received this Mode-A/C again, so it must be a new aircraft. It could be another aircraft
            // at the same Mode-C altitude, or it could be a new airctraft with a new Mods-A squawk.
            //
            // To avoid masking this aircraft from the interactive display, clear the MODEAC_MSG_MODES_OLD flag
            // and set messages to 1;
            //
            a->modeACflags = flags & ~MODEAC_MSG_MODEC_OLD;
            a->messages    = 1;
        }
    }

    return (a);
}
//
//=========================================================================
//
// This function gets a decoded Mode A/S Message and lists it on the screen
// in a human readable format.
//
char str[256];
void interactiveListData(tModesMessage* mm) {
    uint32_t j;
    char* p = str;
//  FILE* hFile;

    p += sprintf(p, "0;%06X;%08X;*", mm->rm.uCrc, (uint32_t)mm->rm.lTime);

    for (j = 0; j < mm->rm.uMsgBits / 8; j++) {
        p += sprintf(p, "%02X", mm->rm.ucMsg[j]);
    }
    p += sprintf(p, ";%08X;\n", mm->rm.signalLevel32);

    printf("%s", str);
/*
    if ((hFile = fopen("airspy1090_u17.txt", "a"))) {
        fprintf(hFile, "%s", str);
        fclose(hFile);
    }
*/
}
//
//=========================================================================
//
// Show the currently captured interactive data on screen.
//
void interactiveShowData(void) {
    tAircraft *a = Modes.pAircraft;
    time_t now = time(NULL);
    int count = 0;
    char progress;
    char spinner[4] = "|/-\\";

    // Refresh screen every (MODES_INTERACTIVE_REFRESH_TIME) miliseconde
    if ((mstime() - Modes.interactive_last_update) < MODES_INTERACTIVE_REFRESH_TIME)
       {return;}

    Modes.interactive_last_update = mstime();

    // Attempt to reconsile any ModeA/C with known Mode-S
    // We can't condition on Modes.modeac because ModeA/C could be comming
    // in from a raw input port which we can't turn off.
    interactiveUpdateAircraftModeS();

    progress = spinner[time(NULL)%4];

#ifndef _WIN32
    printf("\x1b[H\x1b[2J");    // Clear the screen
#else
    cls();
#endif

    printf("Hex     Mode  Sqwk  Flight   Alt    Spd  Hdg    Lat      Long   Sig  Msgs   Ti%c\n", progress);
    printf("-------------------------------------------------------------------------------\n");

    while(a && (count < Modes.interactive_rows)) {

        if ((now - a->seen) < Modes.interactive_display_ttl)
            {
            int msgs  = a->messages;
            int flags = a->modeACflags;

            if ( (((flags & (MODEAC_MSG_FLAG                             )) == 0                    )                 )
              || (((flags & (MODEAC_MSG_MODES_HIT | MODEAC_MSG_MODEA_ONLY)) == MODEAC_MSG_MODEA_ONLY) && (msgs > 4  ) ) 
              || (((flags & (MODEAC_MSG_MODES_HIT | MODEAC_MSG_MODEC_OLD )) == 0                    ) && (msgs > 127) ) 
              ) {
                int altitude = a->altitude, speed = a->speed;
                char strSquawk[5] = " ";
                char strFl[6]     = " ";
                char strTt[5]     = " ";
                char strGs[5]     = " ";

                // Convert units to metric if --metric was specified
/*
                if (Modes.metric) {
                    altitude = (int) (altitude / 3.2828);
                    speed    = (int) (speed    * 1.852);
                }
*/
                if (a->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {
                    snprintf(strSquawk,5,"%04x", a->uModeA);}

                if (a->bFlags & MODES_ACFLAGS_SPEED_VALID) {
                    snprintf (strGs, 5,"%3d", speed);}

                if (a->bFlags & MODES_ACFLAGS_HEADING_VALID) {
                    snprintf (strTt, 5,"%03d", a->track);}

                if (msgs > 99999) {
                    msgs = 99999;}

                {                         // Dump1090 display mode
                    char strMode[5]               = "    ";
                    char strLat[8]                = " ";
                    char strLon[9]                = " ";
                    unsigned char * pSig       = a->signalLevel;
                    unsigned int signalAverage = (pSig[0] + pSig[1] + pSig[2] + pSig[3] + 
                                                  pSig[4] + pSig[5] + pSig[6] + pSig[7] + 3) >> 3; 

                    if ((flags & MODEAC_MSG_FLAG) == 0) {
                        strMode[0] = 'S';
                    } else if (flags & MODEAC_MSG_MODEA_ONLY) {
                        strMode[0] = 'A';
                    }
                    if (flags & MODEAC_MSG_MODEA_HIT) {strMode[2] = 'a';}
                    if (flags & MODEAC_MSG_MODEC_HIT) {strMode[3] = 'c';}

                    if (a->bFlags & MODES_ACFLAGS_LATLON_VALID) {
                        snprintf(strLat, 8,"%7.03f", a->lat);
                        snprintf(strLon, 9,"%8.03f", a->lon);
                    }

                    if (a->bFlags & MODES_ACFLAGS_AOG) {
                        snprintf(strFl, 6," grnd");
                    } else if (a->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
                        snprintf(strFl, 6, "%5d", altitude);
                    }

                    printf("%06X  %-4s  %-4s  %-8s %5s  %3s  %3s  %7s %8s  %3d %5d   %2d\n",
                    a->addr, strMode, strSquawk, a->flight, strFl, strGs, strTt,
                    strLat, strLon, signalAverage, msgs, (int)(now - a->seen));
                }
                count++;
            }
        }
        a = a->next;
    }
}
//
//=========================================================================
//
// When in interactive mode If we don't receive new nessages within
// MODES_INTERACTIVE_DELETE_TTL seconds we remove the aircraft from the list.
//
void interactiveRemoveStaleAircrafts(void) {
    tAircraft *a = Modes.pAircraft;
    tAircraft *prev = NULL;
    time_t now = time(NULL);

    // Only do cleanup once per second
    if (Modes.last_cleanup_time != now) {
        Modes.last_cleanup_time = now;

        while(a) {
            if ((now - a->seen) > Modes.interactive_delete_ttl) {
                // Remove the element from the linked list, with care
                // if we are removing the first element
                if (!prev) {
                    Modes.pAircraft = a->next; free(a); a = Modes.pAircraft;
                } else {
                    prev->next = a->next; free(a); a = prev->next;
                }
            } else {
                prev = a; a = a->next;
            }
        }
    }
}
//
//=========================================================================
//
