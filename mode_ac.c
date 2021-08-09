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
// ===================== Mode A/C detection and decoding  ===================
//
// Input format is : 00:A4:A2:A1:00:B4:B2:B1:00:C4:C2:C1:00:D4:D2:D1
//
int ModeAToModeC(uint32_t ModeA) 
  { 
  unsigned int FiveHundreds = 0;
  unsigned int OneHundreds  = 0;

  if (  (ModeA & 0xFFFF888B)         // D1 set is illegal. D2 set is > 62700ft which is unlikely
    || ((ModeA & 0x000000F0) == 0) ) // C1,,C4 cannot be Zero
    {return -9999;}

  if (ModeA & 0x0010) {OneHundreds ^= 0x007;} // C1
  if (ModeA & 0x0020) {OneHundreds ^= 0x003;} // C2
  if (ModeA & 0x0040) {OneHundreds ^= 0x001;} // C4

  // Remove 7s from OneHundreds (Make 7->5, snd 5->7). 
  if ((OneHundreds & 5) == 5) {OneHundreds ^= 2;}

  // Check for invalid codes, only 1 to 5 are valid 
  if (OneHundreds > 5)
    {return -9999;} 

//if (ModeA & 0x0001) {FiveHundreds ^= 0x1FF;} // D1 never used for altitude
  if (ModeA & 0x0002) {FiveHundreds ^= 0x0FF;} // D2
  if (ModeA & 0x0004) {FiveHundreds ^= 0x07F;} // D4

  if (ModeA & 0x1000) {FiveHundreds ^= 0x03F;} // A1
  if (ModeA & 0x2000) {FiveHundreds ^= 0x01F;} // A2
  if (ModeA & 0x4000) {FiveHundreds ^= 0x00F;} // A4

  if (ModeA & 0x0100) {FiveHundreds ^= 0x007;} // B1 
  if (ModeA & 0x0200) {FiveHundreds ^= 0x003;} // B2
  if (ModeA & 0x0400) {FiveHundreds ^= 0x001;} // B4
    
  // Correct order of OneHundreds. 
  if (FiveHundreds & 1) {OneHundreds = 6 - OneHundreds;} 

  return ((FiveHundreds * 5) + OneHundreds - 13); 
  } 
//
//=========================================================================
//
static uint32_t AltToModeC(int iAlt) {

    uint32_t uFives[] = {4,6,2,3,1};
    uint32_t uTens[]  = {0,4,6,2,3,7,5,1};
    uint32_t uModeC   = 0;
    int      iRem;

    iAlt = ((iAlt + 49) / 100) + 12;

    iRem = iAlt % 10;
    iAlt = iAlt / 5;
    if (iRem > 4) {iRem = 9-iRem;}
    uModeC |= (uFives[iRem] << 4);

    iRem = iAlt & 15;
    iAlt = iAlt >> 3;
    if (iRem > 7) {iRem = 15-iRem;}
    uModeC |= (uTens[iRem] << 8);

    iRem = iAlt & 15;
    iAlt = iAlt >> 3;
    if (iRem > 7) {iRem = 15-iRem;}
    uModeC |= (uTens[iRem] << 12);

    iRem      = iAlt & 15;
    if (iRem > 7) {iRem = 15-iRem;}
    uModeC |= (uTens[iRem] << 0);
  
    return (uModeC);
}
//
//=========================================================================
//
void decodeModeA(tModesMessage *mm) {

    uint32_t uModeA = (mm->rm.ucMsg[0] << 8) | (mm->rm.ucMsg[1]);
    int      iModeC;
    uint32_t uSig;

    mm->rm.uMsgBits = 16; // This currently shows the number of bits set
    mm->rm.uCrc     = 0;

    // Fudge an ICAO address based on Mode A (remove the Ident bit)
    // Use an upper address byte of FF, since this is ICAO unallocated
    mm->rm.uAddr = 0x00FF0000 | (uModeA & 0x0000FF7F);

    // Set the Identity field to ModeA
    mm->uModeA  = uModeA & 0x7777;
    mm->bFlags |= MODES_ACFLAGS_SQUAWK_VALID;

    // Flag ident in flight status
    mm->fs = uModeA & 0x0080;

    // See if the ModeA is a valid ModeC Altitude
    iModeC = ModeAToModeC(uModeA);
    if (iModeC >= -12) {
        mm->altitude = iModeC * 100;
        mm->bFlags  |= MODES_ACFLAGS_ALTITUDE_VALID;
    }

    // Calculate the 8 bit signal level
    uSig            = mm->rm.signalLevel32 >> 12;
    mm->signalLevel = (unsigned char) ((uSig < 255) ? uSig : 255);
}
//
//=========================================================================
//
uint32_t validateModeA(tMessageRaw* rm) {

    uint32_t     uIndex;
    tModeACache* pCache;

    if (rm->nCount < 3) {
        // If this uModeA was seen for fewer than 3 phases, disregard it.
        // We might see up to 9 phases for each detection
        return (0);
    }

    // This uModeA was seen for 3 or more phases, so could be valid
    uIndex =  (rm->ucMsg[0] & 0x07)       | ((rm->ucMsg[0] & 0x70) >> 1)
           | ((rm->ucMsg[1] & 0x07) << 6) | ((rm->ucMsg[1] & 0x70) << 5);

    pCache = &Modes.ModeA_cache[uIndex];

    if      (pCache->lTime == 0) {
        // This is the first time we've seen this uModeA

    } else if ((rm->lTime - pCache->lTime) > MODEA_CACHE_TTL) {
        // If it's been more than MODEA_CACHE_TTL since the last time we saw this uModeA
        // ... clear both the Cache Counts
        pCache->uPrfCount = pCache->uRevCount = 0;

    } else if ((rm->lTime - pCache->lTime) > MODEA_CACHE_MAX_PRF) {
        // If it's been more than MODEA_CACHE_MAX_PRF but less than MODEA_CACHE_TTL since the
        // last time we saw this uModeA, then assume this is the next sweep of the radar head

        // ... If the previous sweeps PrfCount was 3 or more, increase the RevCount
        if ( (pCache->uPrfCount > 2)
          && (pCache->uRevCount < 65535) )
            {pCache->uRevCount++;}
        // ... and zero the PrfCount for this sweep
        pCache->uPrfCount = 0;

    } else if ((rm->lTime - pCache->lTime) > MODEA_CACHE_MIN_PRF) {
        // If it's been more than MODEA_CACHE_MIN_PRF but less than MODEA_CACHE_MAX_PRF since 
        // the since the last time we saw this uModeA, then assume this is the same sweep of 
        // the radar head.

        // ... increase the PrfCount for this sweep
        if (pCache->uPrfCount < 65535)
          {pCache->uPrfCount++;}
    }

    // Remember the time of this rm ready for next time.
    pCache->lTime = rm->lTime;

    // Returns '1' if we've seen (pCache->uPrfCount > 2) for more than (pCache->uRevCount > 1)
    return (pCache->uRevCount > 1);
}
//
//=========================================================================
//
// This uModeA has been seen through a validated ModeS Frame, so flag it
// as a valid ModeA in the ModeA cache
//
void updateModeA(tModesMessage *mm) {
    uint32_t     uIndex;
    tModeACache* pCache;

    uIndex =  (mm->uModeA & 0x0007)       | ((mm->uModeA & 0x0070) >> 1)
           | ((mm->uModeA & 0x0700) >> 2) | ((mm->uModeA & 0x7000) >> 3);

    pCache = &Modes.ModeA_cache[uIndex];
    pCache->lTime     = mm->rm.lTime;
    pCache->uRevCount = 65535;
}
//
//=========================================================================
//
// This Altitude has been seen through a validated ModeS Frame, so flag it
// as a valid ModeC in the ModeA cache
//
void updateModeC(tModesMessage *mm) {
    uint32_t     uIndex;
    tModeACache* pCache;
    uint32_t     uModeC = AltToModeC(mm->altitude);

    uIndex =  (uModeC & 0x0007)       | ((uModeC & 0x0070) >> 1)
           | ((uModeC & 0x0700) >> 2) | ((uModeC & 0x7000) >> 3);

    pCache = &Modes.ModeA_cache[uIndex];
    pCache->lTime     = mm->rm.lTime;
    pCache->uRevCount = 65535;
}
//
//=========================================================================
//
