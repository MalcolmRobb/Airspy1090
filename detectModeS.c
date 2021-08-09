// Airspy1090, a Mode 1/2/3/A/C/S messages decoder for Airspy devices.
//
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
//=========================================================================
//
// We are sampling at 9 times the ModeA bit rate, and 10 times the ModeS
// bit rate. Therefore we may get lots of (9 or 10) consecutive succesful
// decodes of the same message. We need to filter these so that we only 
// output one of them - preferably the best quality one.
//
// We can also count the number of consecutive decodes, and reject frames
// that don't have several identical decodes to improve noise rejection. 
//
static tMessageRaw** enQueueModeS(tMessageRaw* rm, tMessageRaw* *pQ) {

	tMessageRaw* qm = Modes.pModeS_Buffer[Modes.uModeS_Buffer & (MODES_DECODE_BUF_NUMBER - 1)];
	Modes.uModeS_Buffer++;
	memmove(qm, rm, sizeof(tMessageRaw));
    qm->pNext = NULL; // Belt and braces

	*pQ = qm; // Add this item to the end of the current queue.

    // Lump all Mode S decodes that occur within a preambles time (320 samples) together
    // ModeS messages can't be inteleaved, so this value could be set to anything from 
    // one bit period (10 samples) to one full frame (1440/2560 samples)
	Modes.uModeSTimeout = 320;

	return (&qm->pNext); // return the address of the pNext element of this qm.
}
//
//=========================================================================
//
static tMessageRaw** postDecodeModeS(void) {

   	// Put the latest decodes in the Decode Thread ModeS FIFO

    // Lock the Decode mutex
    pthread_mutex_lock(&Modes.Decode_mutex);

   	// If there is some space in the ModeS_Fifo
    if ((Modes.uDecode_ModeS_In - Modes.uDecode_ModeS_Out) < MODES_DECODE_FIFO_NUMBER) {
	    uint32_t uDecode_In = Modes.uDecode_ModeS_In & (MODES_DECODE_FIFO_NUMBER - 1);
   		Modes.Decode_ModeS_Fifo[uDecode_In] = Modes.pModeS_List;
    	Modes.uDecode_ModeS_In++;
  	}
    // Signal the Decode thread there is something to do.
    pthread_cond_signal(&Modes.Decode_cond);

   	// UnLock the Decode mutex
    pthread_mutex_unlock(&Modes.Decode_mutex);

   	// Start a new ModeS list
    Modes.pModeS_List = NULL;

    return (&Modes.pModeS_List);
}
//
// ===================== Mode S message validation  ===================
//
// Perform minimal message decoding to validate the DF against the Whitelist
// We will receive several phases of the same message for most decodes, so
// we need to minimise the time spent decoding before we chose the 'best'
//
#ifdef _ARMASM

uint32_t validateModeS(tMessageRaw* rm, tICAOCache* pWhitelist, uint64_t lTimeStamp) {

	__asm volatile(
//      "ldr     r0, [%1]\n\t"               //@ mov r0, rm
//      "ldr     r1, [%1]\n\t"               //@ mov r1, pCache

        "pld    [r0]\n\t"                    //@ Warn the memory system we're going to want rm

        "push   { r4-r12, lr }\n\t"          //@ Push the registers that need to be preserved onto the stack
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
        "mov     r12, r0\n\t"                //@ save rm     in r12
        "mov     r14, r1\n\t"                //@ save pCache in r14

        "add     r0, r0, #8\n\t"             //@ r0 = &rm->msg[0]
        "ldm     r0!, { r4, r5 }\n\t"        //@ r4 = msg[ 3]..msg[ 0], r5 = msg[ 7]..msg[ 4]

        "rev     r4, r4\n\t"                 //@ r4  = msg[ 0]..msg[ 3]
        "and     r10,r4, #0x00FFFFFF\n\t"    //@ r10 = uAddr (for DF11, DF17 & DF18)
        "lsr     r11,r4, #27\n\t"            //@ rll = DFType

        "mov     r6, #0x0831\n\t"
        "movt    r6, #0x0F7F\n\t"            //@ r6 = kDFValid
        "mov     r1, #1\n\t"
        "lsl     r1, r1, r11\n\t"            //@ r1 = (1 << DFType)
        "ands    r1, r1, r6\n\t"             //@ if ((r1 & kDFValid) == 0)....
        "beq     Fail\n\t"                   //@ .... goto Fail
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//      ModeS Checksum
//
        "mov     r6, #0x0480\n\t"
        "movt    r6, #0xFFFA\n\t"            //@ r6 = 0xFFFA0480

        "tst     r4, #0x80000000\n\t"
        "movne   r1, #11\n\t"                //@ if ((msg[0] & 0x80) == 0x80) byte count = (14-3);
        "moveq   r1, #4\n\t"                 //@ if ((msg[0] & 0x80) == 0x00) byte count = ( 7-3);

        "mov     r7, #0\n\t"                 //@ r7  = byte/word load counter = 0
        "b       CSumStart\n\t"

    "CSumLoop:\n\t"
        "lsl     r4, r4, #8\n\t"             //@ r4 = (r4 << 8)
        "bfi     r4, r5, #0, #8\n\t"         //@ r4 = (r4 << 8) | (r5 & 0xFF) 
        "lsr     r5, r5, #8\n\t"             //@ r5 = (r5 >> 8) 
        "add     r7, r7, #1\n\t"             //@ r7 = (r7 + 1)
        "ands    r7, r7, #3\n\t"             //@ r7 = (r7 & 3)
        "ldreq   r5, [r0], #4\n\t"           //@ if   (r7 == 0) {r5 = *r0++;}
    "CSumStart:\n\t"
        "tst     r4, #0x80000000\n\t"
        "eorne   r4, r4, r6\n\t"
        "tst     r4, #0x40000000\n\t"
        "eorne   r4, r4, r6, LSR#1\n\t"
        "tst     r4, #0x20000000\n\t"
        "eorne   r4, r4, r6, LSR#2\n\t"
        "tst     r4, #0x10000000\n\t"
        "eorne   r4, r4, r6, LSR#3\n\t"
        "tst     r4, #0x08000000\n\t"
        "eorne   r4, r4, r6, LSR#4\n\t"
        "tst     r4, #0x04000000\n\t"
        "eorne   r4, r4, r6, LSR#5\n\t"
        "tst     r4, #0x02000000\n\t"
        "eorne   r4, r4, r6, LSR#6\n\t"
        "tst     r4, #0x01000000\n\t"
        "eorne   r4, r4, r6, LSR#7\n\t"

        "subs    r1, r1, #1\n\t"             //@ Loop count --
        "bne     CSumLoop\n\t"               //@ if (Loop count != 0) goto CSumLoop
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//       # r0    = clobbered
//       # r1    = clobbered
//       # r2    = LOWORD(lTimeTag)
//       # r3    = HIWORD(lTimeTag)
//       # r4    = crc
//       # r5    = clobbered
//       # r6    = clobbered
//       # r7    = clobbered
//       # r8    = unused
//       # r9    = unused
//       # r10   = uAddr
//       # r11   = uDF_Type
//       # r12   = rm
//       # r14   = pWhitelist
//
//  Do checking dependent on DF Type
//
        "cmp     r11,#17\n\t"
        "beq     DF1718\n\t"
        "cmp     r11,#18\n\t"
        "beq     DF1718\n\t"
        "cmp     r11,#11\n\t"
        "bne     DFOthers\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// DF11 Checking
//
    "DF11:\n\t"
        "cmp    r4, #0\n\t"                   //@ if (crc == 0)....
        "beq    addRecentlySeenICAOAddr\n\t"  //@ .... goto addRecentlySeenICAOAddr

        "cmp    r4, #80\n\t"                  //@ if (crc < 80) ....
        "bcc    wasRecentlySeenICAOAddr\n\t"  //@ .... goto wasRecentlySeenICAOAddr

        "b      Fail\n\t"                     //@ else goto Fail
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// DF17 & DF18 Checking
//
    "DF1718:\n\t"
        "cmp    r4, #0\n\t"                   //@ if (crc != 0)....
        "bne    Fail\n\t"                     //@ ....goto Fail

    "addRecentlySeenICAOAddr:\n\t"
        "mov    r1, #0x9F3B\n\t"
        "movt   r1, #0x045D\n\t"              //@ r1 = 0x45d9f3b

        "mov    r0, r10\n\t"                  //@ r0 = uHash = uAddr;
        "eor    r0, r0 ,r0, LSR#16\n\t"       //@ r0 = uHash = ((uAddr >> 16) ^ uHash)
        "mul    r0, r0, r1\n\t"               //@ r0 = uHash = ((uAddr >> 16) ^ uHash) * 0x45d9f3b;
        "eor    r0, r0 ,r0, LSR#16\n\t"       //@ r0 = uHash = ((uHash >> 16) ^ uHash)
        "mul    r0, r0, r1\n\t"               //@ r0 = uHash = ((uHash >> 16) ^ uHash) * 0x45d9f3b;
        "eor    r0, r0 ,r0, LSR#16\n\t"       //@ r0 = uHash = ((uHash >> 16) ^ uHash)
        "bfc    r0, #10, #22\n\t"             //@ r0 = uHash = ((uHash >> 16) ^ uHash) & (MODES_ICAO_CACHE_LEN-1);

        "add    r0, r14, r0, LSL#4\n\t"       //@ r0 = &pWhitelist[uHash]
        "stm    r0, {r2, r3, r10}\n\t"        //@ pWhitelist[uHash].lTime = rm->timestampMsg
                                              //@ pWhitelist[uHash].uAddr = uAddr
        "b      Pass\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// All Other DF Checking
//
    "DFOthers:\n\t"
        "mov    r10, r4\n\t"                  //@ uAddr = crc;

    "wasRecentlySeenICAOAddr:"
        "mov    r1, #0x9F3B\n\t"
        "movt   r1, #0x045D\n\t"              //@ r1 = 0x45d9f3b

        "mov    r0, r10\n\t"                  //@ r0 = uHash = uAddr;
        "eor    r0, r0 ,r0, LSR#16\n\t"       //@ r0 = uHash = ((uAddr >> 16) ^ uHash)
        "mul    r0, r0, r1\n\t"               //@ r0 = uHash = ((uAddr >> 16) ^ uHash) * 0x45d9f3b;
        "eor    r0, r0 ,r0, LSR#16\n\t"       //@ r0 = uHash = ((uHash >> 16) ^ uHash)
        "mul    r0, r0, r1\n\t"               //@ r0 = uHash = ((uHash >> 16) ^ uHash) * 0x45d9f3b;
        "eor    r0, r0 ,r0, LSR#16\n\t"       //@ r0 = uHash = ((uHash >> 16) ^ uHash)
        "bfc    r0, #10, #22\n\t"             //@ r0 = uHash = ((uHash >> 16) ^ uHash) & (MODES_ICAO_CACHE_LEN-1);

        "add    r0, r14, r0, LSL#4\n\t"       //@ r0 = &pWhitelist[uHash]
        "ldm    r0, {r6, r7, r8}\n\t"         //@ r6 = LOWORD(pWhitelist[uHash].lTime)
                                              //@ r7 = HIWORD(pWhitelist[uHash].lTime)
                                              //@ r8 = pWhitelist[uHash].uAddr

        "cmp    r8, #0\n\t"                   //@ if (pWhitelist[uHash].uAddr == 0) ....
        "beq    Fail\n\t"                     //@ .... goto Fail

        "cmp    r8, r10\n\t"                  //@ if (pWhitelist[uHash].uAddr != uAddr)
        "bne    Fail\n\t"                     //@ .... goto Fail

        "subs   r6, r2, r6\n\t"               //@ r6 = LOWORD(rm->timestampMsg) - LOWORD(pWhitelist[uHash].lTime)
        "sbc    r7, r3, r7\n\t"               //@ r7 = HIWORD(rm->timestampMsg) - HIWORD(pWhitelist[uHash].lTime) - carry

        "mov    r1, #0x8C00\n\t"              //@ r1 = (MODES_ICAO_CACHE_TTL*MODES_DEFAULT_RATE)
        "movt   r1, #0x4786\n\t"              //@ r1 = 0x47868C00

        "subs   r6, r6, r1\n\t"
        "sbcs   r7, r7, #0\n\t"
        "bcs    Fail\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
    "Pass:\n\t"
        "add     r0, r12, #24\n\t"                //@ r0 = &rm->timetag
        "stm     r0, {r2, r3, r4, r10, r11}\n\t"  //@ rm->timetag = lTimetag;  
                                                  //@ rm->crc     = crc;
                                                  //@ rm->addr    = uAddr;
                                                  //@ rm->msgtype = (msg[0] >> 3);
        "mov     r3, #1\n\t"
        "b       Exit\n\t"

    "Fail:\n\t"
        "mov     r3, #0\n\t"

    "Exit:\n\t" 
        "mov     r0, r3\n\t" 

    "Done:\n\t" 
        "pop    { r4 - r12, lr }\n\t"         //@ Pop the registers that need to be preserved from the stack (no return for inline asm)
//	    "pop    { r4 - r12, pc }\n\t"         //@ Pop the registers that need to be preserved from the stack and return
    "Quit:\n\t" 

      : "=r" (rm) // Outputs in r0
      : "g" (rm), "g" (pWhitelist), "g" (lTimeStamp)
      : "r0","r1","r2","r3","cc","memory"
	);
}

#else

//
// ============================== ICAO Cache ==============================
//
// Add the specified entry to the cache of recently seen ICAO addresses.
// We also add a timestamp so that we can make sure that the
// entry is only valid for MODES_ICAO_CACHE_TTL seconds.
//
static void addRecentlySeenICAOAddr(uint32_t uAddr, tICAOCache* pWhitelist, uint64_t lTime) {
    uint32_t uHash;

    uHash = ((uAddr >> 16) ^ uAddr) * 0x45d9f3b;
    uHash = ((uHash >> 16) ^ uHash) * 0x45d9f3b;
    uHash = ((uHash >> 16) ^ uHash) & (MODES_ICAO_CACHE_LEN-1);

    pWhitelist[uHash].uAddr = uAddr;
    pWhitelist[uHash].lTime = lTime;
}
//
//=========================================================================
//
// Returns TRUE (1) if the specified ICAO address was seen in a DF format with
// proper checksum (not xored with address) no more than * MODES_ICAO_CACHE_TTL
// seconds ago. Otherwise returns 0.
//
static int wasRecentlySeenICAOAddr(uint32_t uAddr, tICAOCache* pWhitelist, uint64_t lTime) {
    uint32_t uHash;
    uint32_t a;
    uint64_t t;

    uHash = ((uAddr >> 16) ^ uAddr) * 0x45d9f3b;
    uHash = ((uHash >> 16) ^ uHash) * 0x45d9f3b;
    uHash = ((uHash >> 16) ^ uHash) & (MODES_ICAO_CACHE_LEN-1);

    a  = pWhitelist[uHash].uAddr;
    t  = pWhitelist[uHash].lTime;

    return ( (a) && (a == uAddr) && ( (lTime - t) <= MODES_ICAO_CACHE_TTL) );
}
//
//=========================================================================
//
static uint32_t modesChecksum(unsigned char* msg, int bits) {

    uint32_t uCRCPipe = (msg[0] << 16) | (msg[1] << 8) | msg[2];
    uint32_t uKey = 0xFFFA0480;
    int i;

    msg += 3; bits -= 24;
    for (i = 0; i < (bits / 8); i++) {
        uCRCPipe = (uCRCPipe << 8) | *msg++;

        if (uCRCPipe & 0x80000000) { uCRCPipe ^= (uKey); }
        if (uCRCPipe & 0x40000000) { uCRCPipe ^= (uKey >> 1); }
        if (uCRCPipe & 0x20000000) { uCRCPipe ^= (uKey >> 2); }
        if (uCRCPipe & 0x10000000) { uCRCPipe ^= (uKey >> 3); }
        if (uCRCPipe & 0x08000000) { uCRCPipe ^= (uKey >> 4); }
        if (uCRCPipe & 0x04000000) { uCRCPipe ^= (uKey >> 5); }
        if (uCRCPipe & 0x02000000) { uCRCPipe ^= (uKey >> 6); }
        if (uCRCPipe & 0x01000000) { uCRCPipe ^= (uKey >> 7); }
    }
    return (uCRCPipe);
}
//
//=========================================================================
//
uint32_t validateModeS(tMessageRaw* rm, tICAOCache* pWhitelist, uint64_t lTime) {
    uint32_t bCrcOk;
    uint32_t uAddr;
    uint32_t uCrc;

    // Get the DF message type ASAP as other operations depend on it
    uint32_t uMsgType = (rm->ucMsg[0] >> 3); // Downlink Format

    // Only DF0, DF4, DF5, DF11, DF16, DF17, DF18, DF19, DF20, DF21 and DF24-27 are valid
    if (0 == (kDFValid & (1 << uMsgType))) {
        return (0);
    }

    uCrc = modesChecksum(rm->ucMsg, rm->uMsgBits);

    if (uMsgType == 11) { // DF11
        uAddr = (rm->ucMsg[1] << 16) | (rm->ucMsg[2] << 8) | (rm->ucMsg[3]);
        if ((bCrcOk = (0 == uCrc))) {
            // DF 11 : if crc == 0 try to populate our ICAO addresses whitelist.
            addRecentlySeenICAOAddr(uAddr, pWhitelist, lTime);
        }
        else if (uCrc < 80) { // It's a possible DF11 with SI/II non zero
            bCrcOk = wasRecentlySeenICAOAddr(uAddr, pWhitelist, lTime);
        }

    }
    else if ((uMsgType == 17) | (uMsgType == 18)) { // DF17 & DF18
        uAddr = (rm->ucMsg[1] << 16) | (rm->ucMsg[2] << 8) | (rm->ucMsg[3]);
        if ((bCrcOk = (0 == uCrc))) {
            // DF17 & DF18 : if crc == 0 populate our ICAO addresses whitelist.
            addRecentlySeenICAOAddr(uAddr, pWhitelist, lTime);
        }

    }
    else { // All other DF's
     // Compare the checksum with the whitelist of recently seen ICAO 
     // addresses. If it matches one, then declare the message as valid
        bCrcOk = wasRecentlySeenICAOAddr((uAddr = uCrc), pWhitelist, lTime);
    }

    if (bCrcOk) {
        rm->uAddr    = uAddr;
        rm->uCrc     = uCrc;
        rm->uMsgType = uMsgType;
        rm->lTime    = lTime;
    }

    return (bCrcOk);
}
#endif
//
//=========================================================================
//
#ifdef _ARMNEON
//
// puAF and pLanes ***MUST*** all be DWORD aligned. If they aren't this code will fault.
//
uint32_t detectModeSArmNEON(uint16_t* puAF, tMessageRaw* pLane) {

    __asm volatile(
        ".arch armv7-a\n\t"
        ".fpu  neon\n\t"

//      "ldr     r0, [%1]\n\t"               //@ mov r0, puAF
//      "ldr     r1, [%2]\n\t"               //@ mov r1, pLanes

        "pld        [r0,   #0]\n\t"          //@ Warn the memory system we're going to want puAF[ 0]..puAF[31]
        "pld        [r0,  #64]\n\t"          //@ Warn the memory system we're going to want puAF[32]..puAF[63]

        "push       { r4-r12, lr }\n\t"      //@ Push the registers that need to be preserved onto the stack
        "vpush      { q4-q7 }\n\t"           //@ Push neon registers that need to be preserved onto the stack

        "mov        r8, #0\n\t"

        "vldm       r0, { q8-q13 }\n\t"      //@ q8  = d9:d8   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q9  = d19:d18 = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q10 = d21:d20 = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q11 = d23:d22 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q12 = d25:d24 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 
                                             //@ q13 = d27:d26 = uAF[47] : uAF[46] : uAF[45] : uAF[44]: uAF[43] : uAF[42] : uAF[41] : uAF[40] 

        "vext.8     q3, q9,  q10,  #4\n\t"   //@ q3 = d7:d6 = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10]
        "vext.8     q2, q11, q12, #12\n\t"   //@ q2 = d5:d4 = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 
        "vmax.U16   q3, q2,  q3\n\t"         //@ q3 = d7:d6 max(uAF[37],uAF[17]) : ..... : max(uAF[30],uAF[10])
        "vmax.U16   q3, q13, q3\n\t"         //@ q3 = d7:d6 max(uAF[47],uAF[37],uAF[17]) : ..... : max(uAF[40],uAF[30],uAF[10])

        "vext.8     q7, q10, q11,  #8\n\t"   //@ q7 = d11:d10 = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20]   
        "vmin.U16   q2, q8, q7\n\t"          //@ q2 = d5:d4 min(uAF[27],uAF[7]) : ..... : min(uAF[20],uAF[0])

        "vcgt.U16   q0, q2, q3\n\t"          //@ q0 = d1:d0 min(uAF[27],uAF[7]) > max(uAF[47],uAF[37],uAF[17]) : ..... : min(uAF[20],uAF[0]) > max(uAF[40],uAF[30],uAF[10])
                                             //@ q0  = Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
                                             //@       FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000

        "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
        "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00

        "vmov       r4, r5, d2\n\t"
        "orrs       r3, r4, r5\n\t"
        "beq        ExitNeon\n\t"

        "pld        [r0, #128]\n\t"          //@ Warn the memory system we're going to want puAF[64]..puAF[95]
        "pld        [r0, #192]\n\t"          //@ Warn the memory system we're going to want puAF[96]..puAF[128]

        "add        r3, r0, #96\n\t"
        "vldm       r3, { q9-q15 }\n\t"      //@ q9  = d19:d18 = uAF[ 55] : uAF[ 54] : uAF[ 53] : uAF[ 52]: uAF[51] : uAF[50] : uAF[49] : uAF[48] 
                                             //@ q10 = d21:d20 = uAF[ 63] : uAF[ 62] : uAF[ 61] : uAF[ 60]: uAF[59] : uAF[58] : uAF[57] : uAF[56] 
                                             //@ q11 = d23:d22 = uAF[ 71] : uAF[ 70] : uAF[ 69] : uAF[ 68]: uAF[67] : uAF[66] : uAF[65] : uAF[64] 
                                             //@ q12 = d25:d24 = uAF[ 79] : uAF[ 78] : uAF[ 77] : uAF[ 76]: uAF[75] : uAF[74] : uAF[73] : uAF[72] 
                                             //@ q13 = d27:d26 = uAF[ 87] : uAF[ 86] : uAF[ 85] : uAF[ 84]: uAF[83] : uAF[82] : uAF[81] : uAF[80] 
                                             //@ q14 = d29:d28 = uAF[ 95] : uAF[ 94] : uAF[ 93] : uAF[ 92]: uAF[91] : uAF[90] : uAF[89] : uAF[88] 
                                             //@ q15 = d31:d30 = uAF[103] : uAF[102] : uAF[101] : uAF[100]: uAF[99] : uAF[98] : uAF[97] : uAF[96] 

        "vext.8     q3, q9,  q10,  #4\n\t"   //@ q3  = d5:d4 = uAF[57] : uAF[56] : uAF[55] : uAF[54]: uAF[53] : uAF[52] : uAF[51] : uAF[50] 
        "vext.8     q2, q10, q11,  #8\n\t"   //@ q2  = d7:d6 = uAF[67] : uAF[66] : uAF[65] : uAF[64]: uAF[63] : uAF[62] : uAF[61] : uAF[60]
        "vmax.U16   q3, q2,  q3\n\t"         //@ q3  = d7:d6 max(uAF[67],uAF[57]) : ..... : max(uAF[60],uAF[50])
        "vmax.U16   q3, q13, q3\n\t"         //@ q3  = d7:d6 max(uAF[87],uAF[67],uAF[57]) : ..... : max(uAF[80],uAF[60],uAF[50])

        "vext.8     q6, q11, q12, #12\n\t"   //@ q6  = d13:d12 = uAF[77] : uAF[76] : uAF[75] : uAF[74]: uAF[73] : uAF[72] : uAF[71] : uAF[70]   
        "vext.8     q5, q14, q15,  #4\n\t"   //@ q5  = d11:d10 = uAF[97] : uAF[96] : uAF[95] : uAF[94]: uAF[93] : uAF[92] : uAF[91] : uAF[90]   
        "vmin.U16   q2, q5, q6\n\t"          //@ q2  = d5:d4 min(uAF[97],uAF[77]) : ..... : min(uAF[90],uAF[70])

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = d3:d2 min(uAF[97],uAF[77]) > max(uAF[87],uAF[67],uAF[57]) : ..... : min(uAF[90],uAF[70]) > max(uAF[80],uAF[60],uAF[50])

        "vand       q0, q1, q0\n\t"          //@ q0  = Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
                                             //@       FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000

        "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
        "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00

        "vmov       r4, r5, d3\n\t"
        "orrs       r3, r4, r5\n\t"
        "beq        ExitNeon\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
        "pld        [r0, #320]\n\t"          //@ Warn the memory system we're going to want puAF[160].....
        "pld        [r0, #384]\n\t"          //@ Warn the memory system we're going to want puAF[192].....

        "add        r0, r0, #320\n\t"        //@ r0 points to pPayload

                                             //@ q8  = d17:d16 = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4] : uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0]
                                             //@ q7  = d15:d14 = uAF[27] : uAF[26] : uAF[25] : uAF[24] : uAF[23] : uAF[22] : uAF[21] : uAF[20]
                                             //@ q6  = d13:d12 = uAF[77] : uAF[76] : uAF[75] : uAF[74] : uAF[73] : uAF[72] : uAF[71] : uAF[70]
                                             //@ q5  = d11:d10 = uAF[97] : uAF[96] : uAF[95] : uAF[94] : uAF[93] : uAF[92] : uAF[91] : uAF[90] 

        "vaddl.U16  q15,d11,d13\n\t"         //@ q15 = uAF[97]+uAF[77]                : uAF[96]+uAF[76]                : uAF[95]+uAF[75]                : uAF[94]+uAF[74]
        "vaddw.U16  q15,q15,d15\n\t"         //@ q15 = uAF[97]+uAF[77]+uAF[27]        : uAF[96]+uAF[76]+uAF[26]        : uAF[95]+uAF[75]+uAF[25]        : uAF[94]+uAF[74]+uAF[24]
        "vaddw.U16  q15,q15,d17\n\t"         //@ q15 = uAF[97]+uAF[77]+uAF[27]+uAF[7] : uAF[96]+uAF[76]+uAF[26]+uAF[6] : uAF[95]+uAF[75]+uAF[25]+uAF[5] : uAF[94]+uAF[74]+uAF[24]+uAF[4]

        "vaddl.U16  q14,d10,d12\n\t"         //@ q14 = uAF[93]+uAF[73]                : uAF[92]+uAF[72],               : uAF[91]+uAF[71]                : uAF[90]+uAF[70]
        "vaddw.U16  q14,q14,d14\n\t"         //@ q14 = uAF[93]+uAF[73]+uAF[23]        : uAF[92]+uAF[72]+uAF[22]        : uAF[91]+uAF[71]+uAF[21]        : uAF[90]+uAF[70]+uAF[20]
        "vaddw.U16  q14,q14,d16\n\t"         //@ q14 = uAF[93]+uAF[73]+uAF[23]+uAF[3] : uAF[92]+uAF[72]+uAF[22]+uAF[2] : uAF[91]+uAF[71]+uAF[21]+uAF[1] : uAF[90]+uAF[70]+uAF[20]+uAF[0]
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do the bit 0 of the data frame. This is the MSB bit of the DF field, and determines the total frame length (56 or 112 bits)
// Don't allow any "zero" guesses in this field 
//
        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 
        
        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = d3:d2   = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vmvn       q1, q1\n\t"              //@ q1  = 0000/FFFF    0000/FFFF    0000/FFFF    0000/FFFF    0000/FFFF    0000/FFFF    0000/FFFF    0000/FFFF

        "vand       q0, q1, q0\n\t"          //@ q0 &= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
                                             //@       FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000

        "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
        "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
        "vmov       r4, r5, d3\n\t"
        "orrs       r3, r4, r5\n\t"
        "beq        ExitNeon\n\t"            //@ return if all lanes invalid

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q7, q1, #15\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q3   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 1-4 of the DF field into Q12.
//
        //@ Allow One 'zero' count in the remainder of the DF Field
        "vbic.U16   q0, #1\n\t"              //@ q0  = Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
                                             //@       FFFE/0000    FFFE/0000    FFFE/0000    FFFE/0000    FFFE/0000    FFFE/0000    FFFE/0000    FFFE/0000

        "mov        r10, #2\n\t"             //@ r10 = loop count (2 bits per loop)
    "DFLoopNeon:\n\t"
        "vext.8     q2, q4, q5, #8\n\t"      //@ q2  = d5:d4   = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20] 
        "vext.8     q3, q5, q6, #12\n\t"     //@ q3  = d7:d6   = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q7, q7, #1\n\t"
        "vorr       q7, q7, q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 

        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q7, q7, #1\n\t"
        "vorr       q7, q7, q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "subs       r10,r10,#1\n\t"
        "bne        DFLoopNeon\n\t"

        "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
        "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
        "vmov       r4, r5, d3\n\t"
        "orrs       r3, r4, r5\n\t"
        "beq        ExitNeon\n\t"            //@ If more than one zero in DF field, exit
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 5-15 of the Payload into Q7. (We've already got Bit 5 in Q4-6)
//
//@ Allow Three 'zero' counts in the entire frame (two more than currently) 
        "vshr.U16   q1, q0, #15\n\t"    
        "vsub.U16   q0, q0, q1\n\t"    
        "vsub.U16   q0, q0, q1\n\t"    

        "mov        r10, #6\n\t"             //@ r10 = loop count (Two bits per loop)
        "b          Word1Bit5Neon\n\t"
    "Word1LoopNeon:\n\t"
        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 

        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q7, q7, #1\n\t"
        "vorr       q7, q7, q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

    "Word1Bit5Neon:\n\t"
        "vext.8     q2, q4, q5, #8\n\t"      //@ q2  = d5:d4   = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20] 
        "vext.8     q3, q5, q6, #12\n\t"     //@ q3  = d7:d6   = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q7, q7, #1\n\t"
        "vorr       q7, q7, q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "subs       r10,r10,#1\n\t"
        "bne        Word1LoopNeon\n\t"

//      "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
//      "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
//      "vmov       r4, r5, d3\n\t"
//      "orrs       r3, r4, r5\n\t"
//      "beq        ExitNeon\n\t"            //@ If more than three zeros in all frames, exit
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 16-31 of the Payload into Q8.
//
        "mov        r10, #8\n\t"             //@ r10 = loop count (Two bits per loop)
    "Word2LoopNeon:\n\t"
        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 

        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q8, q8, #1\n\t"
        "vorr       q8, q8, q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "vext.8     q2, q4, q5, #8\n\t"      //@ q2  = d5:d4   = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20] 
        "vext.8     q3, q5, q6, #12\n\t"     //@ q3  = d7:d6   = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q8, q8, #1\n\t"
        "vorr       q8, q8, q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "subs       r10,r10,#1\n\t"
        "bne        Word2LoopNeon\n\t"

//      "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
//      "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
//      "vmov       r4, r5, d3\n\t"
//      "orrs       r3, r4, r5\n\t"
//      "beq        ExitNeon\n\t"            //@ If more than three zeros in all frames, exit

        "vrev16.8   q8, q8\n\t"              //@ Byteswap Q8 
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 32-47 of the Payload into Q9.
//
        "mov        r10, #8\n\t"             //@ r10 = loop count (Two bits per loop)
    "Word3LoopNeon:\n\t"
        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 

        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q9, q9, #1\n\t"
        "vorr       q9, q9, q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "vext.8     q2, q4, q5, #8\n\t"      //@ q2  = d5:d4   = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20] 
        "vext.8     q3, q5, q6, #12\n\t"     //@ q3  = d7:d6   = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q9, q9, #1\n\t"
        "vorr       q9, q9, q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "subs       r10,r10,#1\n\t"
        "bne        Word3LoopNeon\n\t"

//      "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
//      "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
//      "vmov       r4, r5, d3\n\t"
//      "orrs       r3, r4, r5\n\t"
//      "beq        ExitNeon\n\t"            //@ If more than three zeros in all frames, exit

        "vrev16.8   q9, q9\n\t"              //@ Byteswap Q9
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 48-55 of the Payload into Q10.
//
        "vmov.I16   q10, #0\n\t"             //@ Zero out the whole word so we don't store garbage in byte 8 of short frames
        "mov        r10, #4\n\t"             //@ r10 = loop count (Two bits per loop)
    "Word4LoopNeon:\n\t"
        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 

        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q10,q10,#1\n\t"
        "vorr       q10,q10,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "vext.8     q2, q4, q5, #8\n\t"      //@ q2  = d5:d4   = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20] 
        "vext.8     q3, q5, q6, #12\n\t"     //@ q3  = d7:d6   = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q10,q10,#1\n\t"
        "vorr       q10,q10,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "subs       r10,r10,#1\n\t"
        "bne        Word4LoopNeon\n\t"

//      "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
//      "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
//      "vmov       r4, r5, d3\n\t"
//      "orrs       r3, r4, r5\n\t"
//      "beq        ExitNeon\n\t"            //@ If more than three zeros in all frames, exit
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//      Store any valid short frames.
//
        "vmvn       q1, q7\n\t"
        "vand       q1, q1, q0\n\t"
        "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
        "vmov       r4, r5, d3\n\t"
        "orrs       r3, r4, r5\n\t"
        "beq        StoreShortEnd\n\t"

        "mov        r9, #56\n\t"

        "vmov       r6, r7, d0\n\t"
        "vrev16.8   q7, q7\n\t"              // Byteswap q7 for now. We'll un-Byteswap it latar

        "tst        r4,#0x00000080\n\t"      //@ Lane 0 is valid and short
        "beq        Lane0ShortEnd\n\t"
        "mov        r2, r1\n\t"
        "vst1.32    d28[0], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d14[0],d16[0],d18[0],d20[0]}, [r2]\n\t"
        "bfc        r6, #0, #16\n\t"
        "orr        r8, r8, #0x01\n\t"
    "Lane0ShortEnd:\n\t" 

        "tst        r4,#0x00008000\n\t"      //@ Lane 1 is valid and short
        "beq        Lane1ShortEnd\n\t"
        "add        r2, r1, #0x40\n\t"
        "vst1.32    d28[1], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d14[1],d16[1],d18[1],d20[1]}, [r2]\n\t"
        "bfc        r6, #16, #16\n\t"
        "orr        r8, r8, #0x02\n\t"
    "Lane1ShortEnd:\n\t" 

        "tst        r4,#0x00800000\n\t"      //@ Lane 2 is valid and short
        "beq        Lane2ShortEnd\n\t"
        "add        r2, r1, #0x80\n\t"
        "vst1.32    d29[0], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d14[2],d16[2],d18[2],d20[2]}, [r2]\n\t"
        "bfc        r7, #0, #16\n\t"
        "orr        r8, r8, #0x04\n\t"
    "Lane2ShortEnd:\n\t" 

        "tst        r4,#0x80000000\n\t"      //@ Lane 3 is valid and short
        "beq        Lane3ShortEnd\n\t"
        "add        r2, r1, #0xC0\n\t"
        "vst1.32    d29[1], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d14[3],d16[3],d18[3],d20[3]}, [r2]\n\t"
        "bfc        r7, #16, #16\n\t"
        "orr        r8, r8, #0x08\n\t"
    "Lane3ShortEnd:\n\t" 

        "vmov       d0, r6, r7\n\t"

        "vmov       r6, r7, d1\n\t"

        "tst        r5,#0x00000080\n\t"      //@ Lane 4 is valid and short
        "beq        Lane4ShortEnd\n\t"
        "add        r2, r1, #0x100\n\t"
        "vst1.32    d30[0], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d15[0],d17[0],d19[0],d21[0]}, [r2]\n\t"
        "bfc        r6,  #0, #16\n\t"
        "orr        r8, r8, #0x10\n\t"
    "Lane4ShortEnd:\n\t" 

        "tst        r5,#0x00008000\n\t"      //@ Lane 5 is valid and short
        "beq        Lane5ShortEnd\n\t"
        "add        r2, r1, #0x140\n\t"
        "vst1.32    d30[1], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d15[1],d17[1],d19[1],d21[1]}, [r2]\n\t"
        "bfc        r6, #16, #16\n\t"
        "orr        r8, r8, #0x20\n\t"
    "Lane5ShortEnd:\n\t" 

        "tst        r5,#0x00800000\n\t"      //@ Lane 6 is valid and short
        "beq        Lane6ShortEnd\n\t"
        "add        r2, r1, #0x180\n\t"
        "vst1.32    d31[0], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d15[2],d17[2],d19[2],d21[2]}, [r2]\n\t"
        "bfc        r7,  #0, #16\n\t"
        "orr        r8, r8, #0x40\n\t"
    "Lane6ShortEnd:\n\t" 

        "tst        r5,#0x80000000\n\t"      //@ Lane 7 is valid and short
        "beq        Lane7ShortEnd\n\t"
        "add        r2, r1, #0x1C0\n\t"
        "vst1.32    d31[1], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d15[3],d17[3],d19[3],d21[3]}, [r2]\n\t"
        "bfc        r7, #16, #16\n\t"
        "orr        r8, r8, #0x80\n\t"
    "Lane7ShortEnd:\n\t" 

        "vmov       d1, r6, r7\n\t"
        "vrev16.8   q7, q7\n\t"              // Un-Byteswap q7

    "StoreShortEnd:\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// If one or more of the valid frames are long, then there is more to do. If not, then we're done
//
        "vand       q1, q7, q0\n\t"
        "vshr.U16   q1, q1, #15\n\t"
        "vuzp.8     d2, d3\n\t"              //@        01/00        01/00        01/00        01/00        01/00        01/00        01/00        01/00
        "vmov       r4, r5, d2\n\t"
        "orrs       r3, r4, r5\n\t"
        "beq        ExitNeon\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 56-63 of the Payload into Q10.
//
        "mov        r10, #4\n\t"             //@ r10 = loop count (Two bits per loop)
    "Word4LoopANeon:\n\t"
        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 

        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q10,q10,#1\n\t"
        "vorr       q10,q10,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "vext.8     q2, q4, q5, #8\n\t"      //@ q2  = d5:d4   = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20] 
        "vext.8     q3, q5, q6, #12\n\t"     //@ q3  = d7:d6   = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q10,q10,#1\n\t"
        "vorr       q10,q10,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "subs       r10,r10,#1\n\t"
        "bne        Word4LoopANeon\n\t"

//      "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
//      "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
//      "vmov       r4, r5, d3\n\t"
//      "orrs       r3, r4, r5\n\t"
//      "beq        ExitNeon\n\t"            //@ If more than three zeros in all frames, exit

        "vrev16.8   q10,q10\n\t"             //@ Byteswap Q10
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 64-79 of the Payload into Q11.
//
        "mov        r10, #8\n\t"             //@ r10 = loop count (Two bits per loop)
    "Word5LoopNeon:\n\t"
        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 

        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q11,q11,#1\n\t"
        "vorr       q11,q11,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "vext.8     q2, q4, q5, #8\n\t"      //@ q2  = d5:d4   = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20] 
        "vext.8     q3, q5, q6, #12\n\t"     //@ q3  = d7:d6   = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q11,q11,#1\n\t"
        "vorr       q11,q11,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "subs       r10,r10,#1\n\t"
        "bne        Word5LoopNeon\n\t"

//      "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
//      "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
//      "vmov       r4, r5, d3\n\t"
//      "orrs       r3, r4, r5\n\t"
//      "beq        ExitNeon\n\t"            //@ If more than three zeros in all frames, exit

        "vrev16.8   q11,q11\n\t"             //@ Byteswap Q11
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 80-95 of the Payload into Q12.
//
        "mov        r10, #8\n\t"             //@ r10 = loop count (Two bits per loop)
    "Word6LoopNeon:\n\t"
        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 

        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q12,q12,#1\n\t"
        "vorr       q12,q12,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "vext.8     q2, q4, q5, #8\n\t"      //@ q2  = d5:d4   = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20] 
        "vext.8     q3, q5, q6, #12\n\t"     //@ q3  = d7:d6   = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q12,q12,#1\n\t"
        "vorr       q12,q12,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "subs       r10,r10,#1\n\t"
        "bne        Word6LoopNeon\n\t"

//      "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
//      "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
//      "vmov       r4, r5, d3\n\t"
//      "orrs       r3, r4, r5\n\t"
//      "beq        ExitNeon\n\t"            //@ If more than three zeros in all frames, exit

        "vrev16.8   q12,q12\n\t"             //@ Byteswap Q12
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 96-111 of the Payload into Q13.
//
        "mov        r10, #8\n\t"             //@ r10 = loop count (Two bits per loop)
    "Word7LoopNeon:\n\t"
        "vldm       r0!, { q2-q6 }\n\t"      //@ q2  = d5:d4   = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4]: uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] 
                                             //@ q3  = d7:d6   = uAF[15] : uAF[14] : uAF[13] : uAF[12]: uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] 
                                             //@ q4  = d9:d8   = uAF[23] : uAF[22] : uAF[21] : uAF[20]: uAF[19] : uAF[18] : uAF[17] : uAF[16] 
                                             //@ q5  = d11:d10 = uAF[31] : uAF[30] : uAF[29] : uAF[28]: uAF[27] : uAF[26] : uAF[25] : uAF[24] 
                                             //@ q6  = d13:d12 = uAF[39] : uAF[38] : uAF[37] : uAF[36]: uAF[35] : uAF[34] : uAF[33] : uAF[32] 

        "vext.8     q3, q3, q4, #4\n\t"      //@ q3  = d7:d6   = uAF[17] : uAF[16] : uAF[15] : uAF[14]: uAF[13] : uAF[12] : uAF[11] : uAF[10] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q13,q13,#1\n\t"
        "vorr       q13,q13,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "vext.8     q2, q4, q5, #8\n\t"      //@ q2  = d5:d4   = uAF[27] : uAF[26] : uAF[25] : uAF[24]: uAF[23] : uAF[22] : uAF[21] : uAF[20] 
        "vext.8     q3, q5, q6, #12\n\t"     //@ q3  = d7:d6   = uAF[37] : uAF[36] : uAF[35] : uAF[34]: uAF[33] : uAF[32] : uAF[31] : uAF[30] 

        "vcgt.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[7] > uAF[17]) : ..... : (uAF[0] > uAF[10])
        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q13,q13,#1\n\t"
        "vorr       q13,q13,q1\n\t"

        "vceq.U16   q1, q2, q3\n\t"          //@ q1  = (uAF[17] == uAF[7]) : ..... : (uAF[10] == uAF[0])
                                             //@ q1  = FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000    FFFF/0000
        "vshr.U16   q1, q1, #15\n\t"         //@ q1  = 0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000    0001/0000
        "vadd.U16   q0, q0, q1\n\t"

        "vmax.U16   q1, q2, q3\n\t"          //@ q1   = max(uAF[17],uAF[7]) : ..... : max(uAF[10],uAF[0])
        "vaddw.U16  q15,q15,d3\n\t"          //@ q15 += max(uAF[17],uAF[7]) : max(uAF[16],uAF[6]) : max(uAF[15],uAF[5]) : max(uAF[14],uAF[4])
        "vaddw.U16  q14,q14,d2\n\t"          //@ q14 += max(uAF[13],uAF[3]) : max(uAF[12],uAF[2]) : max(uAF[11],uAF[1]) : max(uAF[10],uAF[0])

        "subs       r10,r10,#1\n\t"
        "bne        Word7LoopNeon\n\t"

        "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
        "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
        "vmov       r4, r5, d3\n\t"
        "orrs       r3, r4, r5\n\t"
        "beq        ExitNeon\n\t"            //@ If more than three zeros in all frames, exit

        "vrev16.8   q13,q13\n\t"             //@ Byteswap Q13
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//      Store any valid long frames.
//
        "vand       q1, q7, q0\n\t"
        "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00
        "vmov       r4, r5, d3\n\t"

        "mov        r9, #112\n\t"
        "vrev16.8   q7, q7\n\t"              //@ Byteswap Q7. We haven't done it yet because we preserve bit15 as the long/short frame bit 

        "tst        r4,#0x00000080\n\t"      //@ Lane 0 is valid and long
        "beq        Lane0LongEnd\n\t"
        "mov        r2, r1\n\t"
        "vst1.32    d28[0], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d14[0],d16[0],d18[0],d20[0]}, [r2]!\n\t"
        "vst3.16    {d22[0],d24[0],d26[0]}, [r2]\n\t"
        "orr        r8, r8, #0x01\n\t"
    "Lane0LongEnd:\n\t" 

        "tst        r4,#0x00008000\n\t"      //@ Lane 1 is valid and long
        "beq        Lane1LongEnd\n\t"
        "add        r2, r1, #0x40\n\t"
        "vst1.32    d28[1], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d14[1],d16[1],d18[1],d20[1]}, [r2]!\n\t"
        "vst3.16    {d22[1],d24[1],d26[1]}, [r2]\n\t"
        "orr        r8, r8, #0x02\n\t"
    "Lane1LongEnd:\n\t" 

        "tst        r4,#0x00800000\n\t"      //@ Lane 2 is valid and long
        "beq        Lane2LongEnd\n\t"
        "add        r2, r1, #0x80\n\t"
        "vst1.32    d29[0], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d14[2],d16[2],d18[2],d20[2]}, [r2]!\n\t"
        "vst3.16    {d22[2],d24[2],d26[2]}, [r2]\n\t"
        "orr        r8, r8, #0x04\n\t"
    "Lane2LongEnd:\n\t" 

        "tst        r4,#0x80000000\n\t"      //@ Lane 3 is valid and long
        "beq        Lane3LongEnd\n\t"
        "add        r2, r1, #0xC0\n\t"
        "vst1.32    d29[1], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d14[3],d16[3],d18[3],d20[3]}, [r2]!\n\t"
        "vst3.16    {d22[3],d24[3],d26[3]}, [r2]\n\t"
        "orr        r8, r8, #0x08\n\t"
    "Lane3LongEnd:\n\t" 

        "tst        r5,#0x00000080\n\t"      //@ Lane 4 is valid and long
        "beq        Lane4LongEnd\n\t"
        "add        r2, r1, #0x100\n\t"
        "vst1.32    d30[0], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d15[0],d17[0],d19[0],d21[0]}, [r2]!\n\t"
        "vst3.16    {d23[0],d25[0],d27[0]}, [r2]\n\t"
        "orr        r8, r8, #0x10\n\t"
    "Lane4LongEnd:\n\t" 

        "tst        r5,#0x00008000\n\t"      //@ Lane 5 is valid and long
        "beq        Lane5LongEnd\n\t"
        "add        r2, r1, #0x140\n\t"
        "vst1.32    d30[1], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d15[1],d17[1],d19[1],d21[1]}, [r2]!\n\t"
        "vst3.16    {d23[1],d25[1],d27[1]}, [r2]\n\t"
        "orr        r8, r8, #0x20\n\t"
    "Lane5LongEnd:\n\t" 

        "tst        r5,#0x00800000\n\t"      //@ Lane 6 is valid and long
        "beq        Lane6LongEnd\n\t"
        "add        r2, r1, #0x180\n\t"
        "vst1.32    d31[0], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d15[2],d17[2],d19[2],d21[2]}, [r2]!\n\t"
        "vst3.16    {d23[2],d25[2],d27[2]}, [r2]\n\t"
        "orr        r8, r8, #0x40\n\t"
    "Lane6LongEnd:\n\t" 

        "tst        r5,#0x80000000\n\t"      //@ Lane 7 is valid and long
        "beq        Lane7LongEnd\n\t"
        "add        r2, r1, #0x1C0\n\t"
        "vst1.32    d31[1], [r2]!\n\t"
        "str        r9, [r2], #4\n\t"
        "vst4.16    {d15[3],d17[3],d19[3],d21[3]}, [r2]!\n\t"
        "vst3.16    {d23[3],d25[3],d27[3]}, [r2]\n\t"
        "orr        r8, r8, #0x80\n\t"
    "Lane7LongEnd:\n\t" 
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
    "ExitNeon:\n\t" 
        "mov        r3, r8\n\t"
        "mov        r0, r3\n\t" 
        "vpop       { q4 - q7 }\n\t"              //@ Pop the neon registers that need to be preserved from the stack
        "pop        { r4 - r12, lr }\n\t"         //@ Pop the registers that need to be preserved from the stack (no return for inline asm)

        : "=r" (puAF) // Outputs in r0
        : "g" (puAF), "g" (pLane)
        : "r0", "r1", "r2", "r3", "cc", "memory"
        );
}

//
//=========================================================================
//
// Detect a Mode A/S messages inside the magnitude buffer pointed by 'm' and of
// size 'mlen' bytes. Every detected Mode S message is converted into a
// stream of bits and passed to the function to display it.
//

static tMessageRaw rmLane[8];

static void detectBufferModeS(uint16_t* puAF, uint64_t lTime) {

	uint32_t j, uLanes;
	tMessageRaw** pQ;

	memset(&rmLane, 0, sizeof(tMessageRaw) * 8);

	// Find the end of any current queue
	pQ = &Modes.pModeS_List;
    while (*pQ) {
		pQ = & (*pQ)->pNext;
	}

	for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j += 8) {

		if ((uLanes = detectModeSArmNEON(puAF, rmLane))) {

			if ((uLanes & 0x01) && (validateModeS(&rmLane[0], Modes.icao_cache, lTime)))
				{pQ = enQueueModeS(&rmLane[0], pQ);}

			if ((uLanes & 0x02) && (validateModeS(&rmLane[1], Modes.icao_cache, lTime+1)))
				{pQ = enQueueModeS(&rmLane[1], pQ);}

			if ((uLanes & 0x04) && (validateModeS(&rmLane[2], Modes.icao_cache, lTime+2)))
				{pQ = enQueueModeS(&rmLane[2], pQ);}

			if ((uLanes & 0x08) && (validateModeS(&rmLane[3], Modes.icao_cache, lTime+3)))
				{pQ = enQueueModeS(&rmLane[3], pQ);}

			if ((uLanes & 0x10) && (validateModeS(&rmLane[4], Modes.icao_cache, lTime+4)))
				{pQ = enQueueModeS(&rmLane[4], pQ);}

			if ((uLanes & 0x20) && (validateModeS(&rmLane[5], Modes.icao_cache, lTime+5)))
				{pQ = enQueueModeS(&rmLane[5], pQ);}

			if ((uLanes & 0x40) && (validateModeS(&rmLane[6], Modes.icao_cache, lTime+6)))
				{pQ = enQueueModeS(&rmLane[6], pQ);}

			if ((uLanes & 0x80) && (validateModeS(&rmLane[7], Modes.icao_cache, lTime+7)))
				{pQ = enQueueModeS(&rmLane[7], pQ);}

        } else if (Modes.uModeSTimeout > 8) {
            Modes.uModeSTimeout -= 8;

        } else if (Modes.uModeSTimeout) {
            Modes.uModeSTimeout = 0;
            pQ = postDecodeModeS();
        }

		lTime += 8;
		puAF += 8;
	}
}

#elif _ARMSIMD

//
// puAF and pLane ***MUST*** all be DWORD aligned. If they aren't this code will fault.
//
uint32_t detectModeSArmSIMD(uint16_t* puAF, tMessageRaw* pLane) {

	__asm volatile(
//      "ldr     r0, [%1]\n\t"               //@ mov r0, puAF
//      "ldr     r1, [%2]\n\t"               //@ mov r1, pLane

        "push   { r4-r12, lr }\n\t"          //@ Push the registers that need to be preserved onto the stack

        "mov     r14,#0x8000\n\t"            //@ r14 = 0x80008000
        "orr     r14,r14,LSL#16\n\t"
        "add     r2,r1,#0x40\n\t"            //@ r2 = pLane2 = pLane1 + sizeof(struct stMessageRaw)
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
        "ldr     r4, [r0,  #0]\n\t"          //@ r4    = pPreamble[ 1- 0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r6, [r0, #20]\n\t"          //@ r6    = pPreamble[11-10] Pick up two uAF input values r6.h = uAF[11], r6.l = uAF[10]
        "usub16  r10,r6, r4\n\t"             //@ r10.h will be -ve if (uAF[1] > uAF[11])
                                             //@ r10.l will be -ve if (uAF[0] > uAF[10])

        "ands    r3, r10,r14\n\t"            //@ r3.h = (uAF[ 1] > uAF[11])
                                             //@ r3.l = (uAF[ 0] > uAF[10])
        "beq     Exit\n\t"                   //@ If both sign bits are '0' return now

        "ldr     r5, [r0, #40]\n\t"          //@ r5    = pPreamble[21-20] Pick up two uAF input values r4.h = uAF[21], r4.l = uAF[20]
        "ldr     r7, [r0, #60]\n\t"          //@ r7    = pPreamble[31-30] Pick up two uAF input values r6.h = uAF[31], r6.l = uAF[30]
        "usub16  r11,r7, r5\n\t"             //@ r11.h will be -ve if (uAF[21] > uAF[31])
                                             //@ r11.l will be -ve if (uAF[20] > uAF[30])

        "ands    r3, r3, r11\n\t"            //@ r3.h &= (uAF[21] > uAF[31])
                                             //@ r3.l &= (uAF[20] > uAF[30])
        "beq     Exit\n\t"                   //@ If both sign bits are '0' return now
//
//      r10.h = min(uAF[21],uAF[ 1]),         r10.l = min(uAF[20],uAF[ 0])
//
        "usub16  r10,r4, r5\n\t"             //@ r10.h = uAF[ 1] - uAF[21], GE[3:2] set if no borrow, clear if borrow
                                             //@ r10.l = uAF[ 0] - uAF[20], GE[1:0] set if no borrow, clear if borrow
        "sel     r10,r5, r4\n\t"             //@ r10.h = (uAF[ 1] > uAF[21]) ? uAF[ 1] : uAF[21]
                                             //@ r10.l = (uAF[ 0] > uAF[20]) ? uAF[ 0] : uAF[20]
//
//      r11.h = max(uAF[41],uAF[31],uAF[11]), r11.l = max(uAF[40],uAF[30],uAF[10])
//
        "ldr     r11,[r0, #80]\n\t"          //@ r11   = pPreamble[41-40] Pick up two uAF input values r14.h = uAF[41], r14.l = uAF[40]
        "usub16  r12,r11,r6\n\t"             //@ r12.h =  uAF[41] - uAF[11], GE[3:2] set if no borrow, clear if borrow
                                             //@ r12.l =  uAF[40] - uAF[10], GE[1:0] set if no borrow, clear if borrow
        "sel     r11,r11,r6\n\t"             //@ r11.h = (uAF[41] > uAF[11]) ? uAF[41] : uAF[11]
                                             //@ r11.l = (uAF[40] > uAF[10]) ? uAF[40] : uAF[10]
        "usub16  r12,r11,r7\n\t"             //@ r12.h =  max(uAF[41],uAF[11]) - uAF[31], GE[3:2] set if no borrow, clear if borrow
                                             //@ r12.l =  max(uAF[40],uAF[10]) - uAF[30], GE[1:0] set if no borrow, clear if borrow
        "sel     r11,r11,r7\n\t"             //@ r11.h = (max(uAF[41],uAF[11]) > uAF[31]) ? max(uAF[41],uAF[11]) : uAF[31]
                                             //@ r11.l = (max(uAF[40],uAF[10]) > uAF[30]) ? max(uAF[40],uAF[10]) : uAF[30]
//
//      r3.h &= min(uAF[21],uAF[ 1]) > max(uAF[41],uAF[31],uAF[11])
//      r3.l &= min(uAF[20],uAF[ 0]) > max(uAF[40],uAF[30],uAF[10])
//
        "usub16  r10,r11,r10\n\t"            //@ r10.h  = max(uAF[41],uAF[31],uAF[11]) - min(uAF[21],uAF[ 1]), GE[3:2] set if no borrow, clear if borrow
                                             //@ r10.l  = max(uAF[40],uAF[30],uAF[10]) - min(uAF[20],uAF[ 0]), GE[1:0] set if no borrow, clear if borrow
        "ands    r3, r3, r10\n\t"            //@ if both lanes are false / zero, then both lanes fail, so ...
        "beq     Exit\n\t"                   //@ ...return now
//
//      Add uAF[01-00] & uAF[21-20] to the signal strength
//
        "uxth    r8, r4\n\t"                 //@ r8    = sigStrength1  = r4.l
        "uxth    r9, r4,ROR#16\n\t"          //@ r9    = sigStrength2  = r4.h
        "uxtah   r8, r8, r5\n\t"             //@ r8    = sigStrength1 += r5.l
        "uxtah   r9, r9, r5,ROR#16\n\t"      //@ r9    = sigStrength2 += r5.h
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
        "ldr     r6, [r0, #120]\n\t"         //@ r6    = pPreamble[61-60] Pick up two uAF input values r6.h  = uAF[61], r6.l  = uAF[60]
        "ldr     r4, [r0, #140]\n\t"         //@ r4    = pPreamble[71-70] Pick up two uAF input values r4.h  = uAF[71], r4.l  = uAF[70]
        "usub16  r10,r6, r4\n\t"             //@ r10.h will be -ve if (uAF[71] > uAF[61])
                                             //@ r10.l will be -ve if (uAF[70] > uAF[60])

        "ands    r3, r3, r10\n\t"            //@ r3.h &= (uAF[71] > uAF[61])
                                             //@ r3.l &= (uAF[70] > uAF[60])
        "beq     Exit\n\t"                   //@ ...return now

        "ldr     r7, [r0, #160]\n\t"         //@ r7    = pPreamble[81-80] Pick up two uAF input values r7.h  = uAF[81], r7.l  = uAF[80]
        "ldr     r5, [r0, #180]\n\t"         //@ r5    = pPreamble[91-90] Pick up two uAF input values r5.h  = uAF[91], r5.l  = uAF[90]
        "usub16  r11,r7, r5\n\t"             //@ r11.h will be -ve if (uAF[91] > uAF[81])
                                             //@ r11.l will be -ve if (uAF[90] > uAF[80])

        "ands    r3, r3, r11\n\t"            //@ r3.h &= (uAF[91] > uAF[81])
                                             //@ r3.l &= (uAF[90] > uAF[80])
        "beq     Exit\n\t"                   //@ ...return now
//
//      r10.h = min(uAF[91],uAF[71]), r10.l = min(uAF[90],uAF[70])
//
        "usub16  r10,r4, r5\n\t"             //@ r10.h =  uAF[71] - uAF[91], GE[3:2] set if no borrow, clear if borrow
                                             //@ r10.l =  uAF[70] - uAF[90], GE[1:0] set if no borrow, clear if borrow
        "sel     r10,r5, r4\n\t"             //@ r10.h = (uAF[71] > uAF[91]) ? uAF[71] : uAF[91]
                                             //@ r10.l = (uAF[70] > uAF[90]) ? uAF[70] : uAF[90]
//
//      r11.h = max(uAF[81],uAF[61],uAF[51]), r11.l = max(uAF[80],uAF[60],uAF[50])
//
        "ldr     r11,[r0, #100]\n\t"         //@ r11   = pPreamble[51-50] Pick up two uAF input values r11.h = uAF[51], r11.l = uAF[50]
        "usub16  r12,r11,r6\n\t"             //@ r12.h =  uAF[51] - uAF[61], GE[3:2] set if no borrow, clear if borrow
                                             //@ r12.l =  uAF[50] - uAF[60], GE[1:0] set if no borrow, clear if borrow
        "sel     r11,r11,r6\n\t"             //@ r11.h = (uAF[51] > uAF[61]) ? uAF[51] : uAF[61]
                                             //@ r11.l = (uAF[50] > uAF[60]) ? uAF[50] : uAF[60]
        "usub16  r12,r11,r7\n\t"             //@ r12.h =  max(uAF[61],uAF[51]) - uAF[81], GE[3:2] set if no borrow, clear if borrow
                                             //@ r12.l =  max(uAF[60],uAF[50]) - uAF[80], GE[1:0] set if no borrow, clear if borrow
        "sel     r11,r11,r7\n\t"             //@ r11.h = (max(uAF[61],uAF[51]) > uAF[81]) ? max(uAF[61],uAF[51]) : uAF[81]
                                             //@ r11.l = (max(uAF[60],uAF[50]) > uAF[80]) ? max(uAF[60],uAF[50]) : uAF[80]
//
//      r3.h &= min(uAF[91],uAF[71]) > max(uAF[81],uAF[61],uAF[51])
//      r3.l &= min(uAF[90],uAF[70]) > max(uAF[80],uAF[60],uAF[51])
//
        "usub16  r10,r11,r10\n\t"            //@ r3.h = max(uAF[81],uAF[61],uAF[51]) - min(uAF[91],uAF[71]), GE[3:2] set if no borrow, clear if borrow
                                             //@ r3.l = max(uAF[80],uAF[60],uAF[50]) - min(uAF[90],uAF[70]), GE[1:0] set if no borrow, clear if borrow
        "ands    r3, r3, r10\n\t"            //@ if both lanes are false / zero, then both lanes fail, so ...
        "beq     Exit\n\t"                   //@ ...return now
//
//      Add uAF[91-90] & uAF[71-70] to the signal strength
//
        "sxtah   r8, r8, r4\n\t"             //@ r8   = sigStrength1 += r4.l
        "sxtah   r9, r9, r4,ROR#16\n\t"      //@ r9   = sigStrength2 += r5.h
        "sxtah   r8, r8, r5\n\t"             //@ r8   = sigStrength1 += r4.l
        "sxtah   r9, r9, r5,ROR#16\n\t"      //@ r9   = sigStrength2 += r5.h
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//       # If we get here we have either one or two valid preamble decodes
//       #
//       # r0    = pPreamble
//       # r1    = pLane1
//       # r2    = pLane2
//       # r3.h  = 0x8000 / 0x0000,        r3.l = 0x8000 / 0x0000
//       # r4    = clobbered
//       # r5    = clobbered
//       # r6    = clobbered
//       # r7    = clobbered
//       # r8    = sigStrength1
//       # r9    = sigStrength2
//       # r10   = clobbered
//       # r11   = clobbered
//       # r12   = clobbered
//       # r14   = 0x80008000
//
        "add     r0, r0, #320\n\t"           //@ r0 points to pPayload
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do the bit 0 of the data frame. This is the MSB bit of the DF field, and determines the total frame length (56 or 112 bits)
// Don't allow any "zero" guesses in this field 
//
        "ldr     r4, [r0],#20\n\t"           //@ r4    = Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5    = Pick up two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h  = uAF[11] - uAF[1]
                                             //@ r7.l  = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h  = max(uAF[11],uAF[1])
                                             //@ r4.l  = max(uAF[10],uAF[0])
        "and     r12,r7, r14\n\t"            //@ r12.h = (uAF[ 1] > uAF[11]) ? 0x8000 : 0x0000
                                             //@ r12.l = (uAF[ 0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "biceq   r3, r3, #15\n\t"            //@     r3.l = r3.l & 0x7FFF;
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "biceqs  r3, r3, #31\n\t"            //@     r3.h = r3.h & 0x7FFF;
        "beq     Exit\n\t"                   //@ If both lanes are false / zero, then both lanes fail, so return now

        "uxtah   r8, r8, r4\n\t"             //@ r8    = sigStrength1 += r4.l
        "uxtah   r9, r9, r4, ROR#16\n\t"     //@ r9    = sigStrength2 += r4.h

        "ror     r12,r12,#31\n\t"            //@ r12   = ROL(r12,1)
//
// Do bits 1-4 of the DF field.
//
        //@ Allow One 'zero' count in the remainder of the DF Field
        "tst     r3, #0x00008000\n\t"        //@ if (Lane1 Valid)....
        "orrne   r3, r3, #0x000000FE\n\t"    //@ ...r3.l = 0xFFFE;
        "orrne   r3, r3, #0x0000FF00\n\t"    //@ ...r3.l = 0xFFFE;
        "tst     r3, #0x80000000\n\t"        //@ if (Lane2 Valid)....
        "orrne   r3, r3, #0x00FE0000\n\t"    //@ ...r3.h = 0xFFFE;
        "orrne   r3, r3, #0xFF000000\n\t"    //@ ...r3.h = 0xFFFE;

        "mov     r10, #4\n\t"                //@ r10    = loop count
    "DFLoop:\n\t"
        "ldr     r4, [r0],#20\n\t"           //@ r4     = Pick up the next two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5     = Pick up the next two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h   = uAF[11] - uAF[1]
                                             //@ r7.l   = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h   = max(uAF[11],uAF[1])
                                             //@ r4.l   = max(uAF[10],uAF[0])
        "and     r5, r7, r14\n\t"            //@ r12.h |= (uAF[1] > uAF[11]) ? 0x8000 : 0x0000
        "orr     r12,r12, r5\n\t"            //@ r12.l |= (uAF[0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "addeq   r3, r3, #0x00000001\n\t"    //@     r3.l = r3.l + 1
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "addeq   r3, r3, #0x00010000\n\t"    //@     r3.h = r3.h + 1

        "uxtah   r8, r8, r4\n\t"             //@ r8     = sigStrength1 += r4.l
        "uxtah   r9, r9, r4, ROR#16\n\t"     //@ r9     = sigStrength2 += r4.h

        "ror     r12,r12,#31\n\t"            //@ r12    = ROL(r12,1)

        "subs    r10,r10,#1\n\t"
        "bne     DFLoop\n\t"

        "ands    r10,r3, r14\n\t"
        "beq     Exit\n\t"                   //@ If more than one zero in DF field, exit
//
// Do bits 5-15 of the Payload.
//
        //@ Allow Three 'zero' counts in the entire frame
        "tst     r3, #0x00008000\n\t"        //@ if (Lane1 Valid)....
        "subne   r3,r3,#2\n\t"               //@ ....r3.l -= 2
        "tst     r3, #0x80000000\n\t"        //@ if (Lane1 Valid)....
        "subne   r3,r3,#0x20000\n\t"         //@ ....r3.h -= 2

        "mov     r10, #11\n\t"               //@ r10    = loop count
    "Word1Loop:\n\t"
        "ldr     r4, [r0],#20\n\t"           //@ r4     = Pick up the next two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5     = Pick up the next two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h   = uAF[11] - uAF[1]
                                             //@ r7.l   = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h   = max(uAF[11],uAF[1])
                                             //@ r4.l   = max(uAF[10],uAF[0])
        "and     r5, r7, r14\n\t"            //@ r12.h |= (uAF[1] > uAF[11]) ? 0x8000 : 0x0000
        "orr     r12,r12,r5\n\t"             //@ r12.l |= (uAF[0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "addeq   r3, r3, #0x00000001\n\t"    //@     r3.l = r3.l + 1
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "addeq   r3, r3, #0x00010000\n\t"    //@     r3.h = r3.h + 1

        "uxtah   r8, r8, r4\n\t"             //@ r8     = sigStrength1 += r4.l
        "uxtah   r9, r9, r4, ROR#16\n\t"     //@ r9     = sigStrength2 += r4.h

        "ror     r12,r12,#31\n\t"            //@ r12    = ROL(r12,1)

        "subs    r10,r10,#1\n\t"
        "bne     Word1Loop\n\t"

        "ands    r10,r3, r14\n\t"
        "beq     Exit\n\t"                   //@ If more than three errors in both frames, exit
                                             //@ r12.h = Lane1 b7-b0:b15-b8
//                                           //@ r12.l = Lane2 b7-b0:b15-b8
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 16-31 of the data frame.
//
        "mov     r6,  #0\n\t"                //@ r6   = 0
        "mov     r10, #16\n\t"               //@ r10  = loop count
    "Word2Loop:\n\t"
        "ldr     r4, [r0],#20\n\t"           //@ r4   = Pick up the next two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5   = Pick up the next two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h   = uAF[11] - uAF[1]
                                             //@ r7.l   = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h  = max(uAF[11],uAF[1])
                                             //@ r4.l  = max(uAF[10],uAF[0])
        "and     r5, r7, r14\n\t"            //@ r6.h |= (uAF[1] > uAF[11]) ? 0x8000 : 0x0000
        "orr     r6, r6, r5\n\t"             //@ r6.l |= (uAF[0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "addeq   r3, r3, #0x00000001\n\t"    //@     r3.l = r3.l + 1
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "addeq   r3, r3, #0x00010000\n\t"    //@     r3.h = r3.h + 1

        "uxtah   r8, r8, r4\n\t"             //@ r8   = sigStrength1 += r4.l
        "uxtah   r9, r9, r4, ROR#16\n\t"     //@ r9   = sigStrength2 += r4.h

        "ror     r6, r6, #31\n\t"            //@ r6    = ROL(r6,1)

        "subs    r10,r10,#1\n\t"
        "bne     Word2Loop\n\t"

        "ands    r10,r3, r14\n\t"
        "beq     Exit\n\t"                   //@ If more than three errors in both frames, exit
                                             //@ r6.h = Lane1 b23-b16:b31-b24
//                                           //@ r6.l = Lane2 b23-b16:b31-b24
//----------------------------------------------------------------------------------------------------------------------------------------
//
// We've got the first 32 bits for at least one frame
//
        "pkhtb   r4, r12,r6,ASR#16\n\t"      //@ r4.h  = r12.h  Lane1 b7-b0   : b15-b8
                                             //@ r4.l  = r6.h   Lane1 b23-b16 : b31-b24
                                             //@ r4    =        Lane1 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r4, r4\n\t"                 //@ r4    =        Lane1 b31-b24 : b23-b16 : b15-b8  : b7-b0
        "tst     r3, #0x00008000\n\t"        //@ if (Lane1 Valid)....
        "strne   r4, [r1,#8]\n\t"            //@ ....Store Lane1Word

        "pkhbt   r5, r6, r12,LSL#16\n\t"     //@ r5.h  = r12.l  Lane2 b7-b0   : b15-b8
                                             //@ r5.l  = r6.l   Lane2 b23-b16 : b31-b24
                                             //@ r5    =        Lane2 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r5, r5\n\t"                 //@ r5    =        Lane2 b31-b24 : b23-b16 : b15-b8  : b7-b0
        "tst     r3, #0x80000000\n\t"        //@ if (Lane2 Valid)....
        "strne   r5, [r2,#8]\n\t"            //@ ....Store Lane2Word
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 32-47 of the data frame.
//
        "mov     r6,  #0\n\t"                //@ r6    = 0
        "mov     r10, #16\n\t"               //@ r10   = loop count
    "Word3Loop:\n\t"
        "ldr     r4, [r0],#20\n\t"           //@ r4    = Pick up the next two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5    = Pick up the next two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h   = uAF[11] - uAF[1]
                                             //@ r7.l   = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h  = max(uAF[11],uAF[1])
                                             //@ r4.l  = max(uAF[10],uAF[0])
        "and     r5, r7, r14\n\t"            //@ r6.h |= (uAF[1] > uAF[11]) ? 0x8000 : 0x0000
        "orr     r6, r6, r5\n\t"             //@ r6.l |= (uAF[0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "addeq   r3, r3, #0x00000001\n\t"    //@     r3.l = r3.l + 1
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "addeq   r3, r3, #0x00010000\n\t"    //@     r3.h = r3.h + 1

        "uxtah   r8, r8, r4\n\t"             //@ r8    = sigStrength1 += r4.l
        "uxtah   r9, r9, r4, ROR#16\n\t"     //@ r9    = sigStrength2 += r4.h

        "ror     r6, r6, #31\n\t"            //@ r6    = ROL(r6,1)

        "subs    r10,r10,#1\n\t"
        "bne     Word3Loop\n\t"

        "ands    r10,r3, r14\n\t"
        "beq     Exit\n\t"                   //@ If more than three errors in both frames, exit
                                             //@ r6.h = Lane1 b39-b32:b47-b40
//                                           //@ r6.l = Lane2 b39-b32:b47-b40
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 48-55 of the data frame.
//
        "mov     r11, #0\n\t"                //@ r11    = 0
        "mov     r10, #8\n\t"                //@ r10    = loop count
    "Word4Loop:\n\t"
        "ldr     r4, [r0],#20\n\t"           //@ r4     = Pick up the next two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5     = Pick up the next two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h   = uAF[11] - uAF[1]
                                             //@ r7.l   = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h  = max(uAF[11],uAF[1])
                                             //@ r4.l  = max(uAF[10],uAF[0])
        "and     r5, r7, r14\n\t"            //@ r11.h |= (uAF[1] > uAF[11]) ? 0x8000 : 0x0000
        "orr     r11,r11,r5\n\t"             //@ r11.l |= (uAF[0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "addeq   r3, r3, #0x00000001\n\t"    //@     r3.l = r3.l + 1
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "addeq   r3, r3, #0x00010000\n\t"    //@     r3.h = r3.h + 1

        "uxtah   r8, r8, r4\n\t"             //@ r8   = sigStrength1 += r4.l
        "uxtah   r9, r9, r4, ROR#16\n\t"     //@ r9   = sigStrength2 += r4.h

        "ror     r11,r11, #31\n\t"           //@ r11   = ROL(r11,1)

        "subs    r10,r10, #1\n\t"
        "bne     Word4Loop\n\t"

        "ands    r10,r3, r14\n\t"
        "beq     Exit\n\t"                   //@ If more than three errors in both frames, exit
                                             //@ r11.h = Lane1 0000:b55-b48
                                             //@ r11.l = Lane2 0000:b55-b48
        "ands    r10,r10,r12,ROR#16\n\t"
        "bne     Word4LoopA\n\t"             //@ If one or both frames are valid and long, goto Word4LoopA
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// One or Both valid frames are short and we've got bits 32-56 for them
//
        "ror     r10,r11,#24\n\t"            //@ r10 = ROL(r11,8)

        "pkhtb   r4, r6, r10,ASR#16\n\t"     //@ r4.h  = r6.h   Lane1 b7-b0   : b15-b8
                                             //@ r4.l  = r11.h  Lane1 b23-b16 : b31-b24
        "rev     r4, r4\n\t"                 //@ r4    =        Lane1 b31-b24 : b23-b16 : b15-b8  : b7-b0

        "pkhbt   r5, r10, r6,LSL#16\n\t"     //@ r5.h  = r6.l   Lane2 b7-b0   : b15-b8
                                             //@ r5.l  = r11.l  Lane2 b23-b16 : b31-b24
        "rev     r5, r5\n\t"                 //@ r5    =        Lane2 b31-b24 : b23-b16 : b15-b8  : b7-b0

        "mov     r10, #56\n\t"               //@ r10 = Bit Count
 
        "tst     r3, #0x00008000\n\t"        //@ if (Lane1 Valid)....
        "strne   r4, [r1, #12]\n\t"          //@ ....Store Lane1Word
        "stmne   r1, { r8, r10 }\n\t"        //@ ....Store Lane1sigStrength and Bit Count

        "tst     r3, #0x80000000\n\t"        //@ if (Lane2 Valid)....
        "strne   r5, [r2, #12]\n\t"          //@ ....Store Lane2Word
        "stmne   r2, { r9, r10 }\n\t"        //@ ....Store Lane2sigStrength and Bit Count

        "and     r0, r3, r14\n\t" 
        "b       Done\n\t" 
//      "pop    { r4 - r12, pc }\n\t"        //@ Pop the registers that need to be preserved from the stackand return
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// One or both frames are Long, and is/are valid so far...
// However, we could have (unlikely I know) one long and one short valid frames
//
    "Word4LoopA:\n\t" 
        "tst     r3,  #0x00008000\n\t"       //@ if (Lane1 not Valid)....
        "beq     Word4LoopD\n\t"             //@ ...goto Word4LoopD (Lane2 must be valid and long)
        "tst     r12, #0x80000000\n\t"       //@ if (Lane1 is Long)
        "bne     Word4LoopC\n\t"             //@ ...goto Word4LoopC

        "ror     r10,r11,#24\n\t"            //@ r10 = ROL(r11,8)

        "pkhtb   r4, r6, r10,ASR#16\n\t"     //@ r4.h  = r6.h   Lane1 b7-b0   : b15-b8
                                             //@ r4.l  = r11.h  Lane1 b23-b16 : b31-b24
                                             //@ r4    =        Lane1 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r4, r4\n\t"                 //@ r4    =        Lane1 b31-b24 : b23-b16 : b15-b8  : b7-b0

        "mov     r10, #56\n\t"               //@ r10 = Bit Count
        "str     r4, [r1, #12]\n\t"          //@ ....Store Lane1Word
        "stm     r1, { r8, r10 }\n\t"        //@ ....Store Lane1sigStrength and Bit Count

        "eor     r3, r3, #0x00007000\n\t"    //@ ...r3.l -= 0x7000 
        "beq     Word4LoopD\n\t"             //@ ...goto Word4LoopD (Lane2 must be valid and long since Lane1 was valid and short)

    "Word4LoopC:\n\t"
        "tst     r3,  #0x80000000\n\t"       //@ if (Lane2 not Valid)....
        "beq     Word4LoopD\n\t"             //@ ...goto Word4LoopD
        "tst     r12, #0x00008000\n\t"       //@ if (Lane2 is Long).....
        "bne     Word4LoopD\n\t"             //@ ...goto Word4LoopD (Lane2 is valid and long)

        "ror     r10,r11,#24\n\t"            //@ r10 = ROL(r11,8)

        "pkhbt   r5, r10, r6,LSL#16\n\t"     //@ r5.h  = r6.l   Lane2 b7-b0   : b15-b8
                                             //@ r5.l  = r11.l  Lane2 b23-b16 : b31-b24
                                             //@ r5    =        Lane2 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r5, r5\n\t"                 //@ r5    =        Lane2 b31-b24 : b23-b16 : b15-b8  : b7-b0
 
        "mov     r10, #56\n\t"               //@ r10 = Bit Count
        "str     r5, [r2, #12]\n\t"          //@ ....Store Lane2Word
        "stm     r2, { r9, r10 }\n\t"        //@ ....Store Lane2sigStrength and Bit Count

        "eor     r3, r3, #0x70000000\n\t"    //@ ...r3.l -= 0x70000000 
    "Word4LoopD:\n\t"
//
// Do bits 56-63 of the data frame.
//
        "mov     r10, #8\n\t"                //@ r10     = loop count
    "Word4LoopB:\n\t"
        "ldr     r4, [r0],#20\n\t"           //@ r4     = Pick up the next two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5     = Pick up the next two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h   = uAF[11] - uAF[1]
                                             //@ r7.l   = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h  = max(uAF[11],uAF[1])
                                             //@ r4.l  = max(uAF[10],uAF[0])
        "and     r5, r7, r14\n\t"            //@ r11.h |= (uAF[1] > uAF[11]) ? 0x8000 : 0x0000
        "orr     r11,r11,r5\n\t"             //@ r11.l |= (uAF[0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "addeq   r3, r3, #0x00000001\n\t"    //@     r3.l = r3.l + 1
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "addeq   r3, r3, #0x00010000\n\t"    //@     r3.h = r3.h + 1

        "uxtah   r8,r8,r4\n\t"               //@ r8   = sigStrength1 += r4.l
        "uxtah   r9,r9,r4,ROR#16\n\t"        //@ r9   = sigStrength2 += r4.h

        "ror     r11,r11, #31\n\t"           //@ r11   = ROL(r11,1)

        "subs    r10,r10,#1\n\t"
        "bne     Word4LoopB\n\t"

        "ands    r10,r3, r14\n\t"
        "beq     Exit\n\t"                   //@ If more than three errors in both frames, exit
                                             //@ r11.h = Lane1 b55-b48:b63-b56
//                                           //@ r11.l = Lane2 b55-b48:b63-b56
//----------------------------------------------------------------------------------------------------------------------------------------
//
// We've got bits 32-63 for at least one valid frame
//
        "pkhtb   r4, r6,r11,ASR#16\n\t"      //@ r4.h  = r11.h  Lane1 b7-b0   : b15-b8
                                             //@ r4.l  = r6.h   Lane1 b23-b16 : b31-b24
                                             //@ r4    =        Lane1 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r4, r4\n\t"                 //@ r4    =        Lane1 b31-b24 : b23-b16 : b15-b8  : b7-b0
        "tst     r3, #0x00004000\n\t"        //@ if (Lane1 Valid)....
        "strne   r4, [r1, #12]\n\t"          //@ ....Store Lane1Word

        "pkhbt   r5, r11, r6,LSL#16\n\t"     //@ r5.h  = r11.l  Lane2 b7-b0   : b15-b8
                                             //@ r5.l  = r6.l   Lane2 b23-b16 : b31-b24
                                             //@ r5    =        Lane2 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r5, r5\n\t"                 //@ r5    =        Lane2 b31-b24 : b23-b16 : b15-b8  : b7-b0
        "tst     r3, #0x40000000\n\t"        //@ if (Lane2 Valid)....
        "strne   r5, [r2, #12]\n\t"          //@ ....Store Lane2Word
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 64-79 of the data frame.
//
        "mov     r6,  #0\n\t"                //@ r6    = 0
        "mov     r10, #16\n\t"               //@ r10   = loop count
    "Word5Loop:\n\t"
        "ldr     r4, [r0],#20\n\t"           //@ r4   = Pick up the next two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5   = Pick up the next two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h   = uAF[11] - uAF[1]
                                             //@ r7.l   = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h  = max(uAF[11],uAF[1])
                                             //@ r4.l  = max(uAF[10],uAF[0])
        "and     r5, r7, r14\n\t"            //@ r6.h |= (uAF[1] > uAF[11]) ? 0x8000 : 0x0000
        "orr     r6, r6, r5\n\t"             //@ r6.l |= (uAF[0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "addeq   r3, r3, #0x00000001\n\t"    //@     r3.l = r3.l + 1
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "addeq   r3, r3, #0x00010000\n\t"    //@     r3.h = r3.h + 1

        "uxtah   r8, r8, r4\n\t"             //@ r8   = sigStrength1 += r4.l
        "uxtah   r9, r9, r4, ROR#16\n\t"     //@ r9   = sigStrength2 += r4.h

        "ror     r6, r6, #31\n\t"            //@ r6   = ROL(r6,1)

        "subs    r10,r10,#1\n\t"
        "bne     Word5Loop\n\t"

        "ands    r10,r3, r14\n\t"
        "beq     Exit\n\t"                   //@ If more than three errors in both frames, exit
                                             //@ r6.h = Lane1 b71-b64:b79-b72
//                                           //@ r6.l = Lane2 b71-b64:b79-b72
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 80-95 of the data frame.
//
        "mov     r11, #0\n\t"                //@ r11    = 0
        "mov     r10, #16\n\t"               //@ r10    = loop count
    "Word6Loop:\n\t"
        "ldr     r4, [r0],#20\n\t"           //@ r4     = Pick up the next two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5     = Pick up the next two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h   = uAF[11] - uAF[1]
                                             //@ r7.l   = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h  = max(uAF[11],uAF[1])
                                             //@ r4.l  = max(uAF[10],uAF[0])
        "and     r5, r7, r14\n\t"            //@ r11.h |= (uAF[1] > uAF[11]) ? 0x8000 : 0x0000
        "orr     r11,r11,r5\n\t"             //@ r11.l |= (uAF[0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "addeq   r3, r3, #0x00000001\n\t"    //@     r3.l = r3.l + 1
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "addeq   r3, r3, #0x00010000\n\t"    //@     r3.h = r3.h + 1

        "uxtah   r8, r8, r4\n\t"             //@ r8     = sigStrength1 += r4.l
        "uxtah   r9, r9, r4, ROR#16\n\t"     //@ r9     = sigStrength2 += r4.h

        "ror     r11,r11, #31\n\t"           //@ r11    = ROL(r11,1)

        "subs    r10,r10,#1\n\t"
        "bne     Word6Loop\n\t"

        "ands    r10,r3, r14\n\t"
        "beq     Exit\n\t"                   //@ If more than three errors in both frames, exit
                                             //@ r11.h = Lane1 b87-b80:b95-b88
//                                           //@ r11.l = Lane2 b87-b80:b95-b88
//----------------------------------------------------------------------------------------------------------------------------------------
//
// We've got bits 64-95 for at least one valid frame
//
        "pkhtb   r4, r6,r11,ASR#16\n\t"      //@ r4.h  = r11.h  Lane1 b7-b0   : b15-b8
                                             //@ r4.l  = r6.h   Lane1 b23-b16 : b31-b24
                                             //@ r4    =        Lane1 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r4, r4\n\t"                 //@ r4    =        Lane1 b31-b24 : b23-b16 : b15-b8  : b7-b0
        "tst     r3, #0x00004000\n\t"        //@ if (Lane1 Valid)....
        "strne   r4, [r1,#16]\n\t"           //@ ....Store Lane1Word

        "pkhbt   r5, r11, r6,LSL#16\n\t"     //@ r5.h  = r11.l  Lane2 b7-b0   : b15-b8
                                             //@ r5.l  = r6.l   Lane2 b23-b16 : b31-b24
                                             //@ r5    =        Lane2 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r5, r5\n\t"                 //@ r5    =        Lane2 b31-b24 : b23-b16 : b15-b8  : b7-b0
        "tst     r3, #0x40000000\n\t"        //@ if (Lane2 Valid)....
        "strne   r5, [r2,#16]\n\t"           //@ ....Store Lane2Word
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Do bits 96-111 of the data frame.
//
        "mov     r6,  #0\n\t"                //@ r6    = 0
        "mov     r10, #16\n\t"               //@ r10   = loop count
    "Word7Loop:\n\t"
        "ldr     r4, [r0],#20\n\t"           //@ r4   = Pick up the next two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldr     r5, [r0],#20\n\t"           //@ r5   = Pick up the next two uAF input values r5.h = uAF[11], r5.l = uAF[10]
        "usub16  r7, r5, r4\n\t"             //@ r7.h   = uAF[11] - uAF[1]
                                             //@ r7.l   = uAF[10] - uAF[0]
        "sel     r4, r5, r4\n\t"             //@ r4.h  = max(uAF[11],uAF[1])
                                             //@ r4.l  = max(uAF[10],uAF[0])
        "and     r5, r7, r14\n\t"            //@ r6.h |= (uAF[1] > uAF[11]) ? 0x8000 : 0x0000
        "orr     r6, r6, r5\n\t"             //@ r6.l |= (uAF[0] > uAF[10]) ? 0x8000 : 0x0000

        "lsls    r5, r7, #16\n\t"            //@ if (0 == (uAF[10] - uAF[0]))
        "addeq   r3, r3, #0x00000001\n\t"    //@     r3.l = r3.l + 1
        "lsrs    r5, r7, #16\n\t"            //@ if (0 == (uAF[11] - uAF[1]))
        "addeq   r3, r3, #0x00010000\n\t"    //@     r3.h = r3.h + 1

        "uxtah   r8, r8, r4\n\t"             //@ r8   = sigStrength1 += r4.l
        "uxtah   r9, r9, r4, ROR#16\n\t"     //@ r9   = sigStrength2 += r4.h

        "ror     r6, r6, #31\n\t"            //@ r6   = ROL(r6,1)

        "subs    r10,r10,#1\n\t"
        "bne     Word7Loop\n\t"

        "ands    r10,r3, r14\n\t"
        "beq     Exit\n\t"                   //@ If more than three errors in both frames, exit
                                             //@ r6.h = Lane1 0000:b111-b96
//                                           //@ r6.l = Lane2 0000:b111-b96
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// We've got bits 96-111 for at least one valid frame
//
        "mov     r11, #0\n\t"                //@ r11   = 0
        "pkhtb   r4, r6,r11,ASR#16\n\t"      //@ r4.h  = r11.h  Lane1 b7-b0   : b15-b8
                                             //@ r4.l  = r6.h   Lane1 b23-b16 : b31-b24
                                             //@ r4    =        Lane1 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r4, r4\n\t"                 //@ r4    =        Lane1 b31-b24 : b23-b16 : b15-b8  : b7-b0

        "mov     r10, #112\n\t"              //@ r10 = Bit Count
        "tst     r3, #0x00004000\n\t"        //@ if (Lane1 Valid)....
        "strne   r4, [r1, #20]\n\t"          //@ ....Store Lane1Word
        "stmne   r1, { r8, r10 }\n\t"        //@ ....Store Lane1sigStrength and Bit Count

        "pkhbt   r5, r11, r6, LSL#16\n\t"    //@ r5.h  = r11.l  Lane2 b7-b0   : b15-b8
                                             //@ r5.l  = r6.l   Lane2 b23-b16 : b31-b24
                                             //@ r5    =        Lane2 b7-b0   : b15-b8  : b23-b16 : b31-b24
        "rev     r5, r5\n\t"                 //@ r5    =        Lane2 b31-b24 : b23-b16 : b15-b8  : b7-b0
        "tst     r3, #0x40000000\n\t"        //@ if (Lane2 Valid)....
        "strne   r5, [r2, #20]\n\t"          //@ ....Store Lane2Word
        "stmne   r2, { r9, r10 }\n\t"        //@ ....Store Lane2sigStrength and Bit Count

        "and     r3, r3, r14\n\t"
        "mov     r0, r3\n\t"
        "b       Done\n\t" 
//      "pop    { r4 - r12, pc }\n\t"         //@ Pop the registers that need to be preserved from the stackand return
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
    "Exit:\n\t" 
        "mov     r3, #0\n\t" 
        "mov     r0, #0\n\t" 

    "Done:\n\t" 
        "pop    { r4 - r12, lr }\n\t"         //@ Pop the registers that need to be preserved from the stack (no return for inline asm)
//	    "pop    { r4 - r12, pc }\n\t"         //@ Pop the registers that need to be preserved from the stack and return
    "Quit:\n\t" 

      : "=r" (puAF) // Outputs in r0
      : "g" (puAF), "g" (pLane)
      : "r0","r1","r2","r3","cc","memory"
	);
}

static tMessageRaw rmLane[2];

static void detectBufferModeS(uint16_t* puAF, uint64_t lTime) {

	uint32_t j, uLanes;
	tMessageRaw** pQ;

	memset(&rmLane, 0, sizeof(tMessageRaw) * 2);

	// Find the end of any current queue
	pQ = &Modes.pModeS_List;
    while (*pQ) {
		pQ = & (*pQ)->pNext;
	}

	for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j += 2) {

		if ((uLanes = detectModeSArmSIMD(puAF, rmLane))) {

			if ((uLanes & 0x00008000) && (validateModeS(&rmLane[0], Modes.icao_cache, lTime)))
				{pQ = enQueueModeS(&rmLane[0], pQ);}

			if ((uLanes & 0x80000000) && (validateModeS(&rmLane[1], Modes.icao_cache, lTime+1)))
				{pQ = enQueueModeS(&rmLane[1], pQ);}

        } else if (Modes.uModeSTimeout > 2){
            Modes.uModeSTimeout -= 2;

		} else if (Modes.uModeSTimeout) {
            Modes.uModeSTimeout = 0;
            pQ = postDecodeModeS();
        }

		lTime+=2;
		puAF += 2;
	}
}

#else

uint32_t detectModeS(uint16_t* puAF, tMessageRaw* pLane) {

    uint16_t* pPayload;
    unsigned char* pMsg;
    int i, nBitGuess;
    uint8_t  theByte, dfErrBits;
    int nMsgLen, sigStrength;
    uint16_t a, b;

    // First check relations between the first 10 samples
    // representing a valid preamble. We don't even investigate further
    // if this simple test is not passed
    if (!(
        (puAF[0] > puAF[10]) &&
        (puAF[0] > puAF[30]) &&
        (puAF[0] > puAF[40]) &&
        (puAF[20] > puAF[10]) &&
        (puAF[20] > puAF[30]) &&
        (puAF[20] > puAF[40]) &&
        (puAF[90] > puAF[50]) &&
        (puAF[90] > puAF[60]) &&
        (puAF[90] > puAF[80]) &&
        //  (puAF[90] > puAF[100]) &&
        (puAF[70] > puAF[50]) &&
        (puAF[70] > puAF[60]) &&
        (puAF[70] > puAF[80]) &&
        ((a = puAF[160]) != (b = puAF[170]))
        )) {
        return (0);
    }

    // We should have 4 'bits' of 0/1 and 1/0 samples in the preamble, 
    // so include these in the signal strength 
    sigStrength = puAF[0] + puAF[20] + puAF[70] + puAF[90];

    // We've already got the first bit in a&b, and this bit determines message length.
    if (a > b) {
        sigStrength += a;
        pLane->uMsgBits = nMsgLen = MODES_LONG_MSG_BITS;
        theByte = 2;
    }
    else {
        sigStrength += b;
        pLane->uMsgBits = nMsgLen = MODES_SHORT_MSG_BITS;
        theByte = 0;
    }

    pPayload = &puAF[MODES_PREAMBLE_SAMPLES + 20];
    pMsg = pLane->ucMsg;
    dfErrBits = 0;
    nBitGuess = 0;

    for (i = 1; i < nMsgLen; i++) {
        a = pPayload[0];
        b = pPayload[10];
        pPayload += 20;

        sigStrength += ((a > b) ? a : b);

        if (a > b) {
            theByte |= 1;
        } else if (a == b) {
            dfErrBits |= 1;
            // If we've exceeded the permissible number of encoding errors, abandon ship now
            if (++nBitGuess > MODES_MSG_ENCODER_ERRS) {
                return (0);
            }
        }

        if ((i & 7) == 7) {
            *pMsg++ = theByte;
        }

        else if (i == 4) { // We can check the DF type validity here
            //
            // If we guessed at any of the bits in the DF type field, then look to see if our guess was sensible.
            // Do this by looking to see if the original guess results in the DF type being one of the ICAO defined
            // message types. If it isn't then toggle the guessed bit and see if this new value is ICAO defined.
            // if the new value is ICAO defined, then update it in our message.
            //
            if (nBitGuess > 1) { // More than one guess in the DF field, so abandon ship now
                return (0);
            } else if (nBitGuess) { // One and only one guess in the DF field...
             // See if our guess is likely to be correct by comparing the DF against a list of known good DF's
                uint32_t thisDFbit = (1 << theByte);
                if (0 == (kDFValid & thisDFbit)) {
                    // The current DF is not ICAO defined, so is probably an error. 
                    // Toggle the bit we guessed at and see if the resultant DF is any more likely
                    theByte ^= dfErrBits;
                    thisDFbit = (1 << theByte);
                    // Is this DF any more likely?
                    if (0 == (kDFValid & thisDFbit)) {
                        // Nope, this DF isn't valid either. Abort now.
                        //theByte ^= dfErrBits;
                        return (0);
                    }
                }
            }
        }
        theByte = theByte << 1;
        dfErrBits = dfErrBits << 1;
    }
    pLane->signalLevel32 = sigStrength;
    return (1);
}

static tMessageRaw rmLane;

static void detectBufferModeS(uint16_t* puAF, uint64_t lTime) {

	uint32_t j;
	tMessageRaw** pQ;

	memset(&rmLane, 0, sizeof(tMessageRaw));

	// Find the end of any current queue
	pQ = &Modes.pModeS_List;
    while (*pQ) {
		pQ = & (*pQ)->pNext;
	}

	for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j++) {
/*
        if (lTime == 0x23581A26) {
//      if (lTime == 0x0217945F) {
            __asm nop;
        }
*/
		if ( (detectModeS(&puAF[j], &rmLane))
		  && (validateModeS(&rmLane, Modes.icao_cache, lTime)))
			{pQ = enQueueModeS(&rmLane, pQ);}
        else if ((Modes.uModeSTimeout) && (!(--Modes.uModeSTimeout)))
            {pQ = postDecodeModeS();}

		lTime++;
	}
}

#endif
//
//=========================================================================
//
// This thread takes output buffers from IF_threadproc and feeds them them 
// into buffers used by the Mode A/C/S detection code.
//
// Modes.pAF_Fifo[Modes.uAF_Out] is the next input buffer to process
// Modes.pAF_Fifo[Modes.uAF_In] is the newest input buffer added to the FIFO
//
// The number of buffers in the Fifo is (Modes.uAF_In - Modes.uAF_Out)
//
void* ModeS_threadproc(void* arg) {

	uint32_t uAF_Out;
    UNUSED (arg);

#ifdef _WIN32
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif

	// Keep going until USB or program terminates
	while (!Modes.exit) {

		// While there is no data to process, release AF_mutex and wait for AF_cond. 
		// When AF_cond occurs, re-aquire the AF_mutex.
		while ((0 == (Modes.uAF_Out - Modes.uAF_In)) && (!Modes.exit)) {
			pthread_cond_wait(&Modes.AF_cond, &Modes.AF_mutex);
		}
		// We own the AF_mutex at this point

		// If the program is terminating break
		if (Modes.exit) {
			break;
		}

		// Get the next buffer number to process
		uAF_Out = Modes.uAF_Out & (MODES_ASYNC_BUF_NUMBER - 1);
		// Don't change uAF_Out yet - we haven't processed the data

    	// Safe to release the AF_mutex now that we've got Modes.uAF_Out
	    pthread_mutex_unlock(&Modes.AF_mutex);

        if (Modes.mode_ac) {
            //
            // If ModeA is enabled kick the ModeA_cond to get the ModeA 
            // thread to process it.
            //
	    	pthread_mutex_lock(&Modes.ModeA_Go_mutex);
            // Kick the ModeA condex to process the buffer
            pthread_cond_signal(&Modes.ModeA_Go_cond);
    		// Safe to release the mutex now that we've set ModeA_Go_cond
	    	pthread_mutex_unlock(&Modes.ModeA_Go_mutex);

        } else {

            //
            // If ModeA is not running, send a null ModeA packet to the decoder thread 
            // with the time of the end of this (ModeS) processing. This keeps modeS
            // decoding ticking along by not having to wait for (non existant) ModeA's
            //

            // Lock the Decode mutex
            pthread_mutex_lock(&Modes.Decode_mutex);

            // If there is some space in the ModeA_Fifo
            if ((Modes.uDecode_ModeA_In - Modes.uDecode_ModeA_Out) < MODES_DECODE_FIFO_NUMBER) {
                uint32_t uDecode_In = Modes.uDecode_ModeA_In & (MODES_DECODE_FIFO_NUMBER - 1);

                tMessageRaw* qm = Modes.pModeA_Buffer[Modes.uModeA_Buffer & (MODES_DECODE_BUF_NUMBER - 1)];
                Modes.uModeA_Buffer++;
                memset(qm, 0, sizeof(tMessageRaw));
                qm->lTime = Modes.AF_Fifo[uAF_Out].lTime + (MODES_ASYNC_BUF_SAMPLES - 1);

                Modes.Decode_ModeA_Fifo[uDecode_In] = qm;
                Modes.uDecode_ModeA_In++;
            }

            // Signal the Decode thread there is something to do.
            pthread_cond_signal(&Modes.Decode_cond);

            // Unlock the Decode mutex
            pthread_mutex_unlock(&Modes.Decode_mutex);
        }

        // Process the new buffer for ModeS
		detectBufferModeS(Modes.AF_Fifo[uAF_Out].pFifo, Modes.AF_Fifo[uAF_Out].lTime);

        //
        // If there is no leftover from this buffer into the next buffer, send a null ModeS
        // packet to the decoder thread with the time of the end of this (ModeS) processing.
        //
        if (Modes.pModeS_List == NULL) {
            // Lock the Decode mutex
            pthread_mutex_lock(&Modes.Decode_mutex);

            // If there is some space in the ModeS_Fifo
            if ((Modes.uDecode_ModeS_In - Modes.uDecode_ModeS_Out) < MODES_DECODE_FIFO_NUMBER) {
                uint32_t uDecode_In = Modes.uDecode_ModeS_In & (MODES_DECODE_FIFO_NUMBER - 1);

                tMessageRaw* qm = Modes.pModeS_Buffer[Modes.uModeS_Buffer & (MODES_DECODE_BUF_NUMBER - 1)];
                Modes.uModeS_Buffer++;
                memset(qm, 0, sizeof(tMessageRaw));
                qm->lTime = Modes.AF_Fifo[uAF_Out].lTime + (MODES_ASYNC_BUF_SAMPLES - 1);

                Modes.Decode_ModeS_Fifo[uDecode_In] = qm;
                Modes.uDecode_ModeS_In++;
            }
            // Signal the Decode thread there is something to do.
            pthread_cond_signal(&Modes.Decode_cond);

            // Unlock the Decode mutex
            pthread_mutex_unlock(&Modes.Decode_mutex);
        }

        if (Modes.mode_ac) {
            // Wait for ModeA processing to complete
			pthread_cond_wait(&Modes.ModeA_Done_cond, &Modes.ModeA_Done_mutex);

    		// If the program is terminating break
	    	if (Modes.exit) {
		    	break;
		    }
        }

		// re-aqcquire the AF mutex so we can update Modes.uAF_Out
		pthread_mutex_lock(&Modes.AF_mutex);
//		if (Modes.AF_Fifo[uAF_Out].uLost) {
//			printf("%d,Dropped %d\n", Modes.uAF_Out, Modes.AF_Fifo[uAF_Out].uLost);
//		}
		Modes.uAF_Out++;
	}

	// Unlock AF mutex
	pthread_mutex_unlock(&Modes.AF_mutex);

  	// Unlock ModeA_Done_mutex
    pthread_mutex_unlock(&Modes.ModeA_Done_mutex);

#ifdef _WIN32
	return (NULL);
#else
	pthread_exit(NULL);
#endif
}
//
//=========================================================================
//
