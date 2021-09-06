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
static tMessageRaw** enQueueModeA(tMessageRaw* rm, tMessageRaw* *pQ) {

	tMessageRaw* qm = Modes.pModeA_Buffer[Modes.uModeA_Buffer & (MODES_DECODE_BUF_NUMBER - 1)];
	Modes.uModeA_Buffer++;
	memmove(qm, rm, sizeof(tMessageRaw));
    qm->pNext = NULL; // Belt and braces

	*pQ = qm; // Add this item to the end of the current queue.

    // The ModeA bits should be 0.45uS (9 samples) wide. However, except for very strong signals, 
    // it's unlikely we'll see all 9 samples. Therefore if we assume we'll at least see the 
    // strongest sample in the middle of the F1 bit, the end of the F1 bit should be 4 samples later. 
    // Therefore set the timeout to 5. This will allow us to see most interleaved ModeA's from
    // closely spaced targets.
    Modes.uModeATimeout = 5;

	return (&qm->pNext); // return the address of the pNext element of this qm.
}
//
//=========================================================================
//
static tMessageRaw** postModeAQueue(void) {

   	// Put the latest decodes in the Decode Thread ModeA FIFO

    // Lock the Decode mutex
    pthread_mutex_lock(&Modes.Decode_mutex);

   	// If there is some space in the ModeA_Fifo
    if ((Modes.uDecode_ModeA_In - Modes.uDecode_ModeA_Out) < MODES_DECODE_FIFO_NUMBER) {
	    uint32_t uDecode_In = Modes.uDecode_ModeA_In & (MODES_DECODE_FIFO_NUMBER - 1);
   		Modes.Decode_ModeA_Fifo[uDecode_In] = Modes.pModeA_List;
    	Modes.uDecode_ModeA_In++;
  	}
    // Signal the Decode thread there is something to do.
    pthread_cond_signal(&Modes.Decode_cond);

   	// UnLock the Decode mutex
    pthread_mutex_unlock(&Modes.Decode_mutex);

   	// Start a new ModeA list
    Modes.pModeA_List   = NULL;

    return (&Modes.pModeA_List);
}
//
//=========================================================================
//
// https://www.radartutorial.eu/13.ssr/sr15.en.html
//
// NEWEST                                                                                                                    OLDEST
//                                  SP       F2,D4,B4,D2,B2,D1,B1,Z7,A4,C4,A2,C2,A1,C1,F1
// 00,01,02,03,04,05,06,07,08,09,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41   DL1 BIT INDEX
//                                           00,01,02,03,04,05,06,07,08,09,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27   DL2 BIT INDEX
//                                                                                     00,01,02,03,04,05,06,07,08,09,10,11,12,13   DL4 BIT INDEX
// |-------------------DL1-------------------|-|----------------DL2--------------------|-|-----------------DL4-----------------|
// 41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,09,08,07,06,05,04,03,02,01,00   puAF SAMPLE INDEX
//  1  1  1  1  1  1  1  9  9  9  8  8  8  8  7  7  7  6  6  6  6  5  5  5  4  4  4  4  3  3  3  2  2  2  2  1  1  1  8  5  2  0
//  1  1  1  1  0  0  0  8  5  2  9  7  4  1  8  5  2  9  6  3  0  8  5  2  9  6  3  0  7  4  1  9  6  3  0  7  4  1  7  8  9      puAF WORD OFFSET
//  8  6  3  0  7  4  1  6  7  8  9  0  1  2  3  4  5  6  7  8  9  0  1  2  3  4  5  6  7  8  9  0  1  2  3  4  5  6
//  9  0  1  2  3  4  5
//
// DL3[ 1] = (DL1[ 1] & DL1[15]) | (DL2[ 1] & DL2[15]);        = DL2[ 1] & (DL1[ 1] | DL4[ 1])
// DL3[ 2] = (DL1[ 2] & DL1[16]) | (DL2[ 2] & DL2[16]);        = DL2[ 2] & (DL1[ 2] | DL4[ 2])
// DL3[ 3] = (DL1[ 3] & DL1[17]) | (DL2[ 3] & DL2[17]);        = DL2[ 3] & (DL1[ 3] | DL4[ 3])
// DL3[ 4] = (DL1[ 4] & DL1[18]) | (DL2[ 4] & DL2[18]);        = DL2[ 4] & (DL1[ 4] | DL4[ 4])
// DL3[ 5] = (DL1[ 5] & DL1[19]) | (DL2[ 5] & DL2[19]);        = DL2[ 5] & (DL1[ 5] | DL4[ 5])
// DL3[ 6] = (DL1[ 6] & DL1[20]) | (DL2[ 6] & DL2[20]);        = DL2[ 6] & (DL1[ 6] | DL4[ 6])
// DL3[ 7] = (DL1[ 7] & DL1[21]) | (DL2[ 7] & DL2[21]);        = DL2[ 7] & (DL1[ 7] | DL4[ 7])
// DL3[ 8] = (DL1[ 8] & DL1[22]) | (DL2[ 8] & DL2[22]);        = DL2[ 8] & (DL1[ 8] | DL4[ 8])
// DL3[ 9] = (DL1[ 9] & DL1[23]) | (DL2[ 9] & DL2[23]);        = DL2[ 9] & (DL1[ 9] | DL4[ 9])
// DL3[10] = (DL1[10] & DL1[24]) | (DL2[10] & DL2[24]);        = DL2[10] & (DL1[10] | DL4[10])
// DL3[11] = (DL1[11] & DL1[25]) | (DL2[11] & DL2[25]);        = DL2[11] & (DL1[11] | DL4[11]) --Not Used
// DL3[12] = (DL1[12] & DL1[26]) | (DL2[12] & DL2[26]);        = DL2[12] & (DL1[12] | DL4[12])
// DL3[13] = (DL1[13] & DL1[27]) | (DL2[13] & DL2[27]);        = DL2[13] & (DL1[13] | DL4[13])

// uGarbled = DL3[1] | DL3[2] | DL3[3] | DL3[4] | DL3[5] | DL3[6] | DL3[7] | DL3[8] | DL3[9] | DL3[10] | DL3[12] | DL3[13];

// FRAME = (DL2[0] & DL2[14]) & !GARBLED;

//
// The "off air" format is,,
// _F1_C1_A1_C2_A2_C4_A4_xx_B1_D1_B2_D2_B4_D4_F2_xx_xx_SPI_
//
// Bit spacing is 1.45uS, with 0.45uS high, and 1.00us low.
//
// The bit spacings are..
// F1 :  0.00,   
//       1.45,  2.90,  4.35,  5.80,  7.25,  8.70, 
// Z  : 10.15, 
//    : 11.60, 13.05, 14.50, 15.95, 17.40, 18.85, 
// F2 : 20.30, 
// Z  : 21.75, 23.20
// SPI: 24.65 
//
// This equates to the following sample point centers at 20Mhz.
// [  0], 
// [ 29], [ 58], [ 87], [116], [145], [174], 
// [203], 
// [232], [261], [290], [319], [348], [377]
// [406]
// [435], [464]
// [493]
//
// We know that this is a supposed to be a binary stream, so the signal
// should either be a 1 or a 0.
//
#ifdef _ARMNEON
//
// puAF and pLanes ***MUST*** all be DWORD aligned. If they aren't this code will fault.
//
uint32_t detectModeAArmNEON(uint16_t* puAF, tMessageRaw* pLane) {

    __asm volatile(
        ".arch armv7-a\n\t"
        ".fpu  neon\n\t"

        //      "ldr     r0, [%1]\n\t"               //@ mov r0, puAF
        //      "ldr     r1, [%2]\n\t"               //@ mov r1, pLanes

        "pld        [r0,  #752]\n\t"         //@ Warn the memory system we're going to want 

        "push       { r4-r12, lr }\n\t"      //@ Push the registers that need to be preserved onto the stack
        "vpush      { q4-q7 }\n\t"           //@ Push neon registers that need to be preserved onto the stack

        "mov        r8, #0\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//      q15 = min (F1,F2);      -- This is the lower  of F1(377) and F2(783) 
//      r0 is QWORD aligned, so F1 and F2 are NOT DWORD aligned
//
        "add        r3, r0, #752\n\t"        //@ r3  = &puAF[376] (QWORD aligned)
        "vldm       r3, { q8, q9 }\n\t"      //@ q9  = d19:d18 = uAF[391] : uAF[390] : uAF[389] : uAF[388] : uAF[387] : uAF[386] : uAF[385] : uAF[384]
                                             //@ q8  = d17:d16 = uAF[383] : uAF[382] : uAF[381] : uAF[380] : uAF[379] : uAF[378] : uAF[377] : uAF[376]
        "pld        [r0, #1552]\n\t"         //@ Warn the memory system we're going to want
        "vext.8     q10, q8, q9, #2\n\t"     //@ q10 = d21:d20 = uAF[384] : uAF[383] : uAF[382] : uAF[381] : uAF[380] : uAF[379] : uAF[378] : uAF[377]

        "add        r3, r0, #1552\n\t"       //@ r3  = &puAF[776] (QWORD aligned)
        "vldm       r3, { q8, q9 }\n\t"      //@ q9  = d23:d22 = uAF[791] : uAF[790] : uAF[789] : uAF[788] : uAF[787] : uAF[786] : uAF[785] : uAF[784]
                                             //@ q8  = d23:d22 = uAF[783] : uAF[782] : uAF[781] : uAF[780] : uAF[779] : uAF[778] : uAF[777] : uAF[776]       
        "pld        [r0, #1160]\n\t"         //@ Warn the memory system we're going to want
        "vext.8     q11, q8, q9, #14\n\t"    //@ q11 = d21:d20 = uAF[790] : uAF[789] : uAF[788] : uAF[787] : uAF[786] : uAF[785] : uAF[784] : uAF[783]
                                            
        "vmin.U16   q15, q10, q11\n\t"       //@ q15 = d31:d30 = min(uAF[790],uAF[384]) : ..... : min(uAF[783],uAF[377])
//
//      q14 = max (Z7,Z15,Z16); -- This is the higher of Z7(580), Z15(812) and Z16(841)
//      r0 is QWORD aligned, so Z7 is QWORD aligned, Z15 in DWORD aligned. Z16 is NOT DWORD aligned
//
        "mov        r3, #1160\n\t"
        "add        r3, r0, r3\n\t"          //@ r3  = &puAF[580] (QWORD aligned)
        "vld1.64    { q14 }, [r3]\n\t"       //@ q14 = d29:d28 = uAF[587] : uAF[586] : uAF[585] : uAF[584] : uAF[583] : uAF[582] : uAF[581] : uAF[580]
        "pld        [r0, #1624]\n\t"         //@ Warn the memory system we're going to want

        "mov        r3, #1624\n\t"
        "add        r3, r0, r3\n\t"          //@ r3  = &puAF[812] (DWORD aligned)
        "vld1.64    { q8 }, [r3]\n\t"        //@ q8  = d17:d16 = uAF[819] : uAF[818] : uAF[817] : uAF[816] : uAF[815] : uAF[814] : uAF[813] : uAF[812]
        "pld        [r0, #1680]\n\t"         //@ Warn the memory system we're going to want

        "vmax.U16   q14, q14, q8\n\t"        //@ q14 = d29:d28 = max(uAF[819],uAF[587]) : ..... : max(uAF[812],uAF[580])

        "add        r3, r0, #1680\n\t"       //@ r3  = &puAF[840] (QWORD aligned)
        "vldm       r3, { q8, q9 }\n\t"      //@ q9  = d19:d18 = uAF[855] : uAF[854] : uAF[853] : uAF[852] : uAF[851] : uAF[850] : uAF[849] : uAF[848]
                                             //@ q8  = d17:d16 = uAF[847] : uAF[846] : uAF[845] : uAF[844] : uAF[843] : uAF[842] : uAF[841] : uAF[840]
        "vext.8     q8, q8, q9, #2\n\t"      //@ q8  = d17:d16 = uAF[848] : uAF[847] : uAF[846] : uAF[845] : uAF[844] : uAF[843] : uAF[842] : uAF[841]

        "vmax.U16   q14, q14, q8\n\t"        //@ q14 = d29:d28 = max(uAF[848],uAF[819],uAF[587]) : ..... : max(uAF[841],uAF[812],uAF[580])
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//      "vshr.U16   q13, q15,#1\n\t"         //@ q13 = (min(uAF[783],uAF[377]) / 2
//      "vcgt.U16   q0,  q13, q14\n\t"       //@ q0  = (min(uAF[783],uAF[377]) / 2 > max(uAF[841],uAF[812],uAF[580])

        "vqsub.U16  q13, q15, q14\n\t"       //@ q13 = min(uAF[783],uAF[377]) - max(uAF[841],uAF[812],uAF[580]) 
        "mov        r2, #0x03FF\n\t"
        "vdup.16    q12, r2\n\t"             //@ q12 = 0x03FF : 0x03FF : 0x03FF : 0x03FF : 0x03FF : 0x03FF : 0x03FF : 0x03FF

        "vcgt.U16   q0,  q13, q12\n\t"       //@ q0  = (min(uAF[783],uAF[377] - max(uAF[841],uAF[812],uAF[580])) > MODEAC_MSG_SQUELCH_LEVEL

//      "vcgt.U16   q1,  q13, q12\n\t"       //@ q1  = (min(uAF[783],uAF[377] - max(uAF[841],uAF[812],uAF[580])) > MODEAC_MSG_SQUELCH_LEVEL
//      "vand       q0, q1, q0\n\t"          //@ q0  = min(F1,F2) > (2 * max(Z7,Z15,Z16)) && (min(F1,F2) - max(Z7,Z15,Z16)) > MODEAC_MSG_SQUELCH_LEVEL

        "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
        "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00

        "vmov       r4, r5, d3\n\t"
        "orrs       r3, r4, r5\n\t"
        "beq        ExitNeon\n\t"            //@ if all 8 lanes fail, goto ExitNeon
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//      Work out decision levels, and put in uLevelH in q13, and uLevelL in q11
//
//      q13 = (max(F1,F2) + min(Z7,Z15,Z16)) / 2
        "vshr.S16   q13,q13,#1\n\t"          //@ q13 = (min(uAF[783],uAF[377]) - max(uAF[841],uAF[812],uAF[580])) / 2
        "vsub.U16   q13,q15,q13\n\t"         //@ q13 = (min(uAF[783],uAF[377]) + max(uAF[841],uAF[812],uAF[580])) / 2

//		q13 = mean bit signal level.
//
//      Set q13 = uLevelH = mean * sqrt(2)
//      "vshr.U16   q11,q13,#2\n\t"          //@ q11 = (min(uAF[783],uAF[377]) - max(uAF[841],uAF[812],uAF[580])) / 4
//      "vadd.U16   q13,q13,q11\n\t"         //@ q13 = q13 * 1.25
//      "vshr.U16   q11,q13,#1\n\t"          //@ q11 = (min(uAF[783],uAF[377]) - max(uAF[841],uAF[812],uAF[580])) / 8
//      "vadd.U16   q13,q13,q11\n\t"         //@ q13 = q13 * 1.375
//      "vshr.U16   q11,q13,#2\n\t"          //@ q11 = (min(uAF[783],uAF[377]) - max(uAF[841],uAF[812],uAF[580])) / 32
//      "vadd.U16   q13,q13,q11\n\t"         //@ q13 = q13 * 1.40625
//      "vshr.U16   q11,q13,#2\n\t"          //@ q11 = (min(uAF[783],uAF[377]) - max(uAF[841],uAF[812],uAF[580])) / 128
//      "vadd.U16   q13,q13,q11\n\t"         //@ q13 = q13 * 1.4140625 (sqrt(2))
//
//      Set q11 = uLevelL = mean * sqrt(2) / 2
//      "vshr.U16   q11,q13,#1\n\t"          //@ q11 = q13 / 2

//
//      Calculate the Signal Strength for F1 and F2, and sum into q15..q14
//
        "vaddl.U16  q15,d21,d23\n\t"         //@ q15 = uAF[790]+uAF[384] : uAF[789]+uAF[383] : uAF[788]+uAF[382] : uAF[787]+uAF[381]
        "vaddl.U16  q14,d20,d22\n\t"         //@ q14 = uAF[786]+uAF[380] : uAF[785]+uAF[379] : uAF[784]+uAF[378] : uAF[783]+uAF[377]
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Bit encoding is          b13    b12     b11     b10     b9       b8       b7       b6       b5       b4       b3       b2       b1       b0 
//      Calculate DL4 from uAF[0],uAF[29],uAF[58],uAF[87],uAF[116],uAF[145],uAF[174],uAF[203],uAF[232],uAF[261],uAF[290],uAF[319],uAF[348],uAF[377]
//
        "vmov.I16   q12, #0\n\t"             //@ q12    = 0 = BitsSet
        "vmov.I16   q6,  #0\n\t"             //@ q6     = 0 = DL4
        "mov        r10, #7\n\t"             //@ r10    = loop count, two bits per loop, skip uAF[377]
        "b          DL4Loop1\n\t"

    "DL4Loop:\n\t"
        "vldm       r0, { q4, q5 }\n\t"      //@ q5  = uAF[14] : uAF[13] : uAF[12] : uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] : uAF[ 7]
                                             //@ q4  = uAF[ 6] : uAF[ 5] : uAF[ 4] : uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] : uAF[-1]
        "add        r0, r0, #60\n\t"
        "vext.8     q4, q4, q5, #2\n\t"      //@ q4  = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4] : uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0]

        "vcgt.U16   q1, q4, q13\n\t"         //@ q1 = puAF[7..0] > uLevelH 

        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q6, q6, #1\n\t"          //@ q6 = q6 << 1
        "vorr       q6, q6, q1\n\t"          //@ orr in this bit

    "DL4Loop1:\n\t"
        "vld1.64    { q4 }, [r0]\n\t"        //@ q4 = puAF[7..0] Pick up eight uAF input values
        "add        r0, r0, #56\n\t"

        "vcgt.U16   q1, q4, q13\n\t"         //@ q1 = puAF[7..0] > uLevelH 

        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q6, q6, #1\n\t"          //@ q6 = q6 << 1
        "vorr       q6, q6, q1\n\t"          //@ orr in this bit

        "subs       r10,r10,#1\n\t"
        "bne        DL4Loop\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// We can skip F1, Z7 and F2 since we already know they are 1-0-1
//
        "add        r0,  #60\n\t"            //@ skip F1
        "vmov.I16   q7,  #0\n\t"             //@ q7     = 0 = DL2
        "mov        r10, #3\n\t"             //@ r10    = loop count for C1 to A4

    "DL2LoopAC:\n\t"
        "vld1.64    { q4 }, [r0]\n\t"        //@ q4 = puAF[7..0] Pick up eight uAF input values
        "add        r0, r0, #56\n\t"

        "vcgt.U16   q1, q4, q13\n\t"         //@ q1 = puAF[7..0] > uLevelH 
        //"vcgt.U16   q2, q11,q4\n\t"          //@ q2 = uLevelL > puAF[7..0]
		//"vorr       q2, q1, q2\n\t"          //@ q2 = (puAF[7..0] > uLevelH) || (uLevelL > puAF[7..0])
		//"vand       q0, q0, q2\n\t"          //@ q0 = Lane7Valid...Lane0Valid

        "vand       q4, q1, q4\n\t"          //@ clear the uAF xalues which are less than the decision level
        "vaddw.U16  q15,q15,d9\n\t"          //@ q15 += uAF[7] : uAF[6] : uAF[5] : uAF[4]
        "vaddw.U16  q14,q14,d8\n\t"          //@ q14 += uAF[3] : uAF[2] : uAF[1] : uAF[0]

        "vshr.U16   q1, q1, #15\n\t"
        "vadd.U16   q12,q12,q1\n\t"          //@ count the set bits
        "vshl.U16   q7, q7, #1\n\t"          //@ q7 = q7 << 1
        "vorr       q7, q7, q1\n\t"          //@ orr in this bit

        "vldm       r0, { q4, q5 }\n\t"      //@ q5  = uAF[14] : uAF[13] : uAF[12] : uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] : uAF[ 7]
                                             //@ q4  = uAF[ 6] : uAF[ 5] : uAF[ 4] : uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] : uAF[-1]
        "add        r0, r0, #60\n\t"
        "vext.8     q4, q4, q5, #2\n\t"      //@ q4  = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4] : uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0]

        "vcgt.U16   q1, q4, q13\n\t"         //@ q1 = puAF[7..0] > uLevelH 
        //"vcgt.U16   q2, q11,q4\n\t"          //@ q2 = uLevelL > puAF[7..0]
		//"vorr       q2, q1, q2\n\t"          //@ q2 = (puAF[7..0] > uLevelH) || (uLevelL > puAF[7..0])
		//"vand       q0, q0, q2\n\t"          //@ q0 = Lane7Valid...Lane0Valid

        "vand       q4, q1, q4\n\t"          //@ clear the uAF values which are less than the decision level
        "vaddw.U16  q15,q15,d9\n\t"          //@ q15 += uAF[7] : uAF[6] : uAF[5] : uAF[4]
        "vaddw.U16  q14,q14,d8\n\t"          //@ q14 += uAF[3] : uAF[2] : uAF[1] : uAF[0]

        "vshr.U16   q1, q1, #15\n\t"
        "vadd.U16   q12,q12,q1\n\t"          //@ count the set bits
        "vshl.U16   q7, q7, #1\n\t"          //@ q7 = q7 << 1
        "vorr       q7, q7, q1\n\t"          //@ orr in this bit

        "subs       r10,r10,#1\n\t"
        "bne        DL2LoopAC\n\t"
//
// Skip Z7
//
        "add        r0,     #56\n\t"         //@ skip Z7
        "vshl.U16   q7, q7, #1\n\t"          //@ q7 = q7 << 1
        "mov        r10,    #3\n\t"          //@ r10  = loop count for B1 to D4

    "DL2LoopBD:\n\t"
        "vldm       r0, { q4, q5 }\n\t"      //@ q5  = uAF[14] : uAF[13] : uAF[12] : uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] : uAF[ 7]
                                             //@ q4  = uAF[ 6] : uAF[ 5] : uAF[ 4] : uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] : uAF[-1]
        "add        r0, r0, #60\n\t"
        "vext.8     q4, q4, q5, #2\n\t"      //@ q4  = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4] : uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0]

        "vcgt.U16   q1, q4, q13\n\t"         //@ q1 = puAF[7..0] > uLevelH 
        //"vcgt.U16   q2, q11,q4\n\t"          //@ q2 = uLevelL > puAF[7..0]
		//"vorr       q2, q1, q2\n\t"          //@ q2 = (puAF[7..0] > uLevelH) || (uLevelL > puAF[7..0])
		//"vand       q0, q0, q2\n\t"          //@ q0 = Lane7Valid...Lane0Valid

        "vand       q4, q1, q4\n\t"          //@ clear the uAF xalues which are less than the decision level
        "vaddw.U16  q15,q15,d9\n\t"          //@ q15 += uAF[7] : uAF[6] : uAF[5] : uAF[4]
        "vaddw.U16  q14,q14,d8\n\t"          //@ q14 += uAF[3] : uAF[2] : uAF[1] : uAF[0]

        "vshr.U16   q1, q1, #15\n\t"
        "vadd.U16   q12,q12,q1\n\t"          //@ count the set bits
        "vshl.U16   q7, q7, #1\n\t"          //@ q7 = q7 << 1
        "vorr       q7, q7, q1\n\t"          //@ orr in this bit

        "vld1.64    { q4 }, [r0]\n\t"        //@ q4 = puAF[7..0] Pick up eight uAF input values
        "add        r0, r0, #56\n\t"

        "vcgt.U16   q1, q4, q13\n\t"         //@ q1 = puAF[7..0] > uLevelH 
        //"vcgt.U16   q2, q11,q4\n\t"          //@ q2 = uLevelL > puAF[7..0]
		//"vorr       q2, q1, q2\n\t"          //@ q2 = (puAF[7..0] > uLevelH) || (uLevelL > puAF[7..0])
		//"vand       q0, q0, q2\n\t"          //@ q0 = Lane7Valid...Lane0Valid

        "vand       q4, q1, q4\n\t"          //@ clear the uAF xalues which are less than the decision level
        "vaddw.U16  q15,q15,d9\n\t"          //@ q15 += uAF[7] : uAF[6] : uAF[5] : uAF[4]
        "vaddw.U16  q14,q14,d8\n\t"          //@ q14 += uAF[3] : uAF[2] : uAF[1] : uAF[0]

        "vshr.U16   q1, q1, #15\n\t"
        "vadd.U16   q12,q12,q1\n\t"          //@ count the set bits
        "vshl.U16   q7, q7, #1\n\t"          //@ q7 = q7 << 1
        "vorr       q7, q7, q1\n\t"          //@ orr in this bit

        "subs       r10,r10,#1\n\t"
        "bne        DL2LoopBD\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
        //"vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
        //"vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00

        //"vmov       r4, r5, d3\n\t"
        //"orrs       r3, r4, r5\n\t"
        //"beq        ExitNeon\n\t"            //@ if all 8 lanes fail, goto ExitNeon
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// We can skip F2, Z15 and Z16 since we already know they are 1-0-0
//
//       Go straight to SPI
        "add        r0,  #176\n\t"           //@ skip F2, Z15, Z16 and align with SPI

        "vld1.64    { q4 }, [r0]\n\t"        //@ q4 = puAF[7..0] Pick up eight uAF input values
        "add        r0, r0, #56\n\t"

        "vcgt.U16   q1, q4, q13\n\t"         //@ q1 = puAF[7..0] > decision levels 

        "vand       q4, q1, q4\n\t"          //@ clear the uAF xalues which are less than the decision level
        "vaddw.U16  q15,q15,d9\n\t"          //@ q15 += uAF[7] : uAF[6] : uAF[5] : uAF[4]
        "vaddw.U16  q14,q14,d8\n\t"          //@ q14 += uAF[3] : uAF[2] : uAF[1] : uAF[0]

        "vshr.U16   q8, q1, #15\n\t"         //@ Put the SPI bit in q8
        "vadd.U16   q12,q12,q8\n\t"          //@ count the set bits

        "mov        r10, #5\n\t"             //@ r10    = loop count
    "DL1Loop:\n\t"
        "vldm       r0, { q4, q5 }\n\t"      //@ q5  = uAF[14] : uAF[13] : uAF[12] : uAF[11] : uAF[10] : uAF[ 9] : uAF[ 8] : uAF[ 7]
                                             //@ q4  = uAF[ 6] : uAF[ 5] : uAF[ 4] : uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0] : uAF[-1]
        "add        r0, r0, #60\n\t"
        "vext.8     q4, q4, q5, #2\n\t"      //@ q4  = uAF[ 7] : uAF[ 6] : uAF[ 5] : uAF[ 4] : uAF[ 3] : uAF[ 2] : uAF[ 1] : uAF[ 0]

        "vcgt.U16   q1, q4, q13\n\t"         //@ q1 = puAF[7..0] > uLevelH 

        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q8, q8, #1\n\t"          //@ q8 = q8 << 1
        "vorr       q8, q8, q1\n\t"          //@ orr in this bit

        "vld1.64    { q4 }, [r0]\n\t"        //@ q4 = puAF[7..0] Pick up eight uAF input values
        "add        r0, r0, #56\n\t"

        "vcgt.U16   q1, q4, q13\n\t"         //@ q1 = puAF[7..0] > uLevelH 

        "vshr.U16   q1, q1, #15\n\t"
        "vshl.U16   q8, q8, #1\n\t"          //@ q8 = q8 << 1
        "vorr       q8, q8, q1\n\t"          //@ orr in this bit

        "subs       r10,r10,#1\n\t"
        "bne        DL1Loop\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//    q0  = LaneValid
//    q6  = uDL4Bits
//    q7  = uDL2Bits
//    q8  = uDL1Bits
//    q12 = BitCounts
//
//    uGarbled   = uDL2Bits & (uDL1Bits | uDL4Bits) & 0x1BFF;
//
        "vorr       q6, q6, q8\n\t"          //@ q6   =            (uDL1Bits | uDL4Bits)
        "vand       q6, q7, q6\n\t"          //@ q6   = uDL2Bits & (uDL1Bits | uDL4Bits)
        "vand.I16   q6, #0x1BFF\n\t"         //@ q6   = uDL2Bits & (uDL1Bits | uDL4Bits) & 0x1BFF

        "vceq.I16   q6, q6, #0\n\t"          //@ q6   = 0 == (uDL2Bits & (uDL1Bits | uDL4Bits) & 0x1BFF) ? 0xFFFF : 0x0000
        "vand       q0, q0, q6\n\t"

        "vmov       q1, q0\n\t"              //@ d2=d3= Lane7Valid : Lane6Valid : Lane5Valid : Lane4Valid : Lane3Valid : Lane2Valid : Lane1Valid : Lane0Valid
        "vuzp.8     d2, d3\n\t"              //@        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00        FF/00

        "vmov       r4, r5, d3\n\t"
        "orrs       r3, r4, r5\n\t"
        "beq        ExitNeon\n\t"            //@ if all 8 lanes fail, goto ExitNeon
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//    Descramble the Mode A Bits into q4
//
        "vshr.U16   q2, q7, #7\n\t"
        "vmov.U16   q3, #0x0010\n\t"
        "vand       q4, q2, q3\n\t"          //@ q4 |= (q7 >> 7) & 0x0010 (A1) 
 
        "vshr.U16   q2, q7, #4\n\t"
        "vmov.U16   q3, #0x0020\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 >> 4) & 0x0020 (A2) 
 
        "vshr.U16   q2, q7, #1\n\t"
        "vmov.U16   q3, #0x0040\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 >> 1) & 0x0040 (A4) 
 
        "vshr.U16   q2, q7, #5\n\t"
        "vmov.U16   q3, #0x0001\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 >> 5) & 0x0001 (B1) 
 
        "vshr.U16   q2, q7, #2\n\t"
        "vmov.U16   q3, #0x0002\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 >> 2) & 0x0002 (B2) 
 
        "vshl.U16   q2, q7, #1\n\t"
        "vmov.U16   q3, #0x0004\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 << 1) & 0x0004 (B4) 

        "vmov.U16   q3, #0x1000\n\t"
        "vand       q2, q7, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 >> 0) & 0x1000 (C1) 
 
        "vshl.U16   q2, q7, #3\n\t"
        "vmov.U16   q3, #0x2000\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 << 3) & 0x2000 (C2) 
 
        "vshl.U16   q2, q7, #6\n\t"
        "vmov.U16   q3, #0x4000\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 << 6) & 0x4000 (C4) 

        "vshl.U16   q2, q7, #4\n\t"
        "vmov.U16   q3, #0x0100\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 << 4) & 0x0100 (D1) 
 
        "vshl.U16   q2, q7, #7\n\t"
        "vmov.U16   q3, #0x0200\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 << 7) & 0x0200 (D2) 
 
        "vshl.U16   q2, q7, #10\n\t"
        "vmov.U16   q3, #0x0400\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q7 << 10) & 0x0400 (D4) 

        "vshl.U16   q2, q8, #5\n\t"
        "vmov.U16   q3, #0x8000\n\t"
        "vand       q2, q2, q3\n\t"
        "vorr       q4, q4, q2\n\t"          //@ q4 |= (q8 << 5) & 0x8000 (SPI) 
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//    We've got between one and eight valid ModeA's
//
        "vuzp.8     d0, d1\n\t"                       //@ FF/00 : FF/00 : FF/00 : FF/00 : FF/00 : FF/00 : FF/00 : FF/00
        "vmov       r4, r5, d1\n\t"

        "vmov       q9,  q14\n\t"                     //@ Get sigStrength[3:0] into q9
        "vmovl.U16  q10, d24\n\t"                     //@ Extend BitCount[3:0] to 32 bits, and into q10
        "vmovl.U16  q11, d8\n\t"                      //@ Extend uModeA[3:0] to 32 bits, and into q11

        "tst        r4,#0x00000080\n\t"               //@ Lane 0 is valid ?
        "beq        Lane0End\n\t"
        "mov        r2, r1\n\t"
        "vst3.32    {d18[0],d20[0],d22[0]}, [r2]\n\t" //@ Store Lane 0 sigStrength, nBitsSet, uModeA
        "orr        r8, r8, #0x01\n\t"                //@ Set Lane 0 Valid Bit
    "Lane0End:\n\t" 

        "tst        r4,#0x00008000\n\t"               //@ Lane 1 is valid ?
        "beq        Lane1End\n\t"
        "add        r2, r1, #0x40\n\t"
        "vst3.32    {d18[1],d20[1],d22[1]}, [r2]\n\t" //@ Store Lane 1 sigStrength, nBitsSet, uModeA
        "orr        r8, r8, #0x02\n\t"                //@ Set Lane 1 Valid Bit
    "Lane1End:\n\t"

        "tst        r4,#0x00800000\n\t"               //@ Lane 2 is valid ?
        "beq        Lane2End\n\t"
        "add        r2, r1, #0x80\n\t"
        "vst3.32    {d19[0],d21[0],d23[0]}, [r2]\n\t" //@ Store Lane 2 sigStrength, nBitsSet, uModeA
        "orr        r8, r8, #0x04\n\t"                //@ Set Lane 2 Valid Bit
    "Lane2End:\n\t"

        "tst        r4,#0x80000000\n\t"               //@ Lane 3 is valid ?
        "beq        Lane3End\n\t"
        "add        r2, r1, #0xC0\n\t"
        "vst3.32    {d19[1],d21[1],d23[1]}, [r2]\n\t" //@ Store Lane 3 sigStrength, nBitsSet, uModeA
        "orr        r8, r8, #0x08\n\t"                //@ Set Lane 3 Valid Bit
    "Lane3End:\n\t"

        "vmov       q9,  q15\n\t"                     //@ Get sigStrength[7:4] into q9
        "vmovl.U16  q10, d25\n\t"                     //@ Extend BitCount[7:4] to 32 bits, and into q10
        "vmovl.U16  q11, d9\n\t"                      //@ Extend uModeA[7:4] to 32 bits, and into q11
 
        "tst        r5,#0x00000080\n\t"               //@ Lane 4 is valid ?
        "beq        Lane4End\n\t"
        "add        r2, r1, #0x100\n\t"
        "vst3.32    {d18[0],d20[0],d22[0]}, [r2]\n\t" //@ Store Lane 4 sigStrength, nBitsSet, uModeA
        "orr        r8, r8, #0x10\n\t"                //@ Set Lane 4 Valid Bit
    "Lane4End:\n\t"

        "tst        r5,#0x00008000\n\t"               //@ Lane 5 is valid ?
        "beq        Lane5End\n\t"
        "add        r2, r1, #0x140\n\t"
        "vst3.32    {d18[1],d20[1],d22[1]}, [r2]\n\t" //@ Store Lane 5 sigStrength, nBitsSet, uModeA
        "orr        r8, r8, #0x20\n\t"                //@ Set Lane 5 Valid Bit
    "Lane5End:\n\t"

        "tst        r5,#0x00800000\n\t"               //@ Lane 6 is valid ?
        "beq        Lane6End\n\t"
        "add        r2, r1, #0x180\n\t"
        "vst3.32    {d19[0],d21[0],d23[0]}, [r2]\n\t" //@ Store Lane 6 sigStrength, nBitsSet, uModeA
        "orr        r8, r8, #0x40\n\t"                //@ Set Lane 6 Valid Bit
    "Lane6End:\n\t"

        "tst        r5,#0x80000000\n\t"               //@ Lane 7 is valid ?
        "beq        Lane7End\n\t"
        "add        r2, r1, #0x1C0\n\t"
        "vst3.32    {d19[1],d21[1],d23[1]}, [r2]\n\t" //@ Store Lane 7 sigStrength, nBitsSet, uModeA
        "orr        r8, r8, #0x80\n\t"                //@ Set Lane 7 Valid Bit
    "Lane7End:\n\t"
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

static void detectBufferModeA(uint16_t* puAF, uint64_t lTime) {

	uint32_t j, uLanes;
	tMessageRaw** pQ;

	memset(&rmLane, 0, sizeof(tMessageRaw) * 8);

	// Find the end of any current queue
	pQ = &Modes.pModeA_List;
    while (*pQ) {
		pQ = & (*pQ)->pNext;
	}

	for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j += 8) {

		if ( (uLanes = detectModeAArmNEON(puAF, rmLane))) { // We have found a new ModeA/C in the data

			if ((uLanes & 0x01)) {
				rmLane[0].uMsgType = 32;
        		rmLane[0].lTime = lTime;
				pQ = enQueueModeA(&rmLane[0], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

			if ((uLanes & 0x02)) {
				rmLane[1].uMsgType = 32;
        		rmLane[1].lTime = lTime+1;
				pQ = enQueueModeA(&rmLane[1], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

			if ((uLanes & 0x04)) {
				rmLane[2].uMsgType = 32;
        		rmLane[2].lTime = lTime+2;
				pQ = enQueueModeA(&rmLane[2], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

			if ((uLanes & 0x08)) {
				rmLane[3].uMsgType = 32;
        		rmLane[3].lTime = lTime+3;
				pQ = enQueueModeA(&rmLane[3], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

			if ((uLanes & 0x10)) {
				rmLane[4].uMsgType = 32;
        		rmLane[4].lTime = lTime+4;
				pQ = enQueueModeA(&rmLane[4], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

			if ((uLanes & 0x20)) {
				rmLane[5].uMsgType = 32;
        		rmLane[5].lTime = lTime+5;
				pQ = enQueueModeA(&rmLane[5], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

			if ((uLanes & 0x40)) {
				rmLane[6].uMsgType = 32;
        		rmLane[6].lTime = lTime+6;
				pQ = enQueueModeA(&rmLane[6], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

			if ((uLanes & 0x80)) {
				rmLane[7].uMsgType = 32;
        		rmLane[7].lTime = lTime+7;
				pQ = enQueueModeA(&rmLane[7], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

        } else if (Modes.uModeATimeout > 8){
            Modes.uModeATimeout -= 8;

		} else if (Modes.uModeATimeout) {
            Modes.uModeATimeout = 0;
            pQ = postModeAQueue();
        }

		lTime += 8;
		puAF += 8;
	}
}

#elif _ARMSIMD

//
// puAF and pLane ***MUST*** all be DWORD aligned. If they aren't this code will fault.
//
uint32_t detectModeAArmSIMD(uint16_t* puAF, tMessageRaw* pLane) {

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
//      r10 = min (F1,F2);      -- This is the lower  of F1(377) and F2(783) 
//      r0 is DWORD aligned, so F1 and F1 are NOT DWORD aligned
//
//      "ldr     r4, [r0,  #754]\n\t"        //@ r4   = uAF[378-377] Pick up two uAF input values r4.h = uAF[378], r4.l = uAF[377]
        "add     r11, r0,  #752\n\t"         //@ r11  = &puAF[376] (DWORD aligned)
        "ldm     r11, { r4, r10 }\n\t"       //@ r4.h = uAF[377]     r10.h = puAF[379]
                                             //@ r4.l = uAF[376]     r10.l = puAF[378]
        "pkhbt   r4, r10, r4\n\t"            //@ r4.h = uAF[377]  
                                             //@ r4.l = uAF[378]
//      "ldr     r5, [r0, #1566]\n\t"        //@ r4   = uAF[784-783] Pick up two uAF input values r5.h = uAF[784], r5.l = uAF[783]
        "add     r11, r11, #812\n\t"         //@ r11  = &puAF[782] (DWORD aligned)
        "ldm     r11, { r5, r10 }\n\t"       //@ r5.h = uAF[783]     r10.h = puAF[785]
                                             //@ r5.l = uAF[782]     r10.l = puAF[784]
        "pkhbt   r5, r10, r5\n\t"            //@ r5.h = uAF[783]  
//                                           //@ r5.l = uAF[784]
//      r10.h = min(uAF[783],uAF[377]),          
//      r10.l = min(uAF[784],uAF[378])
//
        "usub16  r10,r4, r5\n\t"             //@ r10.h =  uAF[377] - uAF[783], GE[3:2] set if no borrow, clear if borrow
                                             //@ r10.l =  uAF[378] - uAF[784], GE[1:0] set if no borrow, clear if borrow
        "sel     r10,r5, r4\n\t"             //@ r10.h = (uAF[377] > uAF[783]) ? uAF[377] : uAF[783]
                                             //@ r10.l = (uAF[378] > uAF[784]) ? uAF[378] : uAF[784]
        "ror     r10,r10,#16\n\t"            //@ r10.h = min(uAF[784], uAF[378]) 
                                             //@ r10.h = min(uAF[783], uAF[377]) 
//
//      r11 = max (Z7,Z15,Z16); -- This is the higher of Z7(580), Z15(812) and Z16(841)
//      r0 is DWORD aligned, so Z7 and Z15 are DWORD aligned. Z16 is NOT DWORD aligned
//
//      "ldr     r11,[r0, #1682]\n\t"        //@ r11   = uAF[842-841] Pick up two uAF input values r11.h = uAF[842], r11.l = uAF[841]
        "add     r11, r0, #1680\n\t"         //@ r11   = &puAF[840] (DWORD aligned)
        "ldm     r11, { r7, r11 }\n\t"       //@ r7.h  = uAF[841]     r11.h = puAF[843]
                                             //@ r7.l  = uAF[840]     r11.l = puAF[842]
        "pkhbt   r11, r11, r7\n\t"           //@ r11.h = uAF[841] 
//                                           //@ r11.l = uAF[842]
        "ror     r11,r11,#16\n\t"            //@ r11.h = uAF[842], r11.l = uAF[841]

        "ldr     r6, [r0, #1160]\n\t"        //@ r6    = puAF[581-580] Pick up two uAF input values r6.h  = uAF[581], r6.l  = uAF[580]
        "ldr     r7, [r0, #1624]\n\t"        //@ r7    = puAF[813-812] Pick up two uAF input values r7.h  = uAF[813], r7.l  = uAF[812]

        "usub16  r12,r11,r6\n\t"             //@ r12.h =  uAF[842] - uAF[581], GE[3:2] set if no borrow, clear if borrow
                                             //@ r12.l =  uAF[841] - uAF[580], GE[1:0] set if no borrow, clear if borrow
        "sel     r11,r11,r6\n\t"             //@ r11.h = (uAF[842] > uAF[581]) ? uAF[842] : uAF[581]
                                             //@ r11.l = (uAF[841] > uAF[580]) ? uAF[841] : uAF[580]
        "usub16  r12,r11,r7\n\t"             //@ r12.h =  max(uAF[842],uAF[581]) - uAF[813], GE[3:2] set if no borrow, clear if borrow
                                             //@ r12.l =  max(uAF[841],uAF[580]) - uAF[812], GE[1:0] set if no borrow, clear if borrow
        "sel     r11,r11,r7\n\t"             //@ r11.h = (max(uAF[842],uAF[581]) > uAF[813]) ? max(uAF[842],uAF[581]) : uAF[813]
                                             //@ r11.l = (max(uAF[841],uAF[580]) > uAF[812]) ? max(uAF[841],uAF[580]) : uAF[812]
//      r11.h = max(uAF[842],uAF[581],uAF[813])
//      r11.l = max(uAF[841],uAF[580],uAF[812])
//
        "usub16  r12,r11,r10\n\t"            //@ r12.h = max(uAF[842],uAF[581],uAF[813]) - min(uAF[784],uAF[378])
                                             //@ r12.l = max(uAF[841],uAF[580],uAF[812]) - min(uAF[783],uAF[377])
        "mov     r3,     #0x000003FF\n\t"
        "orr     r3, r3, LSL#16\n\t"         //@ r3.h  = max(uAF[842],uAF[581],uAF[813]) - min(uAF[784],uAF[378]) + MODEAC_MSG_SQUELCH_LEVEL
        "qadd16  r3, r3, r12\n\t"            //@ r3.h  = max(uAF[841],uAF[580],uAF[812]) - min(uAF[783],uAF[377]) + MODEAC_MSG_SQUELCH_LEVEL

        "ands    r3, r3, r14\n\t"            //@ if both lanes are false / zero, then both lanes fail, so ...
        "beq     Exit\n\t"                   //@ ...return now
//
// Work out the decision level, and put it in r12
//
        "and     r11,r12,r14\n\t"            //@ preserve the sign bits
        "and     r12,r12,#0xFFFEFFFF\n\t"    //@ r12.h = (max(uAF[842],uAF[581],uAF[813]) - min(uAF[784],uAF[378])) / 2
        "lsr     r12,r12,#1\n\t"             //@ r12.l = (max(uAF[841],uAF[580],uAF[812]) - min(uAF[783],uAF[377])) / 2
        "orr     r12,r12,r11\n\t"            //@ restore the sign bits
        "uadd16  r12,r12,r10\n\t"            //@ r12.h = (max(uAF[842],uAF[581],uAF[813]) + min(uAF[784],uAF[378])) / 2
                                             //@ r12.l = (max(uAF[841],uAF[580],uAF[812]) + min(uAF[783],uAF[377])) / 2
        "lsr     r11,r14,#15\n\t"            //@ r11 = 0x00010001
        "usub16  r12,r12,r11\n\t"
//
// Calculate the Signal Strength for F1 and F2. r4 & r5 are WORD REVERSED
//
        "uxth    r8, r4,    ROR#16\n\t"      //@ r8    = sigStrength1  = r4.h
        "uxth    r9, r4\n\t"                 //@ r9    = sigStrength2  = r4.l
        "uxtah   r8, r8, r5,ROR#16\n\t"      //@ r8    = sigStrength1 += r5.h
        "uxtah   r9, r9, r5\n\t"             //@ r9    = sigStrength2 += r4.l
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//       # If we get here we have either one or two valid preamble decodes
//       #
//       # r0    = puAF
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
//       # r12.h = Lane2 Decision level    r12.l = Lane1 Decision level
//       # r14   = 0x80008000
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//      https://www.radartutorial.eu/13.ssr/sr15.en.html
//
// Bit encoding is          b13    b12     b11     b10     b9       b8       b7       b6       b5       b4       b3       b2       b1       b0 
//      Calculate DL4 from uAF[0],uAF[29],uAF[58],uAF[87],uAF[116],uAF[145],uAF[174],uAF[203],uAF[232],uAF[261],uAF[290],uAF[319],uAF[348],uAF[377]
//
        "mov     r5,  #0\n\t"                //@ r5     = 0
        "mov     r10, #7\n\t"                //@ r10    = loop count, two bits per loop, skip uAF[377]
        "b       DL4Loop1\n\t"

    "DL4Loop:\n\t"
//      "ldr     r4,  [r0],#58\n\t"          //@ r4   = puAF[1-0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldm     r0, { r4, r11 }\n\t"        //@ r4.h = uAF[ 0]     r11.h = puAF[2]
        "add     r0, #60\n\t"                //@ r4.l = uAF[-1]     r11.l = puAF[1]
        "pkhbt   r4, r11, r4\n\t"            //@ r4.h = uAF[ 0]  
                                             //@ r4.l = uAF[ 1]
        "ror     r4,r4,#16\n\t"              //@ r4.h = uAF[ 1] 
                                             //@ r4.l = uAF[ 0]

        "usub16  r11,r12,r4\n\t"             //@ r11.h will be -ve if (uAF[1] > Lane2 Decision Level)
                                             //@ r11.l will be -ve if (uAF[0] > Lane2 Decision Level)
        "and     r11,r11,r14\n\t"
        "orr     r5, r5, r11\n\t"

        "ror     r5, r5, #31\n\t"            //@ r5   = ROL(r5,1)

    "DL4Loop1:\n\t"
        "ldr     r4, [r0],#56\n\t"           //@ r4     = puAF[1-0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "usub16  r11,r12,r4\n\t"             //@ r11.h will be -ve if (uAF[1] > Lane2 Decision Level)
                                             //@ r11.l will be -ve if (uAF[0] > Lane2 Decision Level)
        "and     r11,r11,r14\n\t"
        "orr     r5, r5, r11\n\t"

        "ror     r5, r5, #31\n\t"            //@ r5   = ROL(r5,1)

        "subs    r10,r10,#1\n\t"
        "bne     DL4Loop\n\t"

        "ror     r5, r5, #16\n\t"            // Align DL4 / r5
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Bit encoding is          b14      b13      b12      b11      b10      b9       b8       b7       b6       b5       b4       b3       b2       b1       b0 
//      Calculate DL2 from uAF[377],uAF[406],uAF[435],uAF[464],uAF[493],uAF[522],uAF[551],uAF[580],uAF[609],uAF[638],uAF[667],uAF[696],uAF[725],uAF[754],uAF[783]
//                          F1       C1       A1       C2       A2       C4       A4       Z7       B1       D1       B2      D2       B4       D4       F2
//
// We can skip F1, Z7 and F2 since we already know they are 1-0-1
//
        "add     r0,  #60\n\t"               //@ skip F1 and correct alignment from DL4_01
        "mov     r6,  #0\n\t"                //@ r6     = 0
        "mov     r10, #3\n\t"                //@ r10    = loop count for C1 to A4

    "DL2LoopAC:\n\t"
        "ldr     r4, [r0],#56\n\t"           //@ r4     = puAF[1-0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "usub16  r11,r12,r4\n\t"             //@ r11.h will be -ve if (uAF[1] > Lane2 Decision Level)
                                             //@ r11.l will be -ve if (uAF[0] > Lane2 Decision Level)
        "and     r11,r11,r14\n\t"
        "orr     r6, r6, r11\n\t"

        "ror     r6, r6, #31\n\t"            //@ r6   = ROL(r6,1)

        "tst     r11,    #0x00008000\n\t"    //@ if bit is a '1'
        "uxtahne r8, r8, r4\n\t"             //@ r8    = sigStrength1 += r4.l
        "addne   r3, r3, #0x00000001\n\t"    //@ count the '1' bits

        "tst     r11,    #0x80000000\n\t"    //@ if bit is a '1'
        "uxtahne r9, r9, r4,ROR#16\n\t"      //@ r9    = sigStrength2 += r4.h
        "addne   r3, r3, #0x00010000\n\t"    //@ count the '1' bits

//      "ldr     r4,  [r0],#58\n\t"          //@ r4     = puAF[1-0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldm     r0, { r4, r11 }\n\t"        //@ r4.h = uAF[ 0]     r11.h = puAF[2]
        "add     r0, #60\n\t"                //@ r4.l = uAF[-1]     r11.l = puAF[1]
        "pkhbt   r4, r11, r4\n\t"            //@ r4.h = uAF[ 0]  
                                             //@ r4.l = uAF[ 1]
        "ror     r4,r4,#16\n\t"              //@ r4.h = uAF[ 1] 
                                             //@ r4.l = uAF[ 0]

        "usub16  r11,r12,r4\n\t"             //@ r11.h will be -ve if (uAF[1] > Lane2 Decision Level)
                                             //@ r11.l will be -ve if (uAF[0] > Lane2 Decision Level)
        "and     r11,r11,r14\n\t"
        "orr     r6, r6, r11\n\t"

        "ror     r6, r6, #31\n\t"            //@ r6   = ROL(r6,1)

        "tst     r11,    #0x00008000\n\t"    //@ if bit is a '1'
        "uxtahne r8, r8, r4\n\t"             //@ r8    = sigStrength1 += r4.l
        "addne   r3, r3, #0x00000001\n\t"    //@ count the '1' bits

        "tst     r11,    #0x80000000\n\t"    //@ if bit is a '1'
        "uxtahne r9, r9, r4,ROR#16\n\t"      //@ r9    = sigStrength2 += r4.h
        "addne   r3, r3, #0x00010000\n\t"    //@ count the '1' bits

        "subs    r10,r10,#1\n\t"
        "bne     DL2LoopAC\n\t"
//
// Skip Z7
//
        "add     r0,     #56\n\t"            //@ skip Z7
        "ror     r6, r6, #31\n\t"            //@ r6   = ROL(r6,1)
        "mov     r10,    #3\n\t"             //@ r10  = loop count for B1 to D4

    "DL2LoopBD:\n\t"
//      "ldr     r4,  [r0],#58\n\t"          //@ r4     = puAF[1-0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldm     r0, { r4, r11 }\n\t"        //@ r4.h = uAF[ 0]     r11.h = puAF[2]
        "add     r0, #60\n\t"                //@ r4.l = uAF[-1]     r11.l = puAF[1]
        "pkhbt   r4, r11, r4\n\t"            //@ r4.h = uAF[ 0]  
                                             //@ r4.l = uAF[ 1]
        "ror     r4,r4,#16\n\t"              //@ r4.h = uAF[ 1] 
                                             //@ r4.l = uAF[ 0]

        "usub16  r11,r12,r4\n\t"             //@ r11.h will be -ve if (uAF[1] > Lane2 Decision Level)
                                             //@ r11.l will be -ve if (uAF[0] > Lane2 Decision Level)
        "and     r11,r11,r14\n\t"
        "orr     r6, r6, r11\n\t"

        "ror     r6, r6, #31\n\t"            //@ r6   = ROL(r6,1)

        "tst     r11,    #0x00008000\n\t"    //@ if bit is a '1'
        "uxtahne r8, r8, r4\n\t"             //@ r8    = sigStrength1 += r4.l
        "addne   r3, r3, #0x00000001\n\t"    //@ count the '1' bits

        "tst     r11,    #0x80000000\n\t"    //@ if bit is a '1'
        "uxtahne r9, r9, r4,ROR#16\n\t"      //@ r9    = sigStrength2 += r4.h
        "addne   r3, r3, #0x00010000\n\t"    //@ count the '1' bits

        "ldr     r4, [r0],#56\n\t"           //@ r4     = puAF[1-0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "usub16  r11,r12,r4\n\t"             //@ r11.h will be -ve if (uAF[1] > Lane2 Decision Level)
                                             //@ r11.l will be -ve if (uAF[0] > Lane2 Decision Level)
        "and     r11,r11,r14\n\t"
        "orr     r6, r6, r11\n\t"

        "ror     r6, r6, #31\n\t"            //@ r6   = ROL(r6,1)

        "tst     r11,    #0x00008000\n\t"    //@ if bit is a '1'
        "uxtahne r8, r8, r4\n\t"             //@ r8    = sigStrength1 += r4.l
        "addne   r3, r3, #0x00000001\n\t"    //@ count the '1' bits

        "tst     r11,    #0x80000000\n\t"    //@ if bit is a '1'
        "uxtahne r9, r9, r4,ROR#16\n\t"      //@ r9    = sigStrength2 += r4.h
        "addne   r3, r3, #0x00010000\n\t"    //@ count the '1' bits

        "subs    r10,r10,#1\n\t"
        "bne     DL2LoopBD\n\t"

        "ror     r6, r6, #16\n\t"            // Align DL2 / r6
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
// Bit encoding is          b14      b13      b12      b11      b10      b9       b8       b7       b6        b5        b4        b3        b2        b1        b0 
//      Calculate DL1 from uAF[783],uAF[812],uAF[841],uAF[870],uAF[899],uAF[928],uAF[957],uAF[986],uAF[1015],uAF[1044],uAF[1073],pAF[1102],uAF[1131],uAF[1160],uAF[1189]
//                          F2       Z15      Z16      SPI
// We can skip F2, Z15 and Z16 since we already know they are 1-0-0 and b0 since it isn't used anywhere
//
//      Deal with SPI
        "add     r0,  #176\n\t"              //@ skip F2, Z15, Z16 and align with SPI

        "ldr     r4,  [r0],#56\n\t"          //@ r4     = puAF[1-0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "usub16  r11,r12,r4\n\t"             //@ r11.h will be -ve if (uAF[1] > Lane2 Decision Level)
                                             //@ r11.l will be -ve if (uAF[0] > Lane2 Decision Level)

        "and     r7, r11,r14\n\t"            //@ put SPI into DL1
        "ror     r7, r7, #31\n\t"            //@ r7   = ROL(r7,1)

        "tst     r11,    #0x00008000\n\t"    //@ if SPI bit is a '1'
        "uxtahne r8, r8, r4\n\t"             //@ r8    = sigStrength1 += r4.l
        "addne   r3, r3, #0x00000001\n\t"    //@ count the '1' bits

        "tst     r11,    #0x80000000\n\t"    //@ if SPI bit is a '1'
        "uxtahne r9, r9, r4,ROR#16\n\t"      //@ r9    = sigStrength2 += r4.h
        "addne   r3, r3, #0x00010000\n\t"    //@ count the '1' bits

        "mov     r10, #5\n\t"                //@ r10    = loop count

    "DL1Loop:\n\t"
//      "ldr     r4,  [r0],#58\n\t"          //@ r4     = puAF[1-0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "ldm     r0, { r4, r11 }\n\t"        //@ r4.h = uAF[ 0]     r11.h = puAF[2]
        "add     r0, #60\n\t"                //@ r4.l = uAF[-1]     r11.l = puAF[1]
        "pkhbt   r4, r11, r4\n\t"            //@ r4.h = uAF[ 0]  
                                             //@ r4.l = uAF[ 1]
        "ror     r4,r4,#16\n\t"              //@ r4.h = uAF[ 1] 
                                             //@ r4.l = uAF[ 0]

        "usub16  r11,r12,r4\n\t"             //@ r11.h will be -ve if (uAF[1] > Lane2 Decision Level)
                                             //@ r11.l will be -ve if (uAF[0] > Lane2 Decision Level)
        "and     r11,r11,r14\n\t"
        "orr     r7, r7, r11\n\t"

        "ror     r7, r7, #31\n\t"            //@ r7   = ROL(r7,1)

        "ldr     r4, [r0],#56\n\t"           //@ r4     = puAF[1-0] Pick up two uAF input values r4.h = uAF[ 1], r4.l = uAF[ 0]
        "usub16  r11,r12,r4\n\t"             //@ r11.h will be -ve if (uAF[1] > Lane2 Decision Level)
                                             //@ r11.l will be -ve if (uAF[0] > Lane2 Decision Level)
        "and     r11,r11,r14\n\t"
        "orr     r7, r7, r11\n\t"

        "ror     r7, r7, #31\n\t"            //@ r7   = ROL(r7,1)

        "subs    r10,r10,#1\n\t"
        "bne     DL1Loop\n\t"

        "ror     r7, r7, #16\n\t"            //@ Align DL1 / r7
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//    r5 = uDL4Bits
//    r6 = uDL2Bits
//    r7 = uDL1Bits
//
//    uGarbled   = uDL2Bits & (uDL1Bits | uDL4Bits) & 0x1BFF;
//
        "orr     r4, r5, r7\n\t"             //@ r4   =            (uDL1Bits | uDL4Bits)
        "and     r4, r4, r6\n\t"             //@ r4   = uDL2Bits & (uDL1Bits | uDL4Bits)
        "mov     r5,     #0x00001B00\n\t"    //@ r5   = 0x00001B00 
        "orr     r5, r5, #0x000000FF\n\t"    //@ r5   = 0x00001BFF 
        "orr     r5, r5, LSL#16\n\t"         //@ r5   = 0x1BFF1BFF
        "and     r4, r4, r5\n\t"             //@ r4   = uDL2Bits & (uDL1Bits | uDL4Bits) & 0x1BFF1BFF

        "usat16  r4, #1, r4\n\t"             //@ r5.h = (r4.h != 0) ? 0x0001 : 0x0000;
                                             //@ r5.l = (r4.l != 0) ? 0x0001 : 0x0000;
        "tst     r4     ,#0x00000001\n\t"
        "andne   r3, r3 ,#0xFFFF7FFF\n\t"
        "tst     r4     ,#0x00010000\n\t"
        "andne   r3, r3 ,#0x7FFFFFFF\n\t"

        "ands    r4, r3, r14\n\t"
        "beq     Exit\n\t"                   //@ ...return now
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//    Descramble the Mode A Bits
//
        "and     r4, r6, r14, LSR#4\n\t"
        "lsr     r4, r4,         #7\n\t"     //@ r4  = (r6 & 0x08000800) >> 7 (A1)
        "and     r5, r6, r14, LSR#6\n\t"
        "orr     r4, r5,      LSR#4\n\t"     //@ r4 |= (r6 & 0x02000200) >> 4 (A2)        
        "and     r5, r6, r14, LSR#8\n\t"
        "orr     r4, r5,      LSR#1\n\t"     //@ r4 |= (r6 & 0x00800080) >> 1 (A4)        

        "and     r5, r6, r14, LSR#10\n\t"
        "orr     r4, r5,      LSR#5\n\t"     //@ r4 |= (r6 & 0x00200020) >> 5 (B1)        
        "and     r5, r6, r14, LSR#12\n\t"
        "orr     r4, r5,      LSR#2\n\t"     //@ r4 |= (r6 & 0x00080008) >> 2 (B2)        
        "and     r5, r6, r14, LSR#14\n\t"
        "orr     r4, r5,      LSL#1\n\t"     //@ r4 |= (r6 & 0x00020002) << 1 (B4)        

        "and     r5, r6, r14, LSR#3\n\t"
        "orr     r4, r5\n\t"                 //@ r4 |= (r6 & 0x10001000) << 0 (C1)        
        "and     r5, r6, r14, LSR#5\n\t"
        "orr     r4, r5,      LSL#3\n\t"     //@ r4 |= (r6 & 0x04000400) << 3 (C2)        
        "and     r5, r6, r14, LSR#7\n\t"
        "orr     r4, r5,      LSL#6\n\t"     //@ r4 |= (r6 & 0x01000100) << 6 (C4)        

        "and     r5, r6, r14, LSR#11\n\t"
        "orr     r4, r5,      LSL#4\n\t"     //@ r4 |= (r6 & 0x00100010) << 4 (D1)        
        "and     r5, r6, r14, LSR#13\n\t"
        "orr     r4, r5,      LSL#7\n\t"     //@ r4 |= (r6 & 0x00040004) << 7 (D2)        
        "and     r5, r6, r14, LSR#15\n\t"
        "orr     r4, r5,      LSL#10\n\t"    //@ r4 |= (r6 & 0x00010001) << 10 (D4)        
 
        "and     r5, r7, r14, LSR#5\n\t"
        "orr     r4, r5,      LSL#5\n\t"     //@ r4 |= (r7 & 0x04000400) << 5  (SPI)        
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
//    We've got one or two valid ModeA's
//
        "mov     r10, #0\n\t"                // r10 = 0
    "Lane1:\n\t"
        "tst     r3,    #0x00008000\n\t"     //@ if (Lane1 not Valid)....
        "beq     Lane1End\n\t"               //@ ...goto Lane1End

        "pkhbt   r11, r4, r10\n\t"           //@ r11.h  = r10.h  Lane1 0x0000
                                             //@ r11.l  = r4.l   Lane1 uModeA
        "and     r10, r3, #0x000000FF\n\t"   //@ r10   = Bit count = r6.l
        "stm     r1, { r8, r10, r11 }\n\t"   //@ Store Lane1sigStrength, Bit Count and Lane1Word
    "Lane1End:\n\t"

    "Lane2:\n\t"
        "tst     r3,    #0x80000000\n\t"     //@ if (Lane2 not Valid)....
        "beq     Lane2End\n\t"               //@ ...goto Lane2End

        "pkhtb   r11, r10, r4,ASR#16\n\t"    //@ r11.h  = r10.h  Lane2 0x0000
                                             //@ r11.l  = r10.l  Lane2 uModeA
        "lsr     r10, r3, #16\n\t"
        "and     r10, r10, #0x000000FF\n\t"  //@ r10   = Bit count = r6.h
        "stm     r2, { r9, r10, r11 }\n\t"   //@ Store Lane2sigStrength, Bit Count and Lane2Word
    "Lane2End:\n\t"
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
        "and     r3, r3, r14\n\t"
        "mov     r0, r3\n\t"
        "b       Done\n\t" 
//      "pop    { r4 - r12, pc }\n\t"        //@ Pop the registers that need to be preserved from the stackand return
//
//----------------------------------------------------------------------------------------------------------------------------------------
//
    "Exit:\n\t" 
        "mov     r3, #0\n\t" 
        "mov     r0, #0\n\t" 

    "Done:\n\t" 
        "pop    { r4 - r12, lr }\n\t"        //@ Pop the registers that need to be preserved from the stack (no return for inline asm)
//	    "pop    { r4 - r12, pc }\n\t"        //@ Pop the registers that need to be preserved from the stack and return

      : "=r" (puAF) // Outputs in r0
      : "g" (puAF), "g" (pLane)
      : "r0","r1","r2","r3","cc","memory"
	);
}

static tMessageRaw rmLane[2];

static void detectBufferModeA(uint16_t* puAF, uint64_t lTime) {

	uint32_t j, uLanes;
	tMessageRaw** pQ;

	memset(&rmLane, 0, sizeof(tMessageRaw) * 2);

	// Find the end of any current queue
	pQ = &Modes.pModeA_List;
    while (*pQ) {
		pQ = & (*pQ)->pNext;
	}

	for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j += 2) {

		if ( (uLanes = detectModeAArmSIMD(puAF, rmLane))) { // We have found a new ModeA/C in the data   

			if ((uLanes & 0x00008000)) {
				rmLane[0].uMsgType = 32;
        		rmLane[0].lTime = lTime;
				pQ = enQueueModeA(&rmLane[0], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

			if ((uLanes & 0x80000000)) {
				rmLane[1].uMsgType = 32;
        		rmLane[1].lTime = lTime+1;
				pQ = enQueueModeA(&rmLane[1], pQ);
            } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
                pQ = postModeAQueue();
            }

        } else if (Modes.uModeATimeout > 2){
            Modes.uModeATimeout -= 2;

		} else if (Modes.uModeATimeout) {
            Modes.uModeATimeout = 0;
            pQ = postModeAQueue();
        }

		lTime+=2;
		puAF += 2;
	}
}

#else

uint32_t iModeABitTable[18] = {
0x80000000, //  0 DL2_0  F1 = 0  Set bit 31 if we see this high (we should)
0x00000010, //  1 DL2_1  C1
0x00001000, //  2 DL2_2  A1
0x00000020, //  3 DL2_3  C2
0x00002000, //  4 DL2_4  A2
0x00000040, //  5 DL2_5  C4
0x00004000, //  6 DL2_6  A4
0x20000000, //  7 DL2_7  xx = 0  Set bit 29 if we see this high
0x00000100, //  8 DL2_8  B1 
0x00000001, //  9 DL2_9  D1
0x00000200, // 10 DL2_10 B2
0x00000002, // 11 DL2_11 D2
0x00000400, // 12 DL2_12 B4
0x00000004, // 13 DL2_13 D4
0x40000000, // 14 DL2_14 F2 = 0  Set bit 30 if we see this high (we should)
0x20000000, // 15 DL1_1  xx = 0  Set bit 28 if we see this high
0x10000000, // 16 DL1_2  xx = 0  Set bit 27 if we see this high
0x00000080, // 17 DL1_3  SPI
};

uint32_t detectModeA(uint16_t* puAF, tMessageRaw* pLane) {
    int i;
    uint32_t uModeA;
    uint32_t nBits;
    uint32_t uSig;
    uint16_t uMin, uMax, uSample, uLevelH;//, uLevelL;
    uint16_t uDL1Bits, uDL2Bits, uDL4Bits;
    uint16_t uGarbled;
/*
    uint64_t lSum   = 0;
    uint64_t lSumSq = 0;
    uint32_t uSum;
    uint32_t uSumSq;
    uint32_t uVar;
    uint32_t uSd;
    uint32_t uRootSumSq;
*/
    // iMax = min (F1,F2);      -- This is the lower of F1 and F2 
    // iMin = max (Z7,Z15,Z16); -- This is the higher of Z7, F15 and F16
    uMax = (puAF[377] < puAF[783]) ? puAF[377] : puAF[783];
    uMin = (puAF[580] > puAF[812]) ? puAF[580] : puAF[812];
    uMin = (uMin      > puAF[841]) ? uMin      : puAF[841];

    // Return if insufficient difference between minimum and maximum.
    if ((uMax - uMin) < MODEAC_MSG_SQUELCH_LEVEL)
        {return (0);}
/*
//    if (uMax <= (uMin << 1))
//        {return (0);}

    for (i = 0; i < 1024; i++){
        lSum   +=  puAF[i];
        lSumSq += (puAF[i] * puAF[i]);
    }
    uSum       = lSum   / 1024;
    uSumSq     = lSumSq / 1024;
    uRootSumSq = sqrt(uSumSq);

    //if (uMax <= (uRootSumSq << 1))
    //    {return (0);}

	// Variance is the sum of the squares minus the square of the sums
	uVar = uSumSq - (uSum * uSum);
	// Standard Deviation is the square root of the variance
	uSd  = (uint32_t) sqrt(uVar);
*/
    // If we get here we have a potential F1,F2 detection.
    // Calculate the decision level for bit detection as half way between uMin and uMax
    uLevelH = (uMin + uMax) / 2;
//  uLevelH = uMin + (2 * (uMax - uMin) / 3);
//  uLevelL = uMin + (    (uMax - uMin) / 3);
    nBits = 0;
    uModeA = 0;

    // Calculate the DL4 shift register (bits 0-12) from puAF[0]..puAF[348]
    uDL4Bits = 0;
    for (i = 0; i < 13; i++) {
        uSample = *puAF; puAF += 29;
        uDL4Bits = uDL4Bits << 1;
        if (uSample >= uLevelH) { // its a one
            uDL4Bits |= 1;
        }
    }
    // Skip F1 (puAF[377]) because we already know it's a '1'
    uSig = *puAF;
    puAF += 29;

    // Calculate the DL2 shift register, the Mode A/C bits, and signal strength 
    // puAF[406]..puAF[551] (bits C1-A4)
    uDL2Bits = 0;
    for (i = 1; i < 7; i++) {
        uSample = *puAF; puAF += 29;
        uDL2Bits = uDL2Bits << 1;
        if (uSample >= uLevelH) { // its a one
            uDL2Bits |= 1;
            uModeA |= iModeABitTable[i]; // or in the correct bit
            nBits++;
            uSig += uSample;
//      } else if (uSample >= uLevelL) { // its questionable
//          return(0);
        }
    }
    // Skip Z7 (puAF[580]) because we already know it's a '0'
    puAF += 29;
    uDL2Bits = uDL2Bits << 1;

    // puAF[609]..puAF[754] (bits C1-A4)
    for (i = 8; i < 14; i++) {
        uSample = *puAF; puAF += 29;
        uDL2Bits = uDL2Bits << 1;
        if (uSample >= uLevelH) { // its a one
            uDL2Bits |= 1;
            uModeA |= iModeABitTable[i]; // or in the correct bit
            nBits++;
            uSig += uSample;
//        } else if (uSample >= uLevelL) { // its questionable
//            return(0);
        }
    }

    // Skip F2 (puAF[783]), Z15 (puAF[812]) and Z16 (puAF[841]) because we already know they're '1','0','0'
    uSig += *puAF;
    puAF += 87;

    // Calculate the SPI bit puAF[870]
    uDL1Bits = 0;
    uSample = *puAF; puAF += 29;
    if (uSample >= uLevelH) { // its a one
        uDL1Bits |= 1;
        uModeA |= iModeABitTable[17]; // or in the correct bit
        nBits++;
        uSig += uSample;
    }

    // Calculate the rest of the DL1 shift register 
    // puAF[899]..puAF[1160]
    for (i = 0; i < 10; i++) {
        uSample = *puAF; puAF += 29;
        uDL1Bits = uDL1Bits << 1;
        if (uSample >= uLevelH) { // its a one
            uDL1Bits |= 1;
        }
    }

    if ((uGarbled = (((uDL1Bits | uDL4Bits) & uDL2Bits) & 0x1BFF))) {
        return (0);
    }

    pLane->signalLevel32 = uSig;
    pLane->uMsgBits      = nBits;
    pLane->ucMsg[0]      = (unsigned char) (uModeA >> 8);
    pLane->ucMsg[1]      = (unsigned char) (uModeA);
    return (1);
}

static tMessageRaw rmLane;

static void detectBufferModeA(uint16_t* puAF, uint64_t lTime) {

	uint32_t j;
	tMessageRaw** pQ;

	memset(&rmLane, 0, sizeof(tMessageRaw));

	// Find the end of any current queue
	pQ = &Modes.pModeA_List;
    while (*pQ) {
		pQ = & (*pQ)->pNext;
	}

    for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j++) {
/*
        if (lTime == 0x0F957773) {
            FILE* pFile = fopen("ModeA.csv", "w");
            if (pFile) {
                int k;
                for (k = 0; k < 1024; k++) {
                    fprintf(pFile, "%d\n", puAF[j + k]);
                }
                fclose(pFile);
            }
            __asm nop;
        }
*/
		if ((detectModeA(&puAF[j], &rmLane))) { // We have found a new ModeA in the data   
            rmLane.lTime    = lTime;
            rmLane.uMsgType = 32;
			pQ = enQueueModeA(&rmLane, pQ);
        } else if ((Modes.uModeATimeout) && (!(--Modes.uModeATimeout))) {
            pQ = postModeAQueue();
        }
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
void* ModeA_threadproc(void* arg) {

	uint32_t uAF_Out;
    UNUSED (arg);

#ifdef _WIN32
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif

	// Keep going until USB or program terminates
	while (!Modes.exit) {

        // Wait for someone to set Modes.ModeA_Go_cond
		pthread_cond_wait(&Modes.ModeA_Go_cond, &Modes.ModeA_Go_mutex);
		// We own the ModeA_mutex at this point

		// If the program is terminating break
		if (Modes.exit) {
			break;
		}

		// Get the buffer number to process
		uAF_Out = Modes.uAF_Out & (MODES_ASYNC_BUF_NUMBER - 1);

		// Process the new buffer for ModeA
		detectBufferModeA(Modes.AF_Fifo[uAF_Out].pFifo, Modes.AF_Fifo[uAF_Out].lTime);

        //
        // If there is no leftover from this buffer into the next buffer, send a null ModeA
        // packet to the decoder thread with the time of the end of this (ModeA) processing.
        //
        if (Modes.pModeA_List == NULL) {
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
            // And Unlock the Decode mutex
            pthread_mutex_unlock(&Modes.Decode_mutex);
        }

    	// Lock ModeA_Done_mutex. We won't be granted the lock until the AF_Thread has finished 
        // ModeS processing, and is waiting for us to signal it that we've finished ModeA processing.
	    pthread_mutex_lock(&Modes.ModeA_Done_mutex);
        // Signal the AF Thread that we're done
        pthread_cond_signal(&Modes.ModeA_Done_cond);
    	// Unlock the ModeA_Done_mutex
	    pthread_mutex_unlock(&Modes.ModeA_Done_mutex);
	}

  	// Unlock the ModeA_Go_mutex before we exit
    pthread_mutex_unlock(&Modes.ModeA_Go_mutex);

#ifdef _WIN32
	return (NULL);
#else
	pthread_exit(NULL);
#endif
}
//
//=========================================================================
//
