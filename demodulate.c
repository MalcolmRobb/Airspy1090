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
// Turn 12 bit unsigned samples pointed by pIfIn into a
// demodulated magnitude vector pointed by pAfOut.
//
#ifdef _ARMASM
//
// pIfIn, pAfOut, pIF_iDelay and pAF_iDelay ***MUST*** all be DWORD aligned. If they aren't this code will fault. ARM doesn't support non word aligned loads/stores
//
//layout asm
//break IfDemodulateArmAsm
//break *0x17b2c
//break *0x17adc
//info registers q0
void IfDemodulateArmAsm(volatile unsigned short* pIfIn, volatile unsigned short* pAfOut, volatile unsigned short* pIF_iDelay, volatile unsigned short* pAF_iDelay) {

	__asm volatile(
    ".arch armv7-a\n\t"
    ".fpu  neon\n\t"

	"push   { r2-r12, lr }\n\t"    //@ Push the registers that need to be preserved onto the stack
 
    "ldm    r2, { r4,r5 }\n\t"     //@ Pick up uIF_In from last time
	                               //@ r5.h = uIF[-1], r5.l = uIF[-2]
                                   //@ r4.h = uIF[-3], r4.l = uIF[-4]

	"ldm    r3, { r8 - r12 }\n\t"  //@ Pick up uAF_In from last time
		                           //@ r12.h = uAF[-1], r12.l = uAF[-2]
		                           //@ r11.h = uAF[-3], r11.l = uAF[-4]
		                           //@ r10.h = uAF[-5], r10.l = uAF[-6]
		                           //@ r9.h = uAF[-7], r9.l = uAF[-8]
		                           //@ r8.h = uAF[-9], r8.l = uAF[-10](unused)


	//# uIF values are 12 bit unsigned(0x0000 - 0x0FFFF)
	//# The IF HPF transfer function is(4 * uIF[-2]) - (uIF[-4] + uIF[-3] + uIF[-1] + uIF[0])
	//# The range of IF output values are therefore 0x3FFC..0xC004, so cannot overflow 16 bit maths
	//# The IF is then rectified, so the final output is in the range 0x3FFC..0x0000
	"uadd16  r7, r4, r5\n\t"          //@ r7.h = (uIF[-3] + uIF[-1])
                                      //@ r7.l = (uIF[-4] + uIF[-2])

	"add     r7, r7, r7, ROR#16\n\t"  //@ r7.h = uIF[-4] + uIF[-3] + uIF[-2] + uIF[-1]
                                      //@ r7.l = uIF[-4] + uIF[-3] + uIF[-2] + uIF[-1]

	"sub     r7, r4, LSL#16\n\t"      //@ r7.h -= uIF[-4] = uIF[-3] + uIF[-2] + uIF[-1]
		                              //@ r7.l -= 0 = uIF[-4] + uIF[-3] + uIF[-2] + uIF[-1]

	//# uAF input values are the rectified uIF output values from the previous HPF, and therefore have
	//# the 14 bit unsigned range 0x3FFC..0x0000
	//# The AF LPF transfer function is uAF[-9] + uAF[-8] + uAF[-7] + uAF[-6] + uAF[5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0]
	//# This has a range of 0x27FD8..0x00000, which can overflow 16 bit maths.However, it is safe to add up to four
	//# uAF[] values(maximum = 0xFFF0)
	"uadd16  	r3, r12, r11\n\t"        //@ r3.h = uAF[-3] + uAF[-1](uint16_t safe)
		                                 //@ r3.l = uAF[-4] + uAF[-2](uint16_t safe)

	"uadd16  	r3, r3, r10\n\t"         //@ r3.h = uAF[-5] + uAF[-3] + uAF[-1](uint16_t safe)
		                                 //@ r3.l = uAF[-6] + uAF[-4] + uAF[-2](uint16_t safe)

	"uadd16  	r3, r3, r9\n\t"          //@ r3.h = uAF[-7] + uAF[-5] + uAF[-3] + uAF[-1](uint16_t safe)
	                                     //@ r3.l = uAF[-8] + uAF[-6] + uAF[-4] + uAF[-2](uint16_t safe)

	"uxth    	r2, r8, ROR#16\n\t"      //@ r2.h = 0x0000:AF[-9]
	"uxtah   	r2, r2, r3\n\t"          //@ r2 = uAF[-9] + uAF[-8] + uAF[-6] + uAF[-4] + uAF[-2]
	"uxtah   	r2, r2, r3, ROR#16\n\t"  //@ r2 = uAF[-9] + uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1]

	"mov     	r3, #65536\n\t"          //@ r3 = loop counter = MODES_ASYNC_BUF_SAMPLES / 2
    //"vmov.U32 q0, #0\n\t"              //@ d0 = d1 = 0
    //"vmov.U32	q1, #0\n\t"              //@ d2 = d3 = 0

"firLoop1:\n\t"
	"ldr     	r6, [r0],#4\n\t"         //@ r6 = *r0++    Pick up the next two uIF input values r6.h = uIF[1], r6.l = uIF[0]

	"lsl     	r14, r5, #2\n\t"         //@ r14.h = uIF[-1] * 4
		                                 //@ r14.l = uIF[-2] * 4
	"uadd16  	r14, r14, r5\n\t"        //@ r14.h = uIF[-1] * 5
                                         //@ r14.l = uIF[-2] * 5

	"uadd16  	r7, r7, r6\n\t"          //@ r7.h += uIF[1] =           uIF[-3] + uIF[-2] + uIF[-1]          + uIF[1]
		                                 //@ r7.l += uIF[0] = uIF[-4] + uIF[-3] + uIF[-2] + uIF[-1] + uIF[0]

	"add     	r7, r7, r6, LSL#16\n\t"  //@ r7.h += uIF[0] =           uIF[-3] + uIF[-2] + uIF[-1] + uIF[0] + uIF[1]
                                         //@ r7.l += uIF[0] = uIF[-4] + uIF[-3] + uIF[-2] + uIF[-1] + uIF[0]

	//# Do filtering for uIF
	"usub16  	r14, r14, r7\n\t"        //@ r14h = (5 * uIF[-1]) - (uIF[-3] + uIF[-2] + uIF[-1] + uIF[0] + uIF[1])
                                         //@ r14l = (5 * uIF[-2]) - (uIF[-4] + uIF[-3] + uIF[-2] + uIF[-1] + uIF[0])

	//# Update r7 ready for next loop
	"usub16		r7, r7, r4\n\t"          //@ r7.h -= uIF[-3] =           uIF[-2] + uIF[-1] + uIF[0] + uIF[1]
					                     //@ r7.l -= uIF[-4] = uIF[-3] + uIF[-2] + uIF[-1] + uIF[0]
	"pkhtb		r7, r7, r7, ASR#16\n\t"  //@ r7.h            =           uIF[-2] + uIF[-1] + uIF[0] + uIF[1]
						                 //@ r7.l            =           uIF[-2] + uIF[-1] + uIF[0] + uIF[1]
	"sub		r7, r7, r5, LSL#16\n\t"  //@ r7.h -= uIF[2]  =                     uIF[-1] + uIF[0] + uIF[1]
					                     //@ r7.l -=      0  =           uIF[-2] + uIF[-1] + uIF[0] + uIF[1]

	//# Shift uIF pipeline ready for next loop.Also frees r6 for re - use
	"mov		r4, r5\n\t"              //@ r4.h = uIF[-1], r4.l = uIF[-2]
	"mov		r5, r6\n\t"              //@ r5.h = uIF[ 1], r5.l = uIF[ 0]

	//# Do ABS operation on r14.l
	"sxth		r6, r14\n\t"             //@ r6 = r14l = (5 * uIF[-2]) - (uIF[-4] + uIF[-3] + uIF[-2] + uIF[-1] + uIF[0])
	"rsbs		r6, r6, #0\n\t"          //@ r6 = 0 - r14l, Carry set if no borrow, Carry clear if borrow
	"pkhtbpl	r14, r14, r6\n\t"        //@ if ((0-r14.l) >=0) r14.l = r6.l

	//# Do ABS operation on r14.h
	"sxth		r6, r14, ROR#16\n\t"     //@ r6 = r14h = (5 * uIF[-1]) - (uIF[-3] + uIF[-2] + uIF[-1] + uIF[0] + uIF[1])
	"rsbs		r6, r6, #0\n\t"          //@ r6 = 0 - r14h
	"pkhbtpl	r14, r14, r6, LSL#16\n\t" //@ if ((0-r14.h) >=0) r14.h = r4.l

	//# Do filtering for uAF[0].At this point r2 = uAF[-9] + uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1]
	"uxtah		r2, r2, r14\n\t"         //@ r2 += uAF[0] = uAF[-9] + uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0]
	"usat		r6, #16, r2\n\t"         //@ r6 = (r2 > 0xFFFF) ? 0xFFFF : r2
	//"vmov.U32	d3[0], r2\n\t"           //@ d3 = uAF[-9] + uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0]
	//"vadd.U32	d2, d2, d3\n\t"          //@ d2 +=  Sample 
	//"vmlal.U32	q0, d3, d3\n\t"      //@ q0 += (Sample*Sample)

	//# Do filtering for uAF[1].At this point r2 = uAF[-9] + uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0]
	"sub		r2, r2, r8, LSR#16\n\t"  //@ r2 -= uAF[-9] = uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0]

	//# At this point we're done with r8. Use it to saturate r2 to 16 bits      
	"uxtah		r2, r2, r14, ROR#16\n\t" //@ r2 += uAF[1] = uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0] + uAF[1]
	"usat		r8, #16, r2\n\t"         //@ r8 = (r2 > 0xFFFF) ? 0xFFFF : r2
	//"vmov.U32	d3[0], r2\n\t"           //@ d3 = uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0] + uAF[1]
	//"vadd.U32	d2, d2, d3\n\t"          //@ d2 +=  Sample 
	//"vmlal.U32	q0, d3, d3\n\t"      //@ q0 += (Sample*Sample)

	//# assemble a Word Write output oAF[1]:oAF[0]
	"pkhbt		r6, r6, r8, LSL#16\n\t"  //@ r6.h = r8.l = sat(uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0] + uAF[1])
                                         //@ r6.l = sat(uAF[-9] + uAF[-8] + uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0])

	//# Write the latest two output words
	"str		r6, [r1], #4\n\t"        //@ *r1++ = r6

	//# Done Filtering, prepare for next iteration
	"uxth		r8, r9\n\t"              //@ r8 = r9.l = uAF[-8]
	"sub		r2, r2, r8\n\t"          //@ r2 -= uAF[-8] = uAF[-7] + uAF[-6] + uAF[-5] + uAF[-4] + uAF[-3] + uAF[-2] + uAF[-1] + uAF[0] + uAF[1]

	//# shift uAF pipeline ready for next loop.
	"mov		r8, r9\n\t"              //@ r8.h = uAF[-7], r8.l = uAF[-8]
	"mov		r9, r10\n\t"             //@ r9.h = uAF[-5], r9.l = uAF[-6]
	"mov		r10, r11\n\t"            //@ r10.h = uAF[-3], r10.l = uAF[-4]
	"mov		r11, r12\n\t"            //@ r11.h = uAF[-1], r11.l = uAF[-2]
	"mov		r12, r14\n\t"            //@ r12.h = uAF[1], r12.l = uAF[0]

	"subs		r3, r3, #1\n\t"          //@ decrement the word count by 1
	"bne		firLoop1\n\t"            //@ loop until word count is zero

	"pop		{ r2 - r3 }\n\t"         //@ Retrieve r2 and r3 from the stack

	"stm		r2, { r4, r5 }\n\t"      //@ save the final  4 uIF values ready for next time
	"stm		r3, { r8 - r12 }\n\t"    //@ save the final 10 uAF values ready for next time

	// d2 = Sum   = sum of the Samples
	// q0 = SumSq = sum of the Squares
	//"vshr.U64	d2, d2, #17\n\t"		 //@ d2 = d2 / MODES_ASYNC_BUF_SAMPLES
	//"vshr.U64	q0, q0, #17\n\t"		 //@ q0 = q0 / MODES_ASYNC_BUF_SAMPLES

	//"vmull.U32  q1, d2, d2\n\t"		 //@ q1 = (d2*d2) = square of the sums
	//"vsub.U64   q1, q0, q1\n\t"	     //@ q1 = sum of the squares - square of the sums 
	//"vmov       r2, r3, d2\n\t"

	//"mov        r0, r3\n\t"
	"pop		{ r4 - r12, lr }\n\t"    //@ Pop the registers that need to be preserved from the stack (no return for inline asm)
//	"pop		{ r4 - r12, pc }\n\t"    //@ Pop the registers that need to be preserved from the stack and return

      : "=r" (pIfIn) // Outputs in r0
      : "g" (pIfIn), "g" (pAfOut), "g" (pIF_iDelay), "g" (pAF_iDelay)
      : "r0","r1","r2","r3","cc","memory"
	);
}
#else
#define kIF_Prev ( 0)
#define kIF_Mid  ( 3)
#define kIF_Next ( 5)
#define kIF_Size ( 5)
#define kAF_Prev ( 0)
#define kAF_Next (10)
#define kAF_Size (10)

static void IfDemodulateC(uint16_t* pIfIn, uint16_t* pAfOut) {
	uint32_t   j;

//	uint64_t lAF_Sum   = 0;
//	uint64_t lAF_SumSq = 0;
//	uint32_t uAF_Sum;
//	uint32_t uAF_SumSq;
//	uint32_t uAF_Var;
//	uint32_t uAF_Sd;
//	uint32_t uAF_NF;

	int16_t* iIF_Root = (int16_t*)Modes.uIF_iDelay;
	int16_t* iAF_Root = (int16_t*)Modes.uAF_iDelay;

	int32_t  iIF_Out = Modes.iIF_Out;                 // This is the current calculation of the DC component of the IF input stream
	int16_t* iIF_Delay = iIF_Root;                    // This is the delay line for calculating the IF HPF

	int16_t  iAF_In;
	int32_t  iAF_Out = Modes.iAF_Out;                 // This is the current calculation of the AF LPF
	int16_t* iAF_Delay = iAF_Root;                    // This is the delay line for calculating the AF LPF

	for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j++) {

		// Run the moving average IF HPF for this step
		iIF_Out -= iIF_Delay[kIF_Prev];                         // Subtract the oldest sample
		iIF_Out += (iIF_Delay[kIF_Next] = (int16_t)(*pIfIn++)); // Add in the newest sample

		// Subtract the IF DC from the IF mid input sample and rectify
		iAF_In = abs((iIF_Delay[kIF_Mid] * kIF_Size) - iIF_Out);

		// Run the moving average AF LPF for this step
		iAF_Out -= iAF_Delay[kAF_Prev];           // Subtract the oldest sample
		iAF_Out += (iAF_Delay[kAF_Next] = iAF_In); // Add in the newest sample

		// Output the filtered sample
		*pAfOut++ = (uint16_t)iAF_Out;

		//lAF_Sum   += iAF_Out;
		//lAF_SumSq += (iAF_Out * iAF_Out);

		iIF_Delay++;
		iAF_Delay++;
	}

	//uAF_SumSq = (uint32_t)(lAF_SumSq / MODES_ASYNC_BUF_SAMPLES);
	//uAF_Sum   = (uint32_t)(lAF_Sum   / MODES_ASYNC_BUF_SAMPLES);

	// Variance is the sum of the squares minus the square of the sums
	//uAF_Var = uAF_SumSq - (uAF_Sum * uAF_Sum);
	// Standard Deviation is the square root of the variance
	//uAF_Sd  = (uint32_t) sqrt(uAF_Var);

    // Set the noise floor to the RMS of the input.
	//uAF_NF = (uint32_t) sqrt (uAF_SumSq);

    // Set the noise floor to the AF mean plus one standard deviation.
    //uAF_NF = uAF_Sum + uAF_Sd;

	memcpy(iIF_Root, iIF_Delay, kIF_Size * sizeof(uint16_t));
	memcpy(iAF_Root, iAF_Delay, kAF_Size * sizeof(uint16_t));

	// Save the final state ready for next time.
	Modes.iIF_Out = (int16_t)iIF_Out;
	Modes.iAF_Out = (int16_t)iAF_Out;

	//return (uAF_NF);
}
#endif
//
//=========================================================================
//
static void IfDemodulate(uint16_t* pIfIn, uint16_t* pAfOut) {

    //FILE* hFile;

	// Move the end of the previous output buffer to the beginning of this one
	memcpy(pAfOut, Modes.uAF_oDelay, (MODES_LONG_FRAME_SAMPLES * sizeof(uint16_t)));

#ifdef _ARMASM
	// Optimised ARM Assembler version of IfDemodulate 
	//Modes.uAF_NF = 
    IfDemodulateArmAsm(pIfIn, &pAfOut[MODES_LONG_FRAME_SAMPLES], Modes.uIF_iDelay, Modes.uAF_iDelay);
#else
	// Optimised C version of IfDemodulate 
	//Modes.uAF_NF = 
    IfDemodulateC(pIfIn, &pAfOut[MODES_LONG_FRAME_SAMPLES]);
#endif
/*
    if ((hFile = fopen("neon1090_u17.txt", "a"))) {
        fprintf(hFile, "%08X\n", Modes.uAF_NF);
        fclose(hFile);
    }
*/
	// Move the end of the current output buffer to the output delay line ready for next time
	memcpy(Modes.uAF_oDelay, &pAfOut[MODES_ASYNC_BUF_SAMPLES], (MODES_LONG_FRAME_SAMPLES * sizeof(uint16_t)));
}
//
//=========================================================================
//
// This thread takes input buffers from libusb (airspy_libusb_transfer_callback),
// bandpass/highpass filters, rectifies and low pass filters them into output buffers
// to be used by AF_threadproc.
//
// Modes.pIF_Fifo[Modes.uIF_Out] is the next input buffer to process
// Modes.pIF_Fifo[Modes.uIF_In] is the newest input buffer added to the FIFO
//
// The number of buffers available in the Fifo is (Modes.uIF_In - Modes.uIF_Out)
//
// Modes.pAF_Fifo[Modes.uAF_In] is the output buffer to put data into
//
void* IF_threadproc(void* arg) {

	int uIF_Out;
	int uAF_In;
    UNUSED (arg);

#ifdef _WIN32
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif

	// Keep going untill USB or program terminates
	while (!Modes.exit) {

		// While there are no buffers to process, release IF_mutex and wait for IF_cond. 
		// When IF_cond occurs, re-aquire the IF_mutex.
		while ((Modes.uIF_In == Modes.uIF_Out) && (!Modes.exit)) {
			pthread_cond_wait(&Modes.IF_cond, &Modes.IF_mutex);
		}
		// We own the IF_mutex at this point, and there is one of more IF buffer to process

		// If the program is terminating break
		if (Modes.exit) {
			break;
		}

		// Get the next input buffer number to process
		uIF_Out = Modes.uIF_Out & (MODES_ASYNC_BUF_NUMBER - 1);
		// Don't change uIF_Out yet - we haven't processed the data
		// Safe to release the IF mutex now that we're done with Modes.uIF_Out
		pthread_mutex_unlock(&Modes.IF_mutex);

		// Lock the AF mutex so we can access the AF Fifo
		// This can block if the AF_threadproc FIFO is more than half full
		pthread_mutex_lock(&Modes.AF_mutex);

		// If there is some space in the AF_Fifo
		if ((Modes.uAF_In - Modes.uAF_Out) < MODES_ASYNC_BUF_NUMBER) {

			uAF_In = Modes.uAF_In & (MODES_ASYNC_BUF_NUMBER - 1);

			// Don't change uAF_In yet - we haven't processed the data
			// Safe to release the IF mutex now that we're done with Modes.uAF_In
			pthread_mutex_unlock(&Modes.AF_mutex);

			// Process the new buffer from Modes.pIF_Fifo[uIF_Out] to Modes.pAF_Fifo[uAF_In]
			IfDemodulate(Modes.IF_Fifo[uIF_Out].pFifo, Modes.AF_Fifo[uAF_In].pFifo);

			// Lock the AF mutex, and increase the uAF_In pointer
			// This can block if the AF_threadproc FIFO is more than half full
			pthread_mutex_lock(&Modes.AF_mutex);
			Modes.uAF_In++;

			// re-aqcquire the IF mutex so we can change Modes.uIF_Out
			pthread_mutex_lock(&Modes.IF_mutex);
			Modes.uIF_Out++;

			// Copy the number of Lost buffers and the time from IF_Fifo to AF Fifo
			Modes.AF_Fifo[uAF_In].uLost = Modes.IF_Fifo[uIF_Out].uLost;
			Modes.AF_Fifo[uAF_In].lTime = Modes.IF_Fifo[uIF_Out].lTime;

			// Unlock the IF Mutex
			pthread_mutex_unlock(&Modes.IF_mutex);
		}

		// Signal the Af thread there is something to do. It won't
		// execute yet because we still own the AF_mutex
		pthread_cond_signal(&Modes.AF_cond);

		// Release the Af mutex so the AF_Thread can execute
		pthread_mutex_unlock(&Modes.AF_mutex);

		// Lock the IF mutex ready for the next loop
		pthread_mutex_lock(&Modes.IF_mutex);
	}

	pthread_mutex_unlock(&Modes.IF_mutex);

#ifdef _WIN32
	return (NULL);
#else
	pthread_exit(NULL);
#endif
}
//
//=========================================================================
//
