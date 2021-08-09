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
static tMessageRaw* deQueueMessage(tMessageRaw* rmIn) {
    tMessageRaw* rmOut;
    tMessageRaw* rmTmp;
    tModesMessage  mm;

    // Normalise the signal strength for each of the remaining decodes. 
    // We may need to use the signal strength to decide between frames, so 
    // they need to be normalised (particularly the Mode A's)
    rmTmp = rmIn;
    while (rmTmp) {
        if (rmTmp->uMsgType == 32) {
            // For ModeA normalise the signal strength per bit set. 
            // Scale the strength up by 16 first to preserve some resolution.  
            // There must me at least 2 bits set (F1 & F2) so add 2 to rm->msgbits
            // to prevent divide by 0
            rmTmp->signalLevel32 = (rmTmp->signalLevel32 << 4) / (2 + rmTmp->uMsgBits);
            rmTmp->uMsgBits      = 16;
        }
        else {
            // For ModeS we measured signal strength over the entire message. Don't forget
            // to add 4 for the preamble samples, so round up and divide by 60 or 116.
            rmTmp->signalLevel32 = (rmTmp->signalLevel32 << 4) / (4 + rmTmp->uMsgBits);
        }
        rmTmp = rmTmp->pNext;
    }

    //
    // Count the number of different decodes, and chose the strongest one
    // of each type
    //
    rmOut = rmIn;
    rmIn = rmIn->pNext;
    rmOut->nCount = 1;
    rmOut->pNext = NULL;
    while (rmIn) {

        rmTmp = rmOut;

        while (rmTmp) {

            if ( (rmIn->uMsgBits == rmTmp->uMsgBits)
              && (!memcmp(rmIn->ucMsg, rmTmp->ucMsg, rmTmp->uMsgBits / 8))) {
                // This new frame is identical to an existing frame
                if (rmIn->signalLevel32 > rmTmp->signalLevel32) {
                    // If the new signal strength is greater than the existing one
                    rmTmp->signalLevel32 = rmIn->signalLevel32;
                    rmTmp->lTime         = rmIn->lTime;
                }
                rmTmp->nCount++;
                rmTmp = NULL;
                rmIn = rmIn->pNext;

            }
            else if ((rmTmp->uMsgBits == rmIn->uMsgBits)
                  && (rmTmp->uMsgBits == 56)
                  && (rmTmp->uMsgType == 11)
                  && (!memcmp(rmTmp->ucMsg, rmIn->ucMsg, 6))) {
                // This new frame is a DF11 that differs only in SI/II
                if (rmIn->signalLevel32 > rmTmp->signalLevel32) {
                    // If the new signal strength is greater than the existing one
                    rmTmp->signalLevel32 = rmIn->signalLevel32;
                    rmTmp->lTime         = rmIn->lTime;
                }
                // Kludge the message to SI/II=0 - we can't be sure which SI/II is the correct one
                rmTmp->ucMsg[6] ^= rmTmp->uCrc;
                rmTmp->uCrc = 0;
                rmTmp->nCount++;
                rmTmp = NULL;
                rmIn = rmIn->pNext;

            } else if (rmTmp->pNext) {
                // There is another frame in the list to compare to
                rmTmp = rmTmp->pNext;

            } else {
                // This new frame is different to the previous one(s). Yuk
                rmIn->nCount = 1;
                rmTmp = (rmTmp->pNext = rmIn);
                rmIn = rmIn->pNext;
                rmTmp = (rmTmp->pNext = NULL);
            }
        }
    }

    //
    // If there is more than one decode remaining, we need to decide 
    // which of the decoded frames is the *best* one. We have two pieces
    // of information - Signal Strength and Count.
    //
    while ((rmTmp = rmOut->pNext)){
        if        (rmTmp->nCount > rmOut->nCount) {
            // rmTmp has occured more often than the current rmOut.
            rmOut = rmTmp;

        } else if (rmTmp->nCount < rmOut->nCount) {
            // rmTmp has occured fewer times than the current rmOut.
            rmOut->pNext = rmTmp->pNext;

        } else if (rmTmp->signalLevel32 > rmOut->signalLevel32) {
            // rmTmp has occured the same number of times as the 
            // the current rmOut, but rmTmp is stronger
            rmOut = rmTmp;

        } else {
            // rmTmp has occured the same number of times as the 
            // the current rmOut, but rmTmp is weaker or equal
            rmOut->pNext = rmTmp->pNext;
        }
    }

    //
    // We have now decided which frame is the best candidate, so use it.
    //
    if ( (rmOut->uMsgType < 32)   // If it's a Mode S (so CRC safe hopefully!)
      || (validateModeA(rmOut)) ){

        memmove(&mm.rm, rmOut, sizeof(tMessageRaw));
        mm.bFlags = 0;
/*
//      if (mm.rm.lTime == 0x01C032FD) {
        if (mm.rm.lTime == 0x23581A26) {
            __asm nop;
        }
*/
        if (mm.rm.uMsgType == 32) {
            decodeModeA(&mm);
        } else {
            decodeModeS(&mm);
        }
/*
        if (mm.rm.uAddr == 0x0094ACDA) {
            __asm nop;
        }
*/		
        // Always track aircraft
        interactiveReceiveData(&mm);

        // In non-interactive non-quiet mode, display messages on standard output
        if (!Modes.interactive && !Modes.quiet) {
            interactiveListData(&mm);
        }

        // Feed output clients
        netQueueOutput(&mm);

        // Heartbeat not required whilst we're seeing real messages
        Modes.net_heartbeat_count = 0;
    }
    return (rmOut);
}
//
//=========================================================================
//
void* Decode_threadproc(void* arg) {

    uint32_t uModeSOut,  uModeAOut;
    uint64_t lModeSTime = 0;
    uint64_t lModeATime = 0;
    uint64_t lMaskTime  = 0;
    tMessageRaw* pModeA = NULL;
    tMessageRaw* pModeS = NULL;
    UNUSED (arg);

#ifdef _WIN32
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif

	// Keep going until program terminates
	while (!Modes.exit) {

		// While there is no data to process, release Decode_mutex and wait for Decode_cond. 
		// When Decode_cond occurs, re-aquire the Decode_mutex.
		while ( ( (Modes.uDecode_ModeS_Out == Modes.uDecode_ModeS_In)
               || (Modes.uDecode_ModeA_Out == Modes.uDecode_ModeA_In) )
             && (!Modes.exit)) {
			pthread_cond_wait(&Modes.Decode_cond, &Modes.Decode_mutex);
        }
		// We own the Decode_mutex at this point

		// If the program is terminating so break
		if (Modes.exit) {
			break;
		}

        // There should be something in both ModeA and ModeS FIFO's
        pModeS = pModeA = NULL;
        if (Modes.uDecode_ModeS_Out - Modes.uDecode_ModeS_In) {
     		// Get the next ModeS buffer number and time to process
            uModeSOut  = Modes.uDecode_ModeS_Out & (MODES_DECODE_FIFO_NUMBER - 1);
            pModeS = Modes.Decode_ModeS_Fifo[uModeSOut];
        }
        if (Modes.uDecode_ModeA_Out - Modes.uDecode_ModeA_In) {
  	    	// Get the next ModeA buffer number and time to process
            uModeAOut  = Modes.uDecode_ModeA_Out & (MODES_DECODE_FIFO_NUMBER - 1);
            pModeA = Modes.Decode_ModeA_Fifo[uModeAOut];
        }
        //
        // Once we've got both a ModeA and a ModeS entry we can decide which one is
        // earliest and deal with it first. This ensures that data is output in ascending
        // time order, rather than whichever thread is fastest to decode.
        // Note that entries with (signalLevel32 == 0) are null entries used to just keep
        // the ModeA and/or ModeS timestamps ticking up, and avoid excessive stalls if there
        // are no ModeA or ModeS frames being decoded in the current block.
        //
        if ((pModeA) && (pModeS)) {
            lModeSTime = pModeS->lTime;
            lModeATime = pModeA->lTime;

            if (lModeSTime <= lModeATime) {
                // ModeS Frame occurs before next ModeA Frame

                if (pModeS->signalLevel32) {
                    // Safe to release the mutex whilst we process the new buffer
                    pthread_mutex_unlock(&Modes.Decode_mutex);

                    // Process the new ModeS decode list
                    pModeS = deQueueMessage(pModeS);

                    // Mask any ModeA's that occur before the end of this ModeS frame
                    lMaskTime = lModeSTime + ((pModeS->uMsgBits + 16) * 20);

                    // re-aqcquire the Decode mutex so we can change Modes.uDecode_ModeS_Out
                    pthread_mutex_lock(&Modes.Decode_mutex);

                } else {
                    // This is a NULL buffer indicating the ModeS processing has reached 
                    // the end of a buffer and has no more Frames pending
                    netFlushClients(lModeSTime);
                }
                Modes.uDecode_ModeS_Out++;

            } else { 
                // ModeA Frame occurs before next ModeS Frame

                if ((pModeA->signalLevel32) && (lModeATime >= lMaskTime)) {
                    // Safe to release the mutex whilst we process the new buffer
                    pthread_mutex_unlock(&Modes.Decode_mutex);

                    // Process the new ModeA decode list
                    pModeA = deQueueMessage(pModeA);

                    // re-aqcquire the Decode mutex so we can change Modes.uDecode_ModeS_Out
                    pthread_mutex_lock(&Modes.Decode_mutex);
                }
                Modes.uDecode_ModeA_Out++;
            }
        }
	}

	// Unlock the Decode mutex before we terminate
	pthread_mutex_unlock(&Modes.Decode_mutex);
#ifdef _WIN32
	return (NULL);
#else
	pthread_exit(NULL);
#endif
}
//
//=========================================================================
//
