/*MIT License

Copyright(c) 2024 MSLM Electric / Osim Abdulhamidov

Permission is hereby granted, free of charge, to any person obtaining a copy
of this softwareand associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright noticeand this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/


#include "BitLogger.h"
#include <string.h> //only for memset

void InitBitLoggerList(BitLoggerList_t* BitLogger)
{
	memset(BitLogger->cntr32bit, 0, sizeof(BitLogger->cntr32bit));
	memset(BitLogger->_tempCntr, 0, sizeof(BitLogger->_tempCntr));
	//You may init here the simple timer and Launch
	return;
}

uint32_t BitLoggerList(BitLoggerList_t *BitLogger)  //InspectBitLoggerList //? mb on here sounds better?
{
	//if(IsTimerWPRinging(BitLogger->BugScannerTimer)){
	//RestartTimerWP(BitLogger->BugScannerTimer);
	for (uint8_t u = 0; u < 32; u++) {
		if(BitLogger->cntr32bit[u] != BitLogger->_tempCntr[u])
			BitLogger->Q32bit |= BIT(u);
		else
			BitLogger->Q32bit &= ~BIT(u);
		BitLogger->_tempCntr[u] = BitLogger->cntr32bit[u];
	}
	//}
	return BitLogger->Q32bit;
}


/*This func. implemented for debugging Multi Thread systems where sometimes we're stuck
on while(someUnhappyFlag){;;;} or on do{;;;}while(someUnhappyFlag) -like operations and
it seems to us that our task where while(...) ops located is not allocated even or it
has been deleted or corrupted or stack/memory overflow allocated. This func. helps to not confuse
and first of all where while(...) -like ops located put this to there and use BitLoggerList(...)
on Timer Interrupt Handler section to know why some tasks are stucked and where. I hope you'll enjoy.
Look the example at FreeRTOS_demo_experiments\ExamplesAndExperiments\RTOSdebuggingTips-Tricks\FindingBugWithBitLoggerList\main.c 
as a reference to guidance. I admit that its not deeply and confidently tested. But it is for while I hope!*/
void SetBitToLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger) //InspectBitToLoggerList //?sounds better?
{
	uint32_t signal = 0;
	for (uint8_t u = 0; u < 32; u++)
	{
		signal = BIT(u) & inputBITx;
		if (signal) {
			BitLogger->cntr32bit[u]++;
			break;
			//BitLogger->Q32bit |= BIT(u);
		}/*else {
			BitLogger->Q32bit &= ~BIT(u);
		}*/
	}
	return;
}

void ResetSpecBitOnLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger)
{
	uint8_t u;
	for (u = 0; u < 32; u++)
	{
		if (inputBITx & BIT(u)) {
			BitLogger->cntr32bit[u] = 0;
			BitLogger->_tempCntr[u] = 0;
			BitLogger->Q32bit &= ~inputBITx;
		}
	}
	return;
}
