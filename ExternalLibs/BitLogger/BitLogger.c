#include "BitLogger.h"
#include <string.h> //only for memset

void InitBitLoggerList(BitLoggerList_t* BitLogger)
{
	memset(BitLogger->cntr32bit, 0, sizeof(BitLogger->cntr32bit));
	memset(BitLogger->_tempCntr, 0, sizeof(BitLogger->_tempCntr));
	//You may init here the simple timer and Launch
	return;
}

uint32_t BitLoggerList(BitLoggerList_t *BitLogger)
{
	//if(IsTimerWPRinging(BitLogger->BugScannerTimer)){
	for (uint32_t u = 0; u < 32; u++) {
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
for us it seemed that our task where while(...) ops located is not allocated even or it
has been deleted or corrupted or stack/memory overflow allocated. This func. helps to not confuse
and first of all where while(...) -like ops located put this to there and use BitLoggerList(...)
on Timer Interrupt Handler section to know why some tasks are stucked and where. I hope you'll enjoy.
Look the example at main.c as a reference to guidance. I admit that its not deeply and confidently 
tested. But it for while I hope!*/
void SetBitToLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger)
{
	uint32_t signal = 0;
	for (uint8_t u = 0; u < 32; u++)
	{
		signal = BIT(u) & inputBITx;
		if (signal) {
			BitLogger->cntr32bit[u]++;
			//BitLogger->Q32bit |= BIT(u);
		}/*else {
			BitLogger->Q32bit &= ~BIT(u);
		}*/
	}
	return;
}

void ResetSpecBitOnLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger)
{
	uint32_t u;
	for (u = 0; u < 32; u++)
	{
		if (inputBITx & BIT(u))
			break;
	}
	BitLogger->cntr32bit[u] = 0;
	BitLogger->_tempCntr[u] = 0;
	BitLogger->Q32bit &= ~inputBITx;
	return;
}