#include "BitLogger.h"
#include <string.h> //only for memset

void InitBitLoggerList(BitLoggerList_t* BitLogger)
{
	memset(BitLogger->cntr32bit, 0, sizeof(BitLogger->cntr32bit));
	return;
}

uint32_t BitLoggerList(BitLoggerList_t *BitLogger)
{
	return BitLogger->Q32bit;
}

void SetBitToLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger)
{
	uint32_t signal = 0;
	for (uint8_t u = 0; u < 32; u++)
	{
		signal = BIT(u) & inputBITx;
		if (signal) {
			BitLogger->cntr32bit[u]++;
			BitLogger->Q32bit |= BIT(u);
		}else {
			BitLogger->Q32bit &= ~BIT(u);
		}
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
	BitLogger->Q32bit &= ~inputBITx;
	return;
}