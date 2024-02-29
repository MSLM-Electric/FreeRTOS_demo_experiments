#include <stdint.h>

#ifndef BIT
#define BIT(x) (uint32_t)(1 << x)
#endif // !BIT


typedef struct {
	uint32_t cntr32bit[32];
	uint32_t Q32bit;
}BitLoggerList_t;

void InitBitLoggerList(BitLoggerList_t* BitLogger);
uint32_t BitLoggerList(BitLoggerList_t* BitLogger);
void SetBitToLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger);
void ResetSpecBitOnLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger);