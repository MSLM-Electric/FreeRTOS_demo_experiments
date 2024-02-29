#include <stdint.h>

#ifndef BIT
#define BIT(x) (uint32_t)(1 << x)
#endif // !BIT


typedef struct {
	uint32_t cntr32bit[32]; //4x32 = 128bytes
	uint32_t _tempCntr[32]; //128bytes  //for 32bit logging signals we waste the 260bytes. mb there are solvings on another way.
	uint32_t Q32bit;
}BitLoggerList_t;

void InitBitLoggerList(BitLoggerList_t* BitLogger);
uint32_t BitLoggerList(BitLoggerList_t* BitLogger);
void SetBitToLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger);
void ResetSpecBitOnLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger);