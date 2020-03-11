#ifndef _MY_EEPROM_H_
#define _MY_EEPROM_H_
#include "include.h"
#include "eeprom.h"

void EEPROM_Init(void);
unsigned char EEPROM_Write(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count);
void EEPROM_Read(uint32_t *pui32Data, uint32_t block, uint32_t ui32Count);
#endif 
