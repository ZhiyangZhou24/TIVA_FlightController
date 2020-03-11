#include "my_eeprom.h"

#define E2PROM_TEST_ADRES 0x800-4
//共有32页，每页64字节

//写入和读取必须为字的整数倍
unsigned char Write_Buffer[32] = {"hahhhaahahhahahahahaha"};
unsigned char Read_Buffer[32];
void EEPROM_Init(void)
{
	unsigned int e2size,e2block;
	/*******************************/
	/* EEPROM SETTINGS */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); // EEPROM activate
	EEPROMInit(); // EEPROM start
	e2size = EEPROMSizeGet(); // Get EEPROM Size存储器大小
	printf("EEPROM Size: %d bytes\n", e2size);
	e2block = EEPROMBlockCountGet(); // Get EEPROM Block Count
	printf("EEPROM Blok Count: %d\n", e2block);
	/*******************************/
}	
unsigned char Buffer[64];
/*写入字节数必须为4的整数倍，共有32个扇区,每个扇区64字节*/
unsigned char EEPROM_Write(uint32_t *pui32Data, uint32_t block, uint32_t ui32Count)
{
	static unsigned int ui32Address;
	ui32Address = 0x0000+block*64;
	if(EEPROMProgram((uint32_t *)pui32Data, ui32Address,ui32Count))
		return 1;
	return 0;//access success!
}
void EEPROM_Read(uint32_t *pui32Data, uint32_t block, uint32_t ui32Count)
{
	//static unsigned int i=0;
	static unsigned int ui32Address;
	ui32Address = 0x0000+block*64;
	EEPROMRead((uint32_t *)pui32Data, ui32Address,ui32Count);
}
