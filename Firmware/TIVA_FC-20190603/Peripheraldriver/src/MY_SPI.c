#include "my_spi.h"
#include "time.h"

/*----------------------------SSI1_Initialisaion for MPU & ICM--------------------------*/
void SSI0_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);  //MPU_FSS
	//GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);  //BARO_FSS
	
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	SSIDisable(SSI0_BASE);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
								 GPIO_PIN_2);
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 |
								 GPIO_PIN_5);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
										 SSI_MODE_MASTER, 1000000, 8);  //1MHZ ,8bit BandWidth
	
	SSIEnable(SSI0_BASE);
	MPU_CS(1);  // CS_n asserted
  //BARO_CS(1);
//// First init SPI interface on PD0-PD3
//	/****一定要打开gpio的数字输出功能，详见数据手册P874页 15.4-5*****/
//	SYSCTL_RCGCSSI_R  |= 0x02;
//	SYSCTL_RCGCGPIO_R |= 0x08;								// activate port D and then wait for bus
//	while((SYSCTL_PRGPIO_R & 0x08) == 0) {};
//																						// configure as SSI ports
	//GPIO_PORTA_PCTL_R = (GPIO_PORTD_PCTL_R & 0xFFFF00F0) + 0x00002202;
	//GPIO_PORTA_AMSEL_R = 0;										// disable analog output
	GPIO_PORTA_AFSEL_R |= 0x34;								// enable alt function on PA2, 4, 5
	GPIO_PORTA_DEN_R |= 0x3C;									// enable digital ports
	//GPIO_PORTA_DIR_R |= 0x08;									// Make PA3 (CS_n) output
	
	SSI0_CR1_R = 0x0;											// disable SSI
	SSI0_CPSR_R = 4;										  // make SSI bandwidth low 1MHz or high 20MHz
	SSI0_CR0_R = 0x000000C7;							// SPO =1, SPH=1; freescale; 8 bits of data at one time
	SSI0_CR1_R |= 0x00000002;							// enable SSI
}

/*----------------------------SSI1_Initialisaion for MPU & ICM--------------------------*/
void SSI1_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	SSIDisable(SSI1_BASE);
	
	HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE+GPIO_O_CR)   |= GPIO_PIN_0;
	HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0x0;
	GPIOPinConfigure(GPIO_PF0_SSI1RX);
	GPIOPinConfigure(GPIO_PF3_SSI1FSS); 
	GPIOPinConfigure(GPIO_PF1_SSI1TX);
	GPIOPinConfigure(GPIO_PF2_SSI1CLK);
	
	MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0);
	MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_3);
	MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_1);
	MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2);

	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
										 SSI_MODE_MASTER, 1000000, 8);  //1MHZ ,8bit BandWidth
	
	SSIEnable(SSI1_BASE);
	BARO_CS(1);  // CS_n asserted

	SSI1_CR1_R = 0x0;											// disable SSI
	SSI1_CPSR_R = 4;										  // make SSI bandwidth low 1MHz or high 20MHz
	SSI1_CR0_R = 0x000000C7;							// SPO =1, SPH=1; freescale; 8 bits of data at one time
	SSI1_CR1_R |= 0x00000002;							// enable SSI
}


/* ----------------------- Internal API for reading/writing registers of MPU&ICM ----------------------------- */
// Send one byte via SPI and then wait for response
unsigned char send_and_receive(unsigned char data) {
	while( (SSI0_SR_R & 0x00000001) == 0) {}		// wait until SSI transmit FIFO empty
	SSI0_DR_R = data;														// write into the data register, start data exchange
	while( (SSI0_SR_R & 0x00000004) == 0) {}		// wait until SSI receive FIFO not empty
	return SSI0_DR_R;														// ack received data
}
// Send one byte via SPI and then wait for response
unsigned char send_and_receive1(unsigned char data) {
	while( (SSI1_SR_R & 0x00000001) == 0) {}		// wait until SSI transmit FIFO empty
	SSI1_DR_R = data;														// write into the data register, start data exchange
	while( (SSI1_SR_R & 0x00000004) == 0) {}		// wait until SSI receive FIFO not empty
	return SSI1_DR_R;														// ack received data
}

unsigned char bytewrite_MPU(unsigned char write_addr, unsigned char write_data) {
	uint8_t return_data[2];
	MPU_CS(0);																// Start transaction
	// delay_micro(100);
	return_data[0] = send_and_receive(write_addr);
	return_data[1] = send_and_receive(write_data);
	MPU_CS(1);
	
	return return_data[1];
}

bool byteread_MPU(unsigned char read_addr, unsigned char *read_data) {
	read_addr |= 0x80;
	*read_data = bytewrite_MPU(read_addr, 0);
	return true;
}
unsigned char ByteRead_MPU(unsigned char Addr)
{
	unsigned char Dat;
	Addr |= 0x80;
	Dat = bytewrite_MPU(Addr, 0);
	return Dat;
}
	
bool Multiread_MPU(unsigned char read_addr, unsigned char *read_buffer, unsigned short bytes_count) {
	read_addr |= 0x80;
	MPU_CS(0);
	send_and_receive(read_addr);
	for(uint16_t i = 0; i < bytes_count; i++) {
		read_buffer[i] = send_and_receive(0);
	}
	MPU_CS(1);
	return true;
}
/*单字节写气压计*/
unsigned char bytewrite_BARO(unsigned char write_addr, unsigned char write_data) {
	uint8_t return_data[2];
	BARO_CS(0);																// Start transaction
	return_data[0] = send_and_receive(write_addr);
	return_data[1] = send_and_receive(write_data);
	BARO_CS(1);
	return return_data[1];
}
/*单字节读取气压计*/
bool byteread_BARO(unsigned char read_addr, unsigned char *read_data) {
	read_addr |= 0x80;
	*read_data = bytewrite_BARO(read_addr, 0);
	return true;
}
unsigned char ByteRead_BARO(unsigned char Addr)
{
	unsigned char Dat;
	Addr |= 0x80;
	Dat = bytewrite_BARO(Addr, 0);
	return Dat;
}
	
bool Multiread_BARO(unsigned char read_addr, unsigned char *read_buffer, unsigned short bytes_count) {
	read_addr |= 0x80;
	BARO_CS(0);
	//Delay_ms(10);
	send_and_receive(read_addr);
	for(uint16_t i = 0; i < bytes_count; i++) {
		read_buffer[i] = send_and_receive(0);
	}
	BARO_CS(1);
	return 0;
}
