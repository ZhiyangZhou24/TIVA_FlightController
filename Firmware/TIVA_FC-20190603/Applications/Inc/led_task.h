//*****************************************************************************
//
// led_task.h - Prototypes for the LED task.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#ifndef __LED_TASK_H__
#define __LED_TASK_H__

#define LED_B(n)    GPIO_PORTB_DATA_R = (n)?(GPIO_PORTB_DATA_R|GPIO_PIN_6):(GPIO_PORTB_DATA_R&(~GPIO_PIN_6))
#define LED_R(n)    GPIO_PORTF_DATA_R = (n)?(GPIO_PORTF_DATA_R|GPIO_PIN_4):(GPIO_PORTF_DATA_R&(~GPIO_PIN_4))
#define LED_G(n)    GPIO_PORTB_DATA_R = (n)?(GPIO_PORTB_DATA_R|GPIO_PIN_7):(GPIO_PORTB_DATA_R&(~GPIO_PIN_7))

//*****************************************************************************
//
// Prototypes for the LED task.
//
//*****************************************************************************
void LEDTaskInit(void);
void LED_Display(unsigned char mode,unsigned char fly_ready);
#endif // __LED_TASK_H__
