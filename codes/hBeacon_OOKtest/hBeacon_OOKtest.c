/****************************************************************************/
/* 					PRATHAM - IITB's Student Satellite                      */
/*                                                                          */
/* Microcontroller:                                                         */
/*          Atmel AVRmega8L                                               */
/* Written for the AVRStudio5 compiler                                      */
/*                                                                          */
/* Author:  Hussain Manasawala, MTech in Electronic Systems, IITB           */
/*                                                                          */
/* Contact: husainmanasa@ee.iitb.ac.in                                      */
/*                                                                          */
/****************************************************************************/
/*
 * hBeacon_OOKtest.c
 *
 * Created: 01-04-2012 01:14:36
 *  Author: Hussain
 */ 

#include "common.h"
#include "uart.h"

/************************************************************/
/*				Main begins									*/
/************************************************************/
int main(void)
{
	/*char array[40];
	char array1[40];
	char array2[50];

	_delay_ms(1000);
 
 	init_UART0();
	
	transmit_UART0('\r');
	transmit_UART0('\r');
	transmit_UART0('H');
	transmit_UART0('e');
	transmit_UART0('l');
	transmit_UART0('l');
	transmit_UART0('o');
	
	sprintf(array,"\t..This is IITB's Student Satellite...\r");
	transmit_string_UART0(array);
	
	sprintf(array1,"\tThis is HUSSAIN's Beacon code...");
	sprintf(array2,"\rTransmitting...\t'PRATHAMIITBOMBAYSTUDENTSATELLITE'\r");*/

	//The morse code will be generated at pin PC0 (Physical pin 23) of the Atmega8.
		
/************************************************************/
	_delay_ms(1000);
	
	DDR_PA = 0x01;
	PORT_PA = 0x00;
	
	while(1){
	
	//transmit_string_UART0(array1);
	//transmit_string_UART0(array2);

/************************************************************/
		//For P
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For R
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For A
		sbi(PORT_PA,PA_EN);	
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For T
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For H
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For A
		sbi(PORT_PA,PA_EN);	
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For M
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For I
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For I
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For T
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For B
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);	
		_delay_ms(300);
		//For O
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For M
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For B
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);	
		_delay_ms(300);
		//For A
		sbi(PORT_PA,PA_EN);	
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For Y
		sbi(PORT_PA,PA_EN);	
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);	
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);	
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);	
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For S
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);	
		_delay_ms(300);
		//For T
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For U
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For D
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For E
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For N
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For T
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For S
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);	
		_delay_ms(300);
		//For A
		sbi(PORT_PA,PA_EN);	
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For T
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For E
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For L
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For L
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For I
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(100);
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For T
		sbi(PORT_PA,PA_EN);
		_delay_ms(300);
		cbi(PORT_PA,PA_EN);
		_delay_ms(300);
		//For E
		sbi(PORT_PA,PA_EN);
		_delay_ms(100);
		cbi(PORT_PA,PA_EN);
		_delay_ms(700);

/************************************************************/
	}
	return 0;
}
/************************************************************/
/*				Main ends									*/
/************************************************************/
