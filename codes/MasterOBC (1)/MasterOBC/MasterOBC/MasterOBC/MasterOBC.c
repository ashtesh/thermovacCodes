/*
 * CFile2.c
 *
 * Created: 11-Dec-14 5:23:00 PM
 *  Author: Harshal Jalan
 */ 

#define F_CPU 8000000
#include "spi.c"
#include "common.h"
#include "comm.h"
#include "util/delay.h"
#include "uart.h"
#include "util/twi.h"
#include "peripherals.h"
#include "mag.h"
#include "avr/sleep.h"
	volatile uint8_t check; //Variable for UART INT vector
	char HM_Data[7]; //Array for HM Data
	volatile struct state Current_state;
	uint8_t address=0x20, read=1, write=0;// Variables for I2C
	unsigned char write_data=0xFC, recv_data;//Instruction to be sent to the Power Board. Each bit is for turning one load ON/OFF
	unsigned int counter1 = 0; //Beacon OverCurrent controller
	unsigned int counter2 = 0; //Control OverCurrent controller
	unsigned int counter3 = 0; //GPS OverCurrent controller
	unsigned int counter4 = 0; //Downlink OverCurrent controller
	unsigned int overGS = 0;//Flag to see if the satellite has to start transmitting or not
	unsigned int counterforGS = 0;//Variable to count the number of cycles passed when not over GS. Here GS is only a representation of the Ground Station. It has nothing to do with the actual GS
	unsigned int CyclesToGS = 1350;//Go to transmission mode after these many cycles
	unsigned int CounterInsideGS = 0;//Variable to count the number of cycles passed when over GS
	unsigned int CyclesForDownlink = 300;//No. of cycles when downlink is ON.
	unsigned int CounterForUplink = 150;//No. of cycles when uplink is ON.
	unsigned int UniversalCycles = 1; //Variable that counts the number of loops
	unsigned int StarReceived = 0;  //Variable for controlling wireless block
	unsigned int CyclesToCollectData = 5; //No. of Cycles after which data is to be sent on UART
	unsigned int StartTorquer = 0; //Variable for controlling Torquer
	unsigned int TorquerCycles = 0; //Cycles when torquer is ON
	uint8_t GPS_Received; //Variable for receiving GPS Data
	
	
	char GPS_Data[13] = "IamUndefined";// Dummy Lat-Long-Alt data, 12 byte long
	
	void calluart(char d)// UART transmit code written by Ashtesh 
	{
		long fosc=8000000;                //setting the baud rate
		int ubrr=((fosc/9600)/16)-1;
		UBRR0H=(unsigned char)(ubrr>>8);
		UBRR0L=(unsigned char)ubrr;
		UCSR0B|=((1<<RXEN0)|(1<<TXEN0)); //enable the receiver and transmitter
		UCSR0C = (1<<USBS0)|(3<<UCSZ00);                 //2 stop bit
		;//8 bit data
		
		
		
		//while(1)
		while(!(UCSR0A&(1<<UDRE0)));     //waiting for transmit buffer to be clear
		UDR0=d;
		
		
	}

	void transmitSunSensorUart(int temp) { //Function for Sending Sunsensor data through UART
		transmit_UART0(temp & 0xff);
		transmit_UART0(temp >> 8);
	}

	uint16_t convert(uint8_t vall, uint8_t valh) //to convert 2 8 bit integer values to one 16 bit value
	{
		
		uint16_t val= valh;
		val=(val<<8)+vall;
		return val;
	}
	/*uint8_t SPI_transfer(uint8_t transmit_byte)
	{
		SPDR = transmit_byte;
		///Wait for Transmission to complete
		while(!(SPSR & (1<<SPIF)));
		///return received Byte
		return SPDR;
	}*/
	
	void TWI_init_master(void) // Function to initialize master for I2C
	{
		//sei();

		TWSR = 0;
		TWCR = 0;
		TWBR = (F_CPU / 200000UL - 16) / 2; // Bit rate
		//TWSR=(0<<TWPS1)|(0<<TWPS0); // Setting prescalar bits
		// SCL freq= F_CPU/(16+2(TWBR).4^TWPS)

	}

	void TWI_start(void) //Function to send I2C start command
	{
		// Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
		TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		while(!(TWCR & (1<<TWINT))); // Wait till start condition is transmitted
		while((TWSR & 0xF8)!= 0x08); // Check for the acknowledgement
	}

	void TWI_repeated_start(void) // Function to send I2C repeated start command. Scarcely used
	{
		// Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
		TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		while(!(TWCR & (1<<TWINT))); // wait till restart condition is transmitted
		while((TWSR & 0xF8)!= 0x10); // Check for the acknoledgement
	}

	void TWI_write_address(unsigned char data)//Function for Master side to send slave address for I2C
	{

		TWDR=data; // Address and write instruction
		TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
		while (!(TWCR & (1<<TWINT)))// Wait till complete TWDR byte transmitted
		while((TWSR & 0xF8)!= 0x18);  // Check for the acknowledgement

	}

	void TWI_read_address(unsigned char data) //Function for slave side to read address sent by Master
	{
		TWDR=data; // Address and read instruction
		TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
		while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte received
		while((TWSR & 0xF8)!= 0x40);  // Check for the acknoledgement
	}

	void TWI_write_data(unsigned char data)//Function to write data on I2C data line
	{
		TWDR=data; // put data in TWDR
		TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
		while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted
		while((TWSR & 0xF8) != 0x28); // Check for the acknoledgement
	}

	void TWI_read_data(void) //Function to read data from I2C data line
	{
		TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
		while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted
		while((TWSR & 0xF8) != 0x58); // Check for the acknoledgement
		recv_data=TWDR;//PORTA=recv_data;
		if(UniversalCycles % CyclesToCollectData == 0){transmit_UART0(recv_data);}
		
	}

	void TWI_stop(void)//Function to stop data transmission
	{
		// Clear TWI interrupt flag, Put stop condition on SDA, Enable TWI
		TWCR= (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
		while(!(TWCR & (1<<TWSTO)));  // Wait till stop condition is transmitted
	}
	
	void torquer_only_x_positive(void){//Function to turn on only x-direction torquer. These functions are often meddled with. So don't be surprised if the definition doesn't match the desclaration	
		reset_PWM();
		Current_state.pwm.x_dir = 0;
		Current_state.pwm.x = 32786;
		Current_state.pwm.y_dir = 0;
		Current_state.pwm.y = 32786;
		Current_state.pwm.z_dir = 0;
		Current_state.pwm.z = 32786;
		set_PWM ();
	}

	void torquer_only_y_positive(void)//Function to turn on only y-direction torquer
	{
		reset_PWM();
		Current_state.pwm.x_dir = 0;
		Current_state.pwm.x = 32786;
		Current_state.pwm.y_dir = 0;
		Current_state.pwm.y = 32786;
		Current_state.pwm.z_dir = 0;
		Current_state.pwm.z = 32786;
		set_PWM ();
	}

	void torquer_only_z_positive(void)//Function to turn on only z-direction torquer
	{
		reset_PWM();
		Current_state.pwm.x_dir = 0;
		Current_state.pwm.x = 0;
		Current_state.pwm.y_dir = 0;
		Current_state.pwm.y = 0;
		Current_state.pwm.z_dir = 0;
		Current_state.pwm.z = 32768;
		set_PWM ();
	}
	
	void torquer_all_zeroes(void)//Function to turn off all torquers
	{
		reset_PWM();
		Current_state.pwm.x_dir = 0;
		Current_state.pwm.x = 0;
		Current_state.pwm.y_dir = 0;
		Current_state.pwm.y = 0;
		Current_state.pwm.z_dir = 0;
		Current_state.pwm.z = 0;
		set_PWM ();
	}

	
ISR(USART0_RX_vect) //Interrupt vector for UART. UARTs received from the GPS and the external PC will be handled by this
{
	check = UDR0;
	//transmit_UART0(check);
	if(check == '*'){PORTA = 0x11;//Instruction received to turn on downlink and uplink. Downlink will start immediately and then uplink and then there will be the cycle Normal - Downlink - Uplink
		_delay_ms(5000);
		StarReceived = 1;
		//FirstStar = 1;
		counterforGS = CyclesToGS-1;
	}
	else if(check == '&')//Instruction received to turn off downlink indefinitely. It own't start till we transmit *
	{
		PORTA = 0xCC;
		_delay_ms(5000);
		StarReceived = 0;
		}
		else if(check == '^')//Instruction received to turn on Torquer. X,Y,Z are turned on serially for 30 seconds
		{
			PORTA = 0xEE;
			_delay_ms(5000);
			StartTorquer = 1;
		}
	
}

/*ISR(USART1_RX_vect)//ISR for Magmeter UART
{
	uint8_t a =UDR1;
	if (UniversalCycles%CyclesToCollectData == 0) {transmit_UART0(a);}
}*/
	

int main (void)
{
	//watch_dog(T_POWER);
//Start:
//timer1_init();
init_SPI();                 
init_UART0();               
init_UART_MM();
configure_torquer();
//sei();
DDRA=0xF0;
DDRB |= (1<<PB0)|(1<<PB5);//PB0 and PB5 are slave select pins for Slave OBC and ADC
int j=1;   
_delay_ms(2000);
PORTA=0b11010000; //LED indicator for debugging


TWI_init_master(); // Function to initialize TWI
uint8_t data_t; 
uint8_t data_r;


//***************ADC Variables************************
uint8_t valh1;//High byte of first sunsensor
uint8_t vall1;//Low byte of first sunsensor
uint8_t valh2;
uint8_t vall2;
uint8_t valh3;
uint8_t vall3;
uint8_t valh4;
uint8_t vall4;
uint8_t valh5;
uint8_t vall5;
uint8_t valh6;
uint8_t vall6;

uint16_t val0;//First sunsensor combined 16 bit value
uint16_t val1;
uint16_t val2;
uint16_t val3;
uint16_t val4;
uint16_t val5;
_delay_ms(2000);
sei(); //

while(1)
{
	
	//check = receive_UART0();
	//if(check == '*'){PORTA = 0x11;
		// _delay_ms(5000);
		 //StarReceived = 1;
		 //FirstStar = 1;
		 //counterforGS = CyclesToGS-1;
		 //}
		 wdt_enable(WDTO_2S);
		 wdt_reset();
		 
	PORTB |= (1<<PB5); //Set slave select of ADC =1
	PORTB &= ~(1<<PB0);//Set slave select of Slave OBC = 0
	//SPCR |= (1<<SPE);
	write_data&= ~(1<<4);// Turn off both downlink
	write_data&= ~(1<<1);//Turn off uplink
	//transmit_UART0('a');
PORTA = 0xAA;
TWI_start(); // Function to send start condition
PORTA=0b11000000;
TWI_write_address(address); // Function to write address and data direction bit(write) on SDA

PORTA=0b01100000;
TWI_write_data(write_data);     // Function to write data in slave
PORTA=0b10100000;
TWI_stop(); // Function to send stop condition
//transmit_UART0('b');

if (UniversalCycles%CyclesToCollectData == 0){transmit_string_UART0("PRA");}
_delay_ms(10); // Delay of 10 mili second
//************************Get HM Data from Power Board*************************************
for(int i=0;i<7;i=i+1) 
{
	TWI_start();
	
	TWI_read_address(address+read); // Function to write address and data direction bit(read) on SDA
	TWI_read_data(); // Function to read data from slave
	HM_Data[i] = recv_data;
	TWI_stop();
}
_delay_ms(10);


	for (uint8_t i =0;i<7;i++) {	
			SPDR = HM_Data[i];
			while(!(SPSR & (1<<SPIF) ));
			PORTA = 0xFF;
			_delay_ms(1);
		}
		PORTB |= (1<<PB0);
		PORTB &= ~(1<<PB5);
		wdt_reset();
		
		//SPCR &= ~(1<<SPE);		
		//****************************OC Check Block *************************************//
		
		if(counter1 == 0) //OC Check for Beacon
		{
			if((HM_Data[6]&(0x80)) == 0)
			{
				counter1 = 1;
				write_data &= ~(1<<7);
				
			}
		}
		
		if((counter1 > 0) && (counter1 < 7))
		{
			counter1 = counter1+1;
		}
		
		if(counter1 == 7)
		{
			counter1 = 0;
			write_data |= (1<<7);
		}
		
		if(counter2 == 0) //OC check for Torquer
		{
			if((HM_Data[6]&(0x40)) == 0)
			{
				counter2 = 1;
				write_data &= ~(1<<6);
				
			}
		}
		
		if((counter2 > 0) && (counter2 < 7))
		{
			counter2 = counter2+1;
		}
		
		if(counter2 == 7)
		{
			counter2 = 0;
			write_data |= (1<<6);
		}
		
		if(counter3 == 0) //OC check for GPS
		{
			if((HM_Data[6]&(0x20)) == 0)
			{
				counter3 = 1;
				write_data &= ~(1<<5);
				
			}
		}
		
		if((counter3 > 0) && (counter3 < 7))
		{
			counter3 = counter3+1;
		}
		
		if(counter3 == 7)
		{
			counter3 = 0;
			write_data |= (1<<5);
		}

		if(counter4 == 0) //OC Check for Downlink
		{
			if((HM_Data[6]&(0x10)) == 0)
			{
				counter4 = 1;
				write_data &= ~(1<<4);
				
			}
		}
		
		if((counter4 > 0) && (counter4 < 7))
		{
			counter4 = counter4+1;
		}
		
		if(counter4 == 7)
		{
			counter4 = 0;
			write_data |= (1<<4);
		}
		//calluart(i);
	//**********************************OC Check Block Ends*********************************************//	
	//**********************************ADC Block Begins************************************************//
	wdt_reset();	
	PORTB |= (1<<PB0);//Set Slave select of Slave OBC to 1
	PORTB &= ~(1<<PB5);// Set slave select of ADC to 0
	//SPCR |=(1<<SPE);
	wdt_disable();

//transmit_UART0(UniversalCycles%CyclesToCollectData);
UniversalCycles = UniversalCycles+1; //Increment Universal loop cycles

}

}
		
