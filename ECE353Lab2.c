// General MIDI Explorer
// Written by: Arthur Singas

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/iom32.h>
#include <util/delay.h>

///////////////////////////
// Definitions for USART //
///////////////////////////

#define BAUD_RATE 31250       
#define BAUD_PRESCALE 7
#define CPU_FREQ 4000000

struct Note                                            // Struct to input and output USART values
{
	unsigned short status;
	unsigned short note_value;
	unsigned int velocity;
	unsigned short time;
};

struct Note note_instance; 

///////////////////////////
// TIMER1 Implementation //
///////////////////////////

void timer1_init()
{
	TCCR1B |= (1 << CS12) ; 							// prescaler = 256  
	TCNT1 = 0;											// Clear count										
	OCR1B = 12500; 										// 800 ms count
	TIMSK |= (1 << OCIE1B); 
	sei();												// enable global interrupts
}	

ISR(TIMER1_COMPB_vect)                                  // turn off LEDs on interrupt
{	
	PORTB = 0x00;
}

// Method to keep track of time that note is played 
unsigned int TIM16_ReadTCNT1(void)
{
	unsigned char sreg;
	unsigned int i;
	sreg = SREG;										// Save global interrupt flag
	cli();												// disable interrupts
	i = TCNT1;											// Read TCNT1 into i
	SREG = sreg;										// Restore global interrupt flag
	return i;
}

///////////////////////////
// USART Implementation ///
///////////////////////////

void USART_Init()
{
	UBRRL = BAUD_PRESCALE;
	UBRRH = (BAUD_PRESCALE >> 8);
	UCSRB = (1 << RXEN) | (1 << TXEN);					// Enable receiver and transmitter
	UCSRC = (1 << URSEL) | (3 << UCSZ0);				// Set frame format: 8data, 2stop bit
}

void USART_Transmit(unsigned char data)
{
	while (!(UCSRA & (1 << UDRE)));					    // Wait for empty transmit buffer
		UDR = data;										// Put data into buffer, sends the data
}

unsigned char USART_Receive(void)                       // return byte recieved from MIDI Ox
{
	while (!(UCSRA & (1 << RXC)) && (PINA &(1 << PA0)));

		if(!(PINA&(1 << PA0)))
		{                             
			return 0;
		}
	   	return UDR;
}

void USART_Flush(void)
{
	unsigned char dummy;
	while (UCSRA & (1 << RXC));
		dummy = UDR;
}
//////////////////////////////
///// Analog to Digital //////
//////////////////////////////

void initADC()
{
	DDRA = DDRA & 0x7F;
	ADMUX = (1<<REFS0);
	ADCSRA = (1<< ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t readADC(uint8_t chan)
{
	ADMUX |= chan;
	ADCSRA |=(1<< ADSC);    							// Start conversion
    while (!(ADCSRA &(1<<ADIF)));
	ADCSRA |= (1<<ADIF);
	return (ADC);

}

//////////////////////////////
/// EEPROM Implementation  ///
//////////////////////////////

void write_eeprom_byte(unsigned short address, unsigned char data)
{
	EEAR = address;
	EEDR = data;
	EECR |= (1<<2);
	EECR |= (1<<1);
	while (EECR & (1<<1));
	EECR &=(1<<2);
}

unsigned char read_eeprom_byte(unsigned short address)
{
	while (EECR & (1<<EEWE));							// Waiting for previous write to complete
	EEAR = address;						
	EECR |= (1<<EERE);									// Start eeprom read by writing EERE
	EECR |= (1<<2);
	EECR |= (1<<1);
	return EEDR;					   					// return data from data register
}

//////////////////////////
////// RECORD Mode ///////
//////////////////////////

void record_mode()
{		 
	char delayOnUpper = 0;
	char delayOffUpper = 0;
	char delayOnLower = 0;
	char delayOffLower = 0;
	int delayOff = 0;
	int address = 0;
	int visited = 0;
	int validAddTop;
	int validAddBot;

	int vBitTop = read_eeprom_byte(1020);
	int vBitBot = read_eeprom_byte(1019);
	int valid = 0;
	valid |= vBitBot;
	valid = valid|(vBitTop << 8);

	timer1_init();

	while (1)
	{
		if (address >= 0x3F0){
			return;
		}

		DDRB = 0xFF;

    	if (!(PINA &(1<<PA0))){
			break;
    	}

		USART_Init();
		
		if (visited == 1){
			address = 0;
		}
		
		note_instance.status = USART_Receive();									// Receiving note byte values and storing them in note structure
		note_instance.note_value = USART_Receive();
		note_instance.velocity = USART_Receive();
		note_instance.time = TIM16_ReadTCNT1();
		
	
		if (note_instance.status == 0x90 && note_instance.velocity == 0x40){
			return;
		}

		if (note_instance.velocity == 0 || note_instance.status == 0 || note_instance.note_value == 0){
			return;
		}
		else {
			if (!(PINA &(1<<PA0))){
				return;
			}

			delayOnLower = (note_instance.time& 0xFF);
			delayOnUpper = (note_instance.time>>8);
			PORTB = note_instance.note_value;
			write_eeprom_byte(address, note_instance.status);
			write_eeprom_byte(address + 1, note_instance.note_value);
			write_eeprom_byte(address + 2, note_instance.velocity);
			write_eeprom_byte(address + 3, delayOnUpper);
			write_eeprom_byte(address + 4, delayOnLower);
			address = address + 5;
			TCNT1 = 0;
			 	 
			note_instance.status = USART_Receive();
			note_instance.note_value = USART_Receive();
			note_instance.velocity = USART_Receive();
			note_instance.time = TIM16_ReadTCNT1();
			
			delayOff = TIM16_ReadTCNT1();
			TCNT1 = 0;
			delayOffLower = (delayOff& 0xFF);			
			delayOffUpper = (delayOff>>8);
			write_eeprom_byte(address, note_instance.status);
			write_eeprom_byte(address + 1, note_instance.note_value);
			write_eeprom_byte(address + 2, note_instance.velocity);
	    	write_eeprom_byte(address + 3, delayOffUpper);
			write_eeprom_byte(address + 4, delayOffLower);
			address = address + 5;
			
			validAddBot = (address & 0xFF);
	    	validAddTop = (address>>8);
			write_eeprom_byte(1019, validAddBot);
        	write_eeprom_byte(1020, validAddTop);
		}
	}
}

void playback_mode()
{	
	USART_Init();
	TCNT1 =0;
	int vBitTop = read_eeprom_byte(1020);
	int vBitBot = read_eeprom_byte(1019);
	int valid = 0;
	valid |= vBitBot;
	valid = valid|(vBitTop<<8);
	
	int delayOff = 0;
	char delayOnUpper = 0;
	char delayOffUpper = 0;
	char delayOnLower = 0;
	char delayOffLower = 0;

	int time1;
	int time2;
	int address = 0;

	while (1){
		
		if (address+10 <= valid){
			note_instance.status = read_eeprom_byte(address);								
			note_instance.note_value = read_eeprom_byte(address + 1);
			note_instance.velocity = read_eeprom_byte(address + 2);
			delayOnUpper = read_eeprom_byte(address + 3);
			delayOnLower = read_eeprom_byte(address + 4);
			address = address + 5;
			
			if (note_instance.note_value == 0){
				return;
			}
			
			time1 = delayOnLower;
			time1 = time1|(delayOnUpper<<8);
			time1 = (time1/2232) * 500;
			_delay_ms(time1);

			//display and send note to USART
			PORTB = note_instance.note_value;
			USART_Transmit(note_instance.status);
			USART_Transmit(note_instance.note_value);
			USART_Transmit(note_instance.velocity);
			TCNT1 =0;
			
			note_instance.status = read_eeprom_byte(address);
			note_instance.note_value = read_eeprom_byte(address + 1);
			note_instance.velocity = read_eeprom_byte(address + 2);
			delayOffUpper = read_eeprom_byte(address + 3);
			delayOffLower = read_eeprom_byte(address + 4);
			address = address + 5;

			if (note_instance.note_value == 0){
				return;
			}
			
			time2 = delayOffLower;
			time2 = time2|(delayOffUpper<<8);
			time2 = (time2/2232) * 500;
			
			_delay_ms(time2);
		
			PORTB = note_instance.note_value;
			USART_Transmit(note_instance.status);
			USART_Transmit(note_instance.note_value);
			USART_Transmit(note_instance.velocity);
			_delay_ms(500);
			
		} else {
			address = 0;
		}

		DDRB = 0xFF;

		if((PINA&(1<<PA2)) && (PINA & (1<<PA1))){
			modify_mode();
		}
		else if(!(PINA & (1<<PA1))){
			break;
		}
	}
}

void modify_mode()
{
	USART_Init();
	TCNT1 =0;
	int vBitTop = read_eeprom_byte(1020);
	int vBitBot = read_eeprom_byte(1019);
	int valid = 0;
	valid |= vBitBot;
	valid = valid|(vBitTop<<8);
	
	int delayOff = 0;
	char delayOnUpper = 0;
	char delayOffUpper = 0;
	char delayOnLower = 0;
	char delayOffLower = 0;

	int time1;
	int time2;
	int address = 0;
	int timeChange = 1;

	while(1){
		
		if (address+10 <= valid){
			uint16_t volt = 0;
            volt = readADC(7);

			if (volt > 700){  
				timeChange = 150;				// Covered, slow down
			}
			else if (volt < 100){   					
				timeChange = 10;				// Lit up, speed up
			}
			else if (volt > 185){
				timeChange = 1;					// Else, normal
			}

			note_instance.status = read_eeprom_byte(address);
			note_instance.note_value = read_eeprom_byte(address +1);
			note_instance.velocity = read_eeprom_byte(address + 2);
			delayOnUpper = read_eeprom_byte(address + 3);
			delayOnLower = read_eeprom_byte(address + 4);
			address = address + 5;
			
			if (note_instance.note_value == 0){
				return;
			}
			
			time1 = delayOnLower;
			time1 = time1|(delayOnUpper<<8);
			time1 = (time1/2232) * 500;

			if (timeChange == 150){
				time1 = time1*4;
			}
			if (timeChange == 10){
				time1 = time1/timeChange;
			}
			if (timeChange == 1){
				time1 = time1/timeChange;
			}
			_delay_ms(time1);

			//display and send note to USART
			PORTB = note_instance.note_value;
			USART_Transmit(note_instance.status);
			USART_Transmit(note_instance.note_value);
			USART_Transmit(note_instance.velocity);
			
			note_instance.status = read_eeprom_byte(address);
			note_instance.note_value = read_eeprom_byte(address +1);
			note_instance.velocity = read_eeprom_byte(address + 2);
			delayOffUpper = read_eeprom_byte(address + 3);
			delayOffLower = read_eeprom_byte(address + 4);
			address = address + 5;

			if (note_instance.note_value == 0){
				return;
			}
			
			time2 = delayOffLower;
			time2 = time2|(delayOffUpper<<8);
			time2 = (time2/2232) * 500;

			if (timeChange == 150){
				time2 = time2*4;
			}
			if (timeChange == 10){
				time2 = time2/timeChange;
			}
			if (timeChange == 1){
				time2 = time2;
			}

			_delay_ms(time2);
			PORTB = note_instance.note_value;
			USART_Transmit(note_instance.status);
			USART_Transmit(note_instance.note_value);
			USART_Transmit(note_instance.velocity);
			_delay_ms(500);
		}
		else {
			address = 0;
		}

		DDRB = 0xFF;

		if (!((PINA&(1<<PA2)) && (PINA & (1<<PA1)))){
			return;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// MAIN ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

int main (void)
{
	DDRA = 0x7F;
	initADC();
	_delay_ms(50);
	while(1)
	{
		DDRB = 0xFF;
        DDRA = DDRA &0xF8;    										    // Record

		// Choosing which mode 

        if (PINA &(1 << PA0)){      	 								// if record, go to record mode
	        record_mode();	
	    } else{                          							    // if not in record, modify or playback
      		if((PINA&(1 << PA2)) && (PINA & (1 << PA1))){				// modify mode
				modify_mode();
			}
			else if(PINA & (1 << PA1)){ 								// if not modify or record, playback		   	    
			    playback_mode();
			} else {
			    PORTB = 0x00;               	 					    // if none on, continue
				continue;
			}
		}
	}	 
}

