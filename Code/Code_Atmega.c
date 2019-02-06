/*
 * Authors : Erendiro Pedro (1160555) & Elmer Graça (1161424)
 */ 

//MACROS
#define F_CPU 1000000UL
#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
#define VELMAX 45
#define VELMED 38
#define S_VEC 650 

//includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>


//Pin name definition
#define leftSensorPin (1<<PD2)
#define middleSensorPin (1<<PD3)
#define rightSensorPin (1<<PD4)
#define rightMotor OCR0B
#define leftMotor OCR0A


//Functions declaration
void init();
void initEXT_INT();
void initTIMER();
void initMOTORS();
void USART_Init();
void usart_init();
void replay();
void change_state();
void USART_Transmit(char * data);
void handle_buffer();
void delay_ms(int count);
unsigned int Cantor_PF(unsigned int a, unsigned int b);

//Global variables

//Flags and counters

volatile unsigned char flag_450ms=0;
volatile unsigned char cont_450ms=0;
volatile unsigned char cont_500ms=0;
volatile unsigned char flag_30ms=0;
volatile unsigned char cont_30ms=0;
volatile unsigned char flag_100ms=0;
volatile unsigned char cont_100ms=0;

	//replay
volatile unsigned char flag_rp=0;
volatile unsigned long cont_time_rp=0;
unsigned char state_b=0;
unsigned char state=0;
unsigned int i=0;

	//Odometry
volatile unsigned short int cont_rev_l=0;
volatile unsigned short int cont_rev_r=0;
volatile unsigned char buffer[15];



//Vectors
unsigned int vec_tmp_chg[S_VEC]; 
unsigned char vec_state[S_VEC]; 


//
int main(void){
	
	init();
	USART_Init();
	initEXT_INT();
	initTIMER();
	initMOTORS();
	
	rightMotor = 0;
	leftMotor = 0;

	for(unsigned int k=0; k<S_VEC; k++){ //initialize all positions
		vec_tmp_chg[k]=0;
		vec_state[k]=6;
	}
	
	_delay_ms(3000); //wait 3s to start
	
	while (1){
		
		// L-left; M-middle; R-right
		// B-black (B=0); W-white (W=1)

		if (flag_rp==0){
			
			state_b=state; change_state();
			
			if( (PIND & leftSensorPin) && !(PIND & middleSensorPin) && (PIND & rightSensorPin) ){ //0 1-0-1
				state=0;
				
				} else if( !(PIND & leftSensorPin) && !(PIND & middleSensorPin) && !(PIND & rightSensorPin) ){ //1 0-0-0
					state=1;
				
					} else if( !(PIND & leftSensorPin) && (PIND & middleSensorPin) && (PIND & rightSensorPin) ){ //2 0-1-1
						state=2;
				
						} else if( (PIND & leftSensorPin) && (PIND & middleSensorPin) && !(PIND & rightSensorPin) ){ //3 1-1-0
							state=3;
				
							} else if( !(PIND & leftSensorPin) && !(PIND & middleSensorPin) && (PIND & rightSensorPin) ){ //4 0-0-1 *
								state=4;
				
								} else if( (PIND & leftSensorPin) && !(PIND & middleSensorPin) && !(PIND & rightSensorPin) ){ //5 1-0-0 *
									state=5;
				
									} else if((PIND & leftSensorPin) && (PIND & middleSensorPin) && (PIND & rightSensorPin)){ //X 1-1-1
										state=6; flag_rp=1; change_state(); //finished mapping
				
										} else { // Will never happen.
											state=7; leftMotor=0; rightMotor=0;  //for debugging
											}


				
			if( (state!=state_b) && (flag_450ms==1) ){
				
			//Every 450ms if the state has changed, store the time and the position on a vector	
				vec_tmp_chg[i]=cont_time_rp;
				vec_state[i]=state; //tenho de fazer alocação dinâmica
				i++;
				flag_450ms=0;
				
			}



		}

		
	}
}


//-----Initializations-----//
void init(){

	//Change clock frequency
	CLKPR = (1<<CLKPCE);
	CLKPR = 0; //all the other bits must be written as 0

	//Pin configuration
	//Motor - Define rotation sense
	DDRC=0b00001111;
	PORTC=0b00001001;
	
	//IR Sensor & Blink_LED
	DDRD=0b01100010; //PD4 - input(sensor); Pin0/1 RX/TX input/output; 
	PORTD=0b00000000;
	
	DDRB=0x01; //PB6, PB7 - input(sensors) | PB0 - output(LED)
	PORTB=0; //PB6, PB7 - pull up on | PB0 - OFF
	
}
void initEXT_INT(){
	
	//Interrupts initializations
	EICRA|=(1<<ISC11)|(1<<ISC10); //INT1 - Interrupts in the rising edge PD3
	EICRA|=(1<<ISC01)|(1<<ISC00); //INT0 - Interrupts in the rising edge PD2
	EIMSK|=(1<<INT1)|(1<<INT0); //enable INT0 and INT1	
	
	PCICR|=(1<<PCIE1); PCMSK1|=(1<<PCINT13); //Activate PCINT13 PC5
	
}
void initTIMER(){	
		
	//Timer2 CTC mode (~=10ms)
	TCCR2B &=~(1<<WGM22); TCCR2A |=(1<<WGM21); TCCR2A &=~(1<<WGM20); //set timer to CTC mode

	TCCR2B |=(1<<CS21);TCCR2B |=(1<<CS22); TCCR2B |=(1<<CS20); //Prescaler 1024
	TIMSK2 |=(1<<OCIE2A); //activate OCIE2A mask to enable interrupt

	OCR2A=38; 

	sei(); //start interruptions
	
}
void initMOTORS(){
	
	//Timer0 PWM fast PWM
	TCCR0B &=~(1<<WGM02); TCCR0A |=(1<<WGM01); TCCR0A |=(1<<WGM00); //Top 0xFF, update of OCRx at BOTTOM
	TCCR0B |=(1<<CS02); TCCR0B &=~(1<<CS01); TCCR0B &=~(1<<CS00); //Prescaler 256

	//Activate OCR0A
	TCCR0A |=(1<<COM0A1); TCCR0A &=~(1<<COM0A0); //non-inverting mode
	//Activate OCR0A
	TCCR0A |=(1<<COM0B1); TCCR0A &=~(1<<COM0B0); //non-inverting mode
	//the wave will be generate in OC0A/Oc0B (PD6/PD5)
	
}
void USART_Init(){
	UBRR0H = 0; //Asyncronous mode
	UBRR0L = 103; //set baud rate 9600 and double rate for less error

	UCSR0B = (1<<RXEN0)|(1<<TXEN0);    //Enable receiver and transmitter 
	UCSR0C = (3<<UCSZ00);     // Set frame format: 8data, 2stop bit 


}

//-----Interrupts-----//
ISR (INT0_vect){ //left motor Encoder
	if(flag_100ms==0) cont_rev_l++; //number of pulses
}
ISR(INT1_vect){//Right motor Encoder
	if (flag_100ms==0) cont_rev_r++; //number of pulses
}
ISR(PCINT1_vect) {//Interrupt do replay 
	if(flag_rp==1) replay();

}
ISR(TIMER2_COMPA_vect){
	//set to be activated at 10ms 

	//For blinking LED
	if(cont_500ms==50){
		//0.5s has passed
		PORTB ^= (1<<PINB0);
		cont_500ms=0; //reset variable
	} else cont_500ms++; 
	
	//To send data throw bluetooth
	if(cont_100ms==10){
		
		int aux = 0; 	
		handle_buffer();
/*
		unsigned int z = Cantor_PF(cont_rev_l, cont_rev_r); 
				
		do
		{
			buffer[aux] = (char)(z % 10) + '0'; //convert integer to character
			z /= 10;
			aux++;
				
		} while(z);
			*/
/*
		for(aux=0; aux<12;aux++){
			buffer[aux]='a';
		}
		*/
		//USART_Transmit((char*) buffer);//call usart and send z ( data updated every 100ms )

		//reset variables
		cont_rev_l=0;
		cont_rev_r=0;
		cont_100ms=0;
			
	} else cont_100ms++;	
	
	//
	if(flag_rp==0){
		if (flag_450ms==0){
			if(cont_450ms==45){
				//0.45s has passed
				flag_450ms=1;
				cont_450ms=0;
			} else cont_450ms++;
		}
		cont_time_rp++; //number of 10 ms that have passed
	}
	
}

//-----Functions-----//
void delay_ms(int count) {
	//we can't use _delay_ms with variables as arguments
	//so we are using this function to help us
	while(count--) {
		_delay_ms(1);

	}
}
void USART_Transmit( char * data){
	unsigned char k=0;
	while(data[k]!='\0'){
		
		while ( !( UCSR0A & (1<<UDRE0)) );     /* Wait for empty transmit buffer */
		UDR0 = data[k]; /* Put data into buffer, sends the data */
		k++;
	}
}
void handle_buffer(){
	unsigned char l=0;
	for (l=0;l<strlen((char*)buffer);l++){
		buffer[l]='\0';
	}	
}
void replay() { 
	unsigned short int c=0;
	state=0; change_state();
	
	for (c=0;c<i; c++){
		state=vec_state[c];
		change_state();
		if(c!=0) delay_ms( (vec_tmp_chg[c]-vec_tmp_chg[c-1])/10); 
		else delay_ms( vec_tmp_chg[c]/10);
	}
	
	state=6; change_state();
	
	//reset flags and variables
	cont_time_rp=0;
	flag_450ms=0;
	flag_rp=0;
	i=0;
}
void change_state(){
	
	// L-left; M-middle; R-right
	// B-black; W-white
	// B=0; W=1
	
	switch(state) {
		
		case 0  : //1-0-1
			PORTC=0b00001001; leftMotor=VELMAX; rightMotor=VELMAX;
		return;
				
		case 1  : //0-0-0
			PORTC=0b00001001; leftMotor=VELMAX; rightMotor=VELMAX;
		return;
		
		case 2  : //0-1-1
			PORTC=0b00001001; leftMotor=VELMED; rightMotor=VELMAX;
		return;
		
		case 3  : //1-1-0
			PORTC=0b00001001; leftMotor=VELMAX; rightMotor=VELMED;
		return; 
		
		case 4  : //0-0-1 *
			PORTC=0b00001010; 
			leftMotor=VELMAX; rightMotor=VELMAX;
		return;
		
		case 5  : //1-0-0 *
			PORTC=0b00000101; 
			leftMotor=VELMAX; rightMotor=VELMAX;
		return;
		
		case 6  : //X finished mapping
			PORTC=0b00001001; leftMotor=0; rightMotor=0;
		return;
				
	}
	
}
unsigned int Cantor_PF(unsigned int a, unsigned int b){ //a-left motor, b-right motor
	
	//A function that encodes two natural numbers into a single natural number (reversible)
	unsigned int z = (unsigned int) ( ( (1/2) * (a + b) * (a + b + 1) ) + b );
	
	return z;
}
