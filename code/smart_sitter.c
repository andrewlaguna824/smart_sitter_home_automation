
#include "trtSettings.h"
#include "trtkernel_1284.c"
#include <stdio.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <util/delay.h>

// serial communication library
#define SEM_RX_ISR_SIGNAL 1
#define SEM_STRING_DONE 2 // user hit <enter>
#include "trtUart.h"
#include "trtUart.c"
#include "lcd_lib.h"
// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// semaphore to protect shared variable
#define SEM_SHARED 5

// the usual
#define begin {
#define end }


//LCD display buffer
int8_t lcd_buffer[17];     //initialize buffer for displaying messages on LCD
volatile int alert;
volatile int count;
volatile int time;

volatile int temperature = 0;
volatile int smoke = 0;

//uart FSM variables:
#define start 1
#define motionAlert 2
#define writeToUART 3
#define wait 4
unsigned char UARTstate;
unsigned char uartAlert;
int x;
int startUp; 

//fire alert FSM variableS:
#define startFire 9
#define fireDetected 10
#define fireStall 11
#define fireWait 12
unsigned char fireState;
unsigned char fireAlert;
unsigned char smokeAlert;
int a;


//initializing LCD FSM variables
#define initializing 5
#define noAlert 6
#define alertOut 7
int alertLCD; 
int delay;
unsigned char alertState;
int peltierOn;

char AinSmoke ; 		//raw A to D number
char AinTemp ;         //raw A to D number

//ADDED:

// --- external interrupt ISR ------------------------
//INT0 attached to motion sensor output!!
ISR (INT0_vect) {
    
}

#define countMS 62  //ticks/mSec

ISR (TIMER0_OVF_vect)
begin
   
    // generate time base for MAIN
    // 62 counts is about 1 mSec
    count--;
    if (0 == count )
    begin
        count=countMS;
        time++;    //in mSec
    end 
end

int args[2] ;
//Task to set Peltier Current direction based on desired temperature value
void peltier(void* args)
{
	uint32_t rel, dead;
	
	while(1)
	{
		
		if (peltierOn) // Command to control temperature received

		{
			if (temperature > AinTemp) // Decrease Temperature
			{
				PORTD = 0x18;
			}
			else if (temperature < AinTemp) // Increase Temperature
			{
				PORTD = 0x28;
			}
			else // Once desired temperature is reached, turn Peltier off
			{
				peltierOn = 0;
				PORTD = 0x00;
			}
		}


		// Sleep
        rel = trtCurrentTime() + SECONDS2TICKS(0.15);
        dead = trtCurrentTime() + SECONDS2TICKS(0.4);
        trtSleepUntil(rel, dead);
	}
}

//Task to read and convert temperature value
void tempSensor(void* args)
{
	uint32_t rel, dead;
	
	while(1)
	{
		trtWait(SEM_SHARED) ; 

		ADMUX |= (1 << REFS0); 
		ADMUX |= (1 << REFS1); // Set ADC ref to 2.56V
    	ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading
		
		ADMUX = (ADMUX & 0xF8)|1; // Port A1 for temp input
        
        
        //enable ADC and set prescaler to 1/128*16MHz=125,000
        //and clear interupt enable
        //and start a conversion
		ADCSRA |= (1<<ADEN) | (1<<ADSC) + 7 ;
		
		while (ADCSRA & (1<<ADSC));

		AinTemp = ADCH - 3;

		trtSignal(SEM_SHARED);
		
		// Sleep
        rel = trtCurrentTime() + SECONDS2TICKS(0.15);
        dead = trtCurrentTime() + SECONDS2TICKS(0.4);
        trtSleepUntil(rel, dead);

	}
}

// Task to read and convert smoke analog signal
void smokeSensor(void* args)
  begin   
      uint32_t rel, dead ;

    while(1)
    begin
	
	trtWait(SEM_SHARED) ;

   ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading
   ADMUX |= (1 << REFS0);
   ADMUX |= (1 << REFS1); // Set ADC ref to 2.56V

   ADMUX |= 0&0b00000111;
    
   //enable ADC and set prescaler to 1/128*16MHz=125,000
   //and clear interupt enable
   //and start a conversion
   ADCSRA |= (1<<ADEN) | (1<<ADSC) + 7;

	while (ADCSRA & (1<<ADSC));
	//get the sample  
 	AinSmoke = ADCH;
 	//start another conversion

	// FSM for the UART update

	trtSignal(SEM_SHARED);	

	if (fireAlert == 1) {
		if (a == 0 ) {
		fprintf(stdout, "RUN FIRE!\n");
		a++;
		smokeAlert = 1;
		}
	}
	else if (fireAlert == 0) {
		a= 0;
	}


	fireState= startFire;
	switch (fireState) 
	begin
		case startFire:
		{
			if (AinSmoke < 75 ) 
			{
				fireAlert = 1;
			}
			else
			{
				fireAlert = 0;
			}
		 fireState= fireWait;
			break;
		}
		case fireDetected: 
		{
			fireAlert= 1;
			fireState = fireStall;
			break;
		}
		case fireStall:
		{
			fireAlert= 0;
			if (AinSmoke < 2 ) {
				fireState = fireWait;
			}
			else  {
				fireState = fireStall;
			}
			break;
		}
		case fireWait:
		{ 
			fireAlert = 0;
			if (AinSmoke < 75 ) {
				fireState = fireDetected;
			}
			else {
				fireState = fireWait;
			}
			break;
		}
	end 
	

	
	
	
	
	
	
       // Sleep
        rel = trtCurrentTime() + SECONDS2TICKS(0.1);
        dead = trtCurrentTime() + SECONDS2TICKS(0.3);
        trtSleepUntil(rel, dead);
    end
  end

// Task to control serial communication based on state
void serialComm(void* args)
  begin
    char cmd[1] ;
    int in;

    while(1)
    begin

        fprintf(stdout, ">") ;
        fscanf(stdin, "%s %d", cmd, &in) ;
       
        // update shared variables inputted from user
        trtWait(SEM_SHARED) ; 
		  
        if (cmd[0] == 't') {

			peltierOn = 1; // turn on peltier device
            temperature = (float) (in);
            fprintf(stdout, "you entered temperature\n");
        }
        else if (cmd[0] == 's') {

			if (alert == 1 ) {
				fprintf(stdout, "Motion Detector: INTRUDER DETECTED\n");
			}
			else {
				fprintf(stdout, "Motion Detector: No Intruders :)\n");
			}
			if (smokeAlert == 1 ) {
				fprintf(stdout, "Smoke Detector: Smoke Detected\n");
				smokeAlert = 0; 	
			}
			else {
				fprintf(stdout, "Smoke Detector: No Smoke!\n");	
			}

			fprintf(stdout, "Temperature: %d\n", AinTemp);
			
		alert = 0;
        }   
        trtSignal(SEM_SHARED);

    end
  end
 
  // --- define task 3  ----------------------------------------
void updateLCD(void* args)
  begin   
      uint32_t rel, dead ;

    while(1)
    begin
    trtWait(SEM_SHARED) ;
	
	LCDGotoXY(0,0); // go to first line of LCD
	LCDsendChar('T');
	LCDsendChar('E');
	LCDsendChar('M');
	LCDsendChar('P');
	LCDsendChar(' ');
	LCDsendChar('=');
	LCDsendChar(' ');
	
	 sprintf(lcd_buffer, "%2d", AinTemp);
     LCDstring(lcd_buffer, strlen(lcd_buffer)); 

    if (!(PIND & 0x04))
	{
        LCDGotoXY(0,1);
        LCDsendChar(' ');
        LCDsendChar(' ');
        LCDsendChar(' ');
        LCDsendChar(' ');
        LCDsendChar(' ');
        LCDsendChar(' ');

    }
    if (PIND & 0x04)
    {
        LCDGotoXY(0,1);
        LCDsendChar('A');
        LCDsendChar('L');
        LCDsendChar('E');
        LCDsendChar('R');
        LCDsendChar('T');
        LCDsendChar('!');
		
		alert = 1;	
    }
	if (fireAlert == 1)
	{
		LCDGotoXY(8,1);
        LCDsendChar('S');
        LCDsendChar('M');
        LCDsendChar('O');
        LCDsendChar('K');
        LCDsendChar('E');
        LCDsendChar('!');
	}
	if (fireAlert == 0)
	{
		LCDGotoXY(8,1);
        LCDsendChar(' ');
        LCDsendChar(' ');
        LCDsendChar(' ');
        LCDsendChar(' ');
        LCDsendChar(' ');
        LCDsendChar(' ');
	}


	if (uartAlert == 1) {
		if (x == 0 ) {
			fprintf(stdout, "Motion Sensor Alert!\n");
			x++;
		}
	}
	else if (uartAlert == 0) {
		x= 0;
	}

	// FSM for the UART update
	UARTstate= start;
	switch (UARTstate) 
	begin
		case start:
		{
			if (PIND & 0x04 ) 
			{
				uartAlert = 1;
			}
			else if (!(PIND & 0x04))
			{
				uartAlert = 0;
			}
			UARTstate = wait;
			break;
		}
		case motionAlert: 
		{
			uartAlert= 1;
			UARTstate = writeToUART;
			break;
		}
		case writeToUART:
		{
			uartAlert= 0;
			if (!(PIND & 0x04) ) {
				UARTstate = wait;
			}
			else {
				UARTstate= writeToUART;
			}
			break;
		}
		case wait:
		{ 
			uartAlert= 0;
			if ( PIND & 0x04 ) {
				UARTstate = motionAlert;
			}
			else {
				UARTstate = wait;
			}
			break;
		}
	end 

    trtSignal(SEM_SHARED);  
       
        // Sleep
        rel = trtCurrentTime() + SECONDS2TICKS(0.15);
        dead = trtCurrentTime() + SECONDS2TICKS(0.25);
        trtSleepUntil(rel, dead);
    end
  end
 
  //LCD setup-init LCD
void init_lcd(void)
{
    LCDinit();                    //initializes LCD
    LCDcursorOFF();               //turns of LCD cursor
    LCDclr();                     //clears LCD
    LCDGotoXY(0, 0);              //Moves LCD to point (0,0)
}//end lcd init function


// --- Main Program ----------------------------------
int main(void) {
  DDRB = 0xff;
  // set D to input
  DDRD = 0x38;

  //set up LCD
  init_lcd();


   // init the time counter
   time=0;

   // timer 0 runs at full rate
   TCCR0B = 1 ; 
  
   //turn on timer 0 overflow ISR
   TIMSK0 = (1<<TOIE0) ;


 	//ADDED:
    //set up INT0
    EIMSK = 1<<INT0 ; // turn on int0
    EICRA = 3 ;       // rising edge
    //EICRA = 1 ;       // any edge
	sei();



  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"\n\r TRT 9feb2009\n\r\n\r");

  // start TRT
  trtInitKernel(80); // 80 bytes for the idle task stack

  // --- create semaphores ----------
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>
 
  // variable protection
  trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variables

 // --- create tasks  ----------------
  trtCreateTask(smokeSensor, 200, SECONDS2TICKS(0.1), SECONDS2TICKS(0.3), &(args[0]));
  trtCreateTask(serialComm, 400, SECONDS2TICKS(0.1), SECONDS2TICKS(0.4), &(args[1]));
  trtCreateTask(updateLCD, 200, SECONDS2TICKS(0.15), SECONDS2TICKS(0.25), &(args[2]));
  trtCreateTask(tempSensor, 200, SECONDS2TICKS(0.15), SECONDS2TICKS(0.4), &(args[3]));
  trtCreateTask(peltier, 200, SECONDS2TICKS(0.15), SECONDS2TICKS(0.4), &(args[4]));
 
  // --- Idle task --------------------------------------
  // just sleeps the cpu to save power
  // every time it executes
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  while (1)
  begin
      sleep_cpu();
  end

} // main

