/*
;************************************************************
;* PROJECT    : MIDI CONTROLLER FOR LINE 6 M9 PEDAL
;* FILE       : M9_MIDI_V1.ASM
;* VERSION    : 2.0
;* PROGRAMMER : Vanderson Pimenta Carvalho
;* EMAIL      : vandersonpc@gmail.com
;* LAST ATUAL.: 29/nov/2011
;* PROCESSOR  : PIC12F675
;************************************************************
;*  Midi foot controller for LINE 6 M9 guitar effect pedal
;*
;************************************************************
;* UPDATES:
;*
;* 13/Oct - v1.0 - Initial Release
;* 18/Oct - v1.1 - Include the Scene up down feature
;* 29/Oct - v1.2 - Impement the 4 folders mode scenes up down
;*
;*		RED - Folder #1
;*      GRN - Folder #2
;*      BLU - Folder #3
;*      PUR - Folder #4
;*      YLW - Bypass / Tuner
;*
;* 15/Nov - v1.3 - Fix Minor scenes selection issues
;*
;* 15/Nov - v1.4 - Change LEDs colors
;*
;* 15/Nov - v1.5 - Fix Tuner / Bypass MIDI message issue
;*
;* 15/Nov - v1.6 - Implement Active first scene at folder change
;*
;* 16/Nov - v1.7 - Fix problems on the scanes button latch
;*          v1.8 - Implement save to EEPROM scene.  
;*                 Press+Hold Sel btn for 5 seconds 
;*                  
;*
;* 25/Nov - v1.9 - Start the LOOP Control implementation
;*                 Play, Record and Overdub.
;*                 Overdub is activated if the system is Playing. 
;* 
;*      WHT - Looper Mode                    
;*
;*
;* 29/Nov - v2.0 - Fix the Overdube mode issue
;*
;************************************************************
;*/

#include <system.h>
#include <stdlib.h>

//======================================
// Version Control 
//======================================

#define Version 	  0x20
#define Day 		  0x29
#define Mouth 		  0x11
#define Year		  0x11

#pragma DATA 0x2105, Version
#pragma DATA 0x2106, 0xDA
#pragma DATA 0x2107, Day
#pragma DATA 0x2108, Mouth
#pragma DATA 0x2109, Year

//======================================
// Setting Configuration and ID Bits
//======================================

#pragma DATA    0x2007, _CPD_OFF & _CP_OFF & _BODEN_OFF & _MCLRE_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT
#pragma CLOCK_FREQ 4000000

//======================================
//Variables Declaration
//======================================

unsigned char	LED;				// LED Status
unsigned char   MODE;				// Operational Modes
unsigned char 	buttons;			// Buttons Flag
unsigned char   scn;	    		// Scene Variable
unsigned int    ecnt;	    		// Counter to Save to EEPROM
unsigned char   opt;                // Options to SAve to EEPROM       
volatile bit    EN_LOOP;            //
volatile bit    is_LOOP;              
//======================================
// Global Definitions
//======================================

// EEPROM Addresses


#define F1_ADDR			0
#define F2_ADDR			1
#define F3_ADDR			2
#define F4_ADDR			3
#define LEN_ADDR		4

#define EE_Time			500    // n*10 = seconds

// Shift-Registers Pins

volatile bit SR_DATAo@GPIO.1;
volatile bit SR_CLOCK@GPIO.2;


// Buttons Pins

volatile bit BTN_SEL@GPIO.5;
volatile bit BTN_B@GPIO.4;
volatile bit BTN_A@GPIO.3;

// Operational Modes

#define FLDR1			0
#define FLDR2			1
#define FLDR3			2
#define FLDR4			3
#define SETUP			4
#define LOOP            5

// Midi Definitions

/*

_ClkOut = FOSC / 4

BAUD_RATE(<19200) = (((_ClkOut / Baud) - 13) / 3)
BAUD_RATE(>19200) = (((_ClkOut / Baud) - 13) / 3) - 1

-----------------------------------------------------
          | 4800 | 9600 | 19200 | 38400 | MIDI(31250)
-----------------------------------------------------
BAUD_RATE |  65  |  30  |  12   |   3   |     5
-----------------------------------------------------
*/
#define BAUD_RATE			5
#define TX_PIN				0

#define MIDI_DATA_MASK		01111111b
#define MIDI_CHANNEL		0x00
#define MIDI_CONTROL_CHANGE	0xB0
#define MIDI_PROGRAM_CHANGE	0xC0
#define CONTROL_CHANGE_ADDR 0x7A


// M9 MIDI Definitions

const rom unsigned char *FOLDER1 = {0 , 1, 2, 3, 4, 5};
const rom unsigned char *FOLDER2 = {6 , 7, 8, 9,10,11};
const rom unsigned char *FOLDER3 = {12,13,14,15,16,17};
const rom unsigned char *FOLDER4 = {18,19,20,21,22,23};

#define SCENES				6

#define BYPASS              23
#define TURNER				69

#define L_CTRL  			86
#define L_Play_Stop         28
#define L_Rec_Over          50

#define Play				64		
#define Stop				0
#define Record				64
#define Overdub				0

// LED Definitions

#define rLED				0
#define gLED				1
#define bLED				2
#define LEDa			    4
#define LEDb				5
#define LED_DLY				1000


//=======================================
// Write the Values to EEPROM
//=======================================

void EEPROM_WRITE(unsigned char adresa,unsigned char data)  //write to EEPROM
{
eeadr  = adresa;
eedata = data;
clear_bit(intcon,GIE);		//Clear GIE
set_bit(eecon1,WREN); 		// Set WREN
eecon2 = 0x55;
eecon2 = 0xAA;
set_bit(eecon1,WR); 		// Set WR
clear_bit(eecon1,WREN); 	// clear WREN
while (test_bit(eecon1,1));
}

//=======================================
// Read the Values from EEPROM
//=======================================

unsigned char EEPROM_READ(char adresa)     // read from EEPROM
{
 asm
     {
    movf	_adresa,W 		 ;_EEPROM_WRITE
	bsf 	_status,RP0 	 ; Bank 1
    movwf	_eeadr	   		 ;ACERTA O ENDERE�O PARA LEITURA
    bsf 	_status,RP0 	 ; BANCO 1
	bsf	    _eecon1,RD	   	 ;PREPARA LEITURA
	movf	_eedata,W 		 ;COLOCA DADO EM W
	bcf 	_status,RP0      ;Bank 0
     }
return(eedata);
}

//=======================================
// Reset the EEPROM Values to Default
//=======================================

void RESET_EE(void)             // erase all EEPROM
{

unsigned char i=0;
do {
EEPROM_WRITE(i,i);
EEPROM_WRITE(i + 60,0);
}while (++i!=127);
}

//=======================================
// Send the display value to 4094
//=======================================

void spi_write(unsigned char data){

unsigned char cnt;

 cnt = 8;
 SR_DATAo = 0;
 nop();
 //SR_LATCH = 0;
 while (cnt != 0)
     {
       SR_DATAo = 0;
       asm rlf _data,F;
       if (test_bit(status,C) )
          {
           SR_DATAo = 1;
          }
       SR_CLOCK = 1;
       nop();
       SR_CLOCK = 0;
       cnt--;
     }

}

//=======================================
// Update LED Status
//=======================================

void display()
{
spi_write(LED);
}

//=======================================
//Internal Oscilator Calibration Routine
//=======================================

inline void init(void)
{
    asm
    {
        call 0x3FF
        bsf _status,RP0
        movwf _osccal
        bcf _status,RP0
    }
}


//=======================================
//Bit Delay for RS232 TX Routine
//=======================================

void BitDelay(void)
{
unsigned char count;

asm{
	movlw	BAUD_RATE	; move baud delay constant to W
	movwf	_count		; initialize delay counter

BitDelayLoop:
	decfsz	_count,F	; decrement delay counter
	goto	BitDelayLoop
}

}

//=======================================
// Send Char via RS232
//=======================================

void SendChar(unsigned char data)
{
  unsigned char count;

asm{
	movlw	0x08
	movwf	_count		; send 8 bits
	bcf		_gpio,TX_PIN		; set _TXPin for start bit
	nop
	nop
	nop
	nop
	call	BitDelay
SendNextBit:
	bcf     _status,C
	rrf     _data,F		; rotate TXReg
	btfsc   _status,C
	goto	_setTX

_clearTX:
	nop				; to get equal set/clear times
	bcf		_gpio,TX_PIN
	goto	_readyTX

_setTX:
	bsf		_gpio,TX_PIN
	goto	_readyTX

_readyTX:
	call    BitDelay		; was: DelayX !!!
	decfsz  _count,F		; decrement bit counter (8..0)
	goto    SendNextBit

	nop
	nop
	nop
	nop
	nop
	bsf    _gpio,TX_PIN		; send stop bit

	call  BitDelay

	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	call	BitDelay		; second stop bit to make sure..

	return

}

}

//=======================================
// Send a Program Change Message
//=======================================

void SendMidiPC(unsigned char PC_Value ){

SendChar(MIDI_PROGRAM_CHANGE + MIDI_CHANNEL);
SendChar(PC_Value);

}

//=======================================
// Send a Control Change Message
//=======================================

void SendMidiCC(unsigned char CC, unsigned char CC_Value){

SendChar(MIDI_CONTROL_CHANGE + MIDI_CHANNEL);
SendChar(CC);
SendChar(CC_Value);

}

//=======================================
// KeyCheck - Check the Pressed Key
//=======================================
void KeyCheck()
{
unsigned char tmpLED = 0;

if (BTN_A) {
   set_bit(buttons,0);
   if (MODE < SETUP) {
       if (!test_bit(LED,LEDa)){
       set_bit(LED,LEDa);clear_bit(LED,LEDb);
       nop();
       display();
       }
    }
    }
if (BTN_B) { 
   set_bit(buttons,1);
   if (MODE < SETUP) {
      if (!test_bit(LED,LEDb)){ 
      clear_bit(LED,LEDa);set_bit(LED,LEDb);
      nop();
      display();
      }
     }
    }
if (BTN_SEL) { 
    
    set_bit(buttons,2);
   
    // Check Button Pressed to save location to EEPROM
    ecnt++;
   if ((MODE < SETUP) && (ecnt >= EE_Time)) { 
    tmpLED = LED;
    clear_bit(buttons,2);
    set_bit(LED,LEDa);set_bit(LED,LEDb);
    nop();
    display();
    delay_ms(200);
    clear_bit(LED,LEDa);clear_bit(LED,LEDb);
    nop();
    display();
    delay_ms(200);
    set_bit(LED,LEDa);set_bit(LED,LEDb);
    nop();
    display();
    delay_ms(200);
    EEPROM_WRITE(MODE,scn);
    delay_ms(500);
    LED = tmpLED;
    display();
    ecnt=0; // Check Button Pressed disable the LOOP Mode
    }else if ((MODE == SETUP) && (ecnt >= EE_Time)) { 
    tmpLED = LED;
    clear_bit(buttons,2);
    set_bit(LED,LEDa);set_bit(LED,LEDb);
    nop();
    display();
    delay_ms(200);
    clear_bit(LED,LEDa);clear_bit(LED,LEDb);
    nop();
    display();
    delay_ms(200);
    set_bit(LED,LEDa);set_bit(LED,LEDb);
    nop();
    display();
    delay_ms(200);
    EN_LOOP = !EN_LOOP;
    
    if (EN_LOOP) {opt=1;}
    else {opt=0;}
    
    EEPROM_WRITE(LEN_ADDR,opt);
    delay_ms(500);
    LED = tmpLED;
    display();
    ecnt=0;
    }
   
  } else ecnt=0;


if ((!(BTN_A)) && (test_bit(buttons,0))) // UP
  {
  clear_bit(buttons,0);
  if (MODE != LOOP) clear_bit(LED,LEDb);
  if (MODE < SETUP) {
     clear_bit(LED,LEDa);
   } else LED ^= (1 << LEDa);

  if (MODE == FLDR1) {
  scn += 1;
  if (scn > SCENES ) { scn = 1;}
  SendMidiPC(FOLDER1[scn-1]);

  }

  if (MODE == FLDR2) {
  scn += 1;
  if (scn > SCENES ) { scn = 1;}
  SendMidiPC(FOLDER2[scn-1]);

  }

  if (MODE == FLDR3) {
  scn += 1;
  if (scn > SCENES ) { scn = 1;}
  SendMidiPC(FOLDER3[scn-1]);

  }

  if (MODE == FLDR4) {
  scn += 1;
  if (scn > SCENES ) { scn = 1;}
  SendMidiPC(FOLDER4[scn-1]);

  }

  if (MODE == SETUP)  {

     if (test_bit(LED,LEDa)) {SendMidiCC(TURNER,0);SendMidiCC(BYPASS,64);}
     else { SendMidiCC(BYPASS,0);}

  }
  
 if (MODE == LOOP)  {
 
   if (test_bit(LED,LEDa) && test_bit(LED,LEDb)) {SendMidiCC(L_CTRL,0);SendMidiCC(L_Rec_Over ,Overdub);is_LOOP=0;}
      else  if (test_bit(LED,LEDa)) {clear_bit(LED,LEDb);SendMidiCC(L_Rec_Over ,Record);}
            else {if (!is_LOOP){SendMidiCC(L_CTRL,64); is_LOOP=1;};clear_bit(LED,LEDb);SendMidiCC(L_Play_Stop ,Stop);} 

 } 
  
  display();
 }
if ((!(BTN_B)) && (test_bit(buttons,1))) // UP
  {
  clear_bit(buttons,1);
  clear_bit(LED,LEDa);
  if (MODE < SETUP) {clear_bit(LED,LEDb);}
      else LED ^= (1 << LEDb);

  if (MODE == FLDR1) {

   if (scn <= 1) { scn = SCENES;}
   else scn -= 1;
   SendMidiPC(FOLDER1[scn-1]);
   }

  if (MODE == FLDR2) {

   if (scn <= 1) { scn = SCENES;}
   else scn -= 1;
   SendMidiPC(FOLDER2[scn-1]);
   }

  if (MODE == FLDR3) {

   if (scn <= 1) { scn = SCENES;}
   else scn -= 1;
   SendMidiPC(FOLDER3[scn-1]);
   }

  if (MODE == FLDR4) {

   if (scn <= 1) { scn = SCENES;}
   else scn -= 1;
   SendMidiPC(FOLDER4[scn-1]);
   }

  if (MODE == SETUP)  {

     if (test_bit(LED,LEDb))  {SendMidiCC(BYPASS,0);SendMidiCC(TURNER,64);}
     else { SendMidiCC(TURNER,0);}

  }
  
  if (MODE == LOOP)  {
     
     if (test_bit(LED,LEDb))  {SendMidiCC(L_Play_Stop ,Play);}
         else {if (!is_LOOP){SendMidiCC(L_CTRL,64); is_LOOP=1;};SendMidiCC(L_Play_Stop ,Stop);}

  } 
  
   display();
}
if ((!(BTN_SEL)) && (test_bit(buttons,2))) // UP
  {
  clear_bit(buttons,2);

  MODE++;

  if (EN_LOOP) {if (MODE > LOOP) MODE = FLDR1;}
     else {if (MODE > SETUP) MODE = FLDR1;}
     
  LED = 0 ;
  ecnt= 0;
  
  if (is_LOOP) {is_LOOP = 0 ; SendMidiCC(L_CTRL,0);};
// SendMidiCC(L_CTRL,64);
 // SendMidiCC(TURNER,0);
 // SendMidiCC(BYPASS,0);

  scn = EEPROM_READ(MODE); 
  if ((scn==0) || (scn>10)) scn=1;
  if (MODE == FLDR1) {set_bit(LED,rLED);SendMidiPC(FOLDER1[scn-1]);}
  if (MODE == FLDR2) {set_bit(LED,gLED);SendMidiPC(FOLDER2[scn-1]);}
  if (MODE == FLDR3) {set_bit(LED,bLED);SendMidiPC(FOLDER3[scn-1]);}
  if (MODE == FLDR4) {set_bit(LED,rLED); set_bit(LED,bLED);SendMidiPC(FOLDER4[scn-1]);}
  if (MODE == SETUP) {set_bit(LED,rLED); set_bit(LED,gLED);set_bit(LED,bLED);}
  if (MODE == LOOP)  {set_bit(LED,rLED); set_bit(LED,gLED);SendMidiCC(L_CTRL,64);is_LOOP=1;}
  display();
  }

}
/*==============================
    Handle the Interruption
==============================*/

void interrupt(void)
{
    if( intcon & ( 1 << T0IF)){    // TMR0 overflow interrupt
        tmr0 = 157;                // -157 generates a 10ms Interruption
        clear_bit(intcon,T0IF);
        KeyCheck();
             
    }

}

//=======================================
//Main Routine
//=======================================

void LED_Test()
{

set_bit(LED,LEDa);
nop();
display();
delay_ms(LED_DLY);
LED=0;
set_bit(LED,LEDb);
nop();
display();
delay_ms(LED_DLY);
LED=0;
set_bit(LED,rLED);
nop();
display();
delay_ms(LED_DLY);
LED=0;
set_bit(LED,gLED);
nop();
display();
delay_ms(LED_DLY);
LED=0;
set_bit(LED,bLED);
nop();
display();
delay_ms(LED_DLY);
LED=0;
set_bit(LED,rLED);
set_bit(LED,bLED);
nop();
display();
delay_ms(LED_DLY);
LED=0;
set_bit(LED,bLED);
set_bit(LED,gLED);
nop();
display();
delay_ms(LED_DLY);

}


//=======================================
//Main Routine
//=======================================

void main()
{
    option_reg  = 10000101b;    		//set prescaler to 1:8 this means 125000 pulses per second and weak pull-up enable
    trisio      = 11111000b;    		//set GP0,GP1,GP3 as input ,others output
    cmcon       = 00000111b;    		//set GP0,GP1,GP2 as a digital IO
    init();                     		//Calibration
    ansel		= 00100000b;   			//Enable fosc/32
	gpio		= 0;
	SR_DATAo	= 0;
	scn         = 0;
	is_LOOP     = 0;
    buttons     = 0;
    tmr0  		= 157;
    intcon		= 10100000b;
    
    if (BTN_SEL) { 
      
       EEPROM_WRITE(FLDR1,1);
       EEPROM_WRITE(FLDR2,1);
       EEPROM_WRITE(FLDR3,1);
       EEPROM_WRITE(FLDR4,1);
       EEPROM_WRITE(LEN_ADDR,1);
       set_bit(LED,LEDa);
	   set_bit(LED,LEDb);
       display();
       delay_s(2);
    }
   
   // Check if LOOP Mode is Enabled
    
    opt = EEPROM_READ(LEN_ADDR);
    if (opt > 2) {opt=1;EEPROM_WRITE(LEN_ADDR,1);}
    
    if (test_bit(opt,0)) { EN_LOOP = 1;}
       else {EN_LOOP = 0;}
   
   //
    
    LED_Test();
    LED 		= 0;
    set_bit(LED,bLED);
	set_bit(LED,gLED);
    display();
    SendChar(0);
    MODE	    = SETUP;
    set_bit(intcon,GIE); 				// Set GIE - Enable all Interrupts
    while(1)
    {
	nop();
    }
}
