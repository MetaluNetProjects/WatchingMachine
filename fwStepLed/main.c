/*********************************************************************
 *
 *                Moteur pas à pas 
 *
 *********************************************************************
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Antoine Rousseau may 17 2018     Original.
 ********************************************************************/

/*
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
# MA  02110-1301, USA.
*/

#define BOARD Step1.2

#include <fruit.h>
#include <eeparams.h>
//#include <ramp.h>
//#include <servo.h>
#include <analog.h>
//#include <switch.h>


//definition des LED
//#define LED1 K1				

#define INTPIN KINT(DETECTOR)
//#define INTPIN 2
#include <intpin.h>

//-------------  Timer1 macros :  ---------------------------------------- 
//prescaler=PS fTMR1=FOSC/(4*PS) nbCycles=0xffff-TMR1init T=nbCycles/fTMR1=(0xffff-TMR1init)*4PS/FOSC
//TMR1init=0xffff-(T*FOSC/4PS) ; max=65536*4PS/FOSC : 
//ex: PS=8 : T=0.01s : TMR1init=0xffff-15000
//Maximum 1s !!
#define	TMR1init(T) (0xffff-((T*FOSC)/32000)) //ms ; maximum: 8MHz:262ms 48MHz:43ms 64MHz:32ms
#define	TMR1initUS(T) (0xffff-((T*FOSC)/32000000)) //us ; 
#define InitTimer(T) do{ TMR1H=TMR1init(T)/256 ; TMR1L=TMR1init(T)%256; PIR1bits.TMR1IF=0; }while(0)
#define InitTimerUS(T) do{ TMR1H=TMR1initUS(T)/256 ; TMR1L=TMR1initUS(T)%256; PIR1bits.TMR1IF=0; }while(0)
#define TimerOut() (PIR1bits.TMR1IF)

unsigned char t,t2=0;

volatile unsigned char MotorOnState=0; // 4 msb = "on" state of MOTx_PINs
volatile long int MotorPos=0;
volatile long int MotorPosOnDetect = 0;
long int oldMotorPosOnDetect = 0;
long int MotorPosConsign=0; 

#define MotorPosTmpConsign MotorPosTmpConsignU.L0
volatile int MotorFracPos;	//fractionnal part of position
unsigned char MotorMaxSpeed=10;
int MotorSpeed=0;
unsigned char MotorMaxCurrent=40;
//unsigned char MotorMaxCurrentOrder=40;
unsigned char Hold=0;
unsigned char MotorStandbyCurrent=2; //5
//#define MotorCurrent CCPR2L
#define setCurrent(val) do { analogWrite(MAI, val); analogWrite(MBI, val);} while(0)
unsigned int MotorStopedCount; // since how long the motor is stopped ?
const unsigned char MotorPhaseStates[8]={
	/*0001*/ 1,
	/*0101*/ 5,
	/*0100*/ 4,
	/*0110*/ 6,
	/*0010*/ 2,
	/*1010*/ 10,
	/*1000*/ 8,
	/*1001*/ 9 };

#define UpdateMotorOnState() MotorOnState=MotorPhaseStates[(unsigned char)MotorPos&0x07]

/*#define SET_PWM(pwm,val) do{ CCP##pwm##CONbits.DC##pwm##B1 = val&2;  CCP##pwm##CONbits.DC##pwm##B0 = val&1; CCPR##pwm##L=val>>2; } while(0)*/

void Step_ISR(void)
{	
	// Do we have a Timer2 interrupt ? (366us rate) -> avance pas 2.7kHz
	if (PIR1bits.TMR2IF)
	{
		// Clear the interrupt 
		PIR1bits.TMR2IF = 0;
		if(MotorPosConsign==MotorPos) {	// si la position n'a pas changé depuis plus de 1500*366us=500ms, 
				        
			if(Hold==0){			// si la consigne Hold est inactive, 
								// baisser le courant
				//MotorSpeed=0;
				if(MotorStopedCount>1500) { setCurrent(MotorStandbyCurrent); }
				else MotorStopedCount++;
			}
		}
		else {
			setCurrent(MotorMaxCurrent);
			MotorStopedCount=0;
			if(MotorPosConsign<MotorPos) {
				MotorFracPos-=MotorMaxSpeed;
				if(MotorFracPos<-256) {
					MotorFracPos+=256;
					MotorPos--;
					UpdateMotorOnState();
				}
			}
			else {
				MotorFracPos+=MotorMaxSpeed;
				if(MotorFracPos>256) {
					MotorFracPos-=256;
					MotorPos++;
					UpdateMotorOnState();
				}
			}	
		}
	}

	// Do we have a Comparator interrupt ?
	if(PIR2bits.C1IF)
	{
		PIR2bits.C1IF=0;
		if(CM1CONbits.C1OUT) {
			digitalClear(MA1); 
			digitalClear(MA2); 
			PIE2bits.C1IE=0;	// Disable the comparator interrupt
		}		
	}
	if(PIR2bits.C2IF)
	{
		PIR2bits.C2IF=0;
		if(CM2CONbits.C2OUT) {
			digitalClear(MB1); 
			digitalClear(MB2); 
			PIE2bits.C2IE=0;	// Disable the comparator interrupt
		}		
	}
	
	// Do we have a Timer1 interrupt? 23kHz
	if (PIR1bits.TMR1IF)
	{
		// Clear the interrupt 
		PIR1bits.TMR1IF = 0;
		
		PIR2bits.C1IF=0;	
		PIR2bits.C2IF=0;
		
		digitalWrite(MA1, MotorOnState & 1); 
		digitalWrite(MA2, MotorOnState & 2); 
		digitalWrite(MB1, MotorOnState & 4); 
		digitalWrite(MB2, MotorOnState & 8);
		
		if(CM1CONbits.C1OUT) {
			digitalClear(MA1); 
			digitalClear(MA2); 
		} else PIE2bits.C1IE=1;	// Enable the comparator interrupt
			
		if(CM2CONbits.C2OUT) {
			digitalClear(MB1); 
			digitalClear(MB2); 
		} else PIE2bits.C2IE=1;	// Enable the comparator interrupt
		
		TMR1H = 255;
		TMR1L = 0;	
	}
	
	if(INTPIN_IF){
		INTPIN_IF = 0;
		MotorPosOnDetect = MotorPos;
	}
}


void highInterrupts()
{
	Step_ISR();

}


unsigned char PERIOD = 25, t = 0;
t_delay mainDelay;

void setup(void)
{
	fruitInit();
	
	pinModeAnalogOut(LEDPWM);
	
	digitalClear(MA1);
	digitalClear(MA2);
	digitalClear(MB1);
	digitalClear(MB2);

	digitalSet(MAEN);
	digitalSet(MBEN);

	pinModeDigitalOut(MA1);	
	pinModeDigitalOut(MA2);	
	pinModeDigitalOut(MAEN);	

	pinModeDigitalOut(MB1);	
	pinModeDigitalOut(MB2);	
	pinModeDigitalOut(MBEN);	


	//---------- Step motor init : --------------
	
		/*Current PWM : */
	/*TRISCbits.TRISC1=0;
	CCP2CON=0x0f; //PWM mode; 
	PR2=255; //without prescaler:47kHz
	CCPR2L=0;*/
	pinModeAnalogOut(MAI);
	pinModeAnalogOut(MBI);

	T2CONbits.TMR2ON = 1;
	T2CONbits.T2OUTPS3 =		//
	T2CONbits.T2OUTPS2 =		//	postscaler=16 -> T2 interrupt = 3kHz
	T2CONbits.T2OUTPS1 =		//
	T2CONbits.T2OUTPS0 = 1;	//

	IPR1bits.TMR2IP=1;	// High priority
	PIR1bits.TMR2IF=0;	// Clear the interrupt flag
	PIE1bits.TMR2IE=1;	// Enable the interrupt
	
	/*TMR1*/
	T1CONbits.TMR1ON = 1;
	//T0CONbits.PSA = 0;		// Use the prescaler
	T1CONbits.TMR1CS = 0;		// Use internal clock
	//T0CONbits.T08BIT = 1;	// 8 bit timer

	T1CONbits.T1CKPS0 = 1;	// 
	T1CONbits.T1CKPS1 = 0;	// 1/2 prescaler: 48/4/2/256=23kHz
	TMR1H = 255;
	TMR1L = 0;

	IPR1bits.TMR1IP = 1;	// Hi priority
	//INTCON2bits.TMR0IP = 0;	// Low priority
	PIR1bits.TMR1IF = 0;	// Clear the interrupt flag
	PIE1bits.TMR1IE = 1;	// Enable the interrupt

	/*COMPARATORS*/
	CM1CONbits.C1POL=1;	// invert 
	CM1CONbits.C1ON=1;	// on 
	CM1CONbits.C1CH=0;	// chan0 
	CM2CON1bits.C1HYS = 0; // activate hysteresis
	
	
	CM2CONbits.C2POL=1;	// invert 
	CM2CONbits.C2ON=1;	// on 
	CM2CONbits.C2CH=1;	// chan1 
	CM2CON1bits.C2HYS = 0; // activate hysteresis

	PIR2bits.C1IF=0;	// Clear the interrupt flag
	PIE2bits.C1IE=0;	// Enable the interrupt
	IPR2bits.C1IP=1;	// High priority*/
	PIR2bits.C2IF=0;	// Clear the interrupt flag
	PIE2bits.C2IE=0;	// Enable the interrupt
	IPR2bits.C2IP=1;	// High priority*/
	
//------------------------------------------
	
	
	
// ------- init PWM1 to pulse MOTC_IN1
//	PSTR1CON=0;
//	PSTR1CONbits.STR1D=1;
	
// ---------- Main Timer on TMR3 -------------
	
/*	T3CONbits.T3CKPS0=1; //
	T3CONbits.T3CKPS1=1; //prescaler 8 : f=fosc/(4*8)=64MHz/32=2MHz : Troll=32ms->30Hz
	T3CONbits.RD16=1; // 16bits
	T3CONbits.TMR3ON=1;
	PIE2bits.TMR3IE=1;	// enable TMR3 interrupt
	IPR2bits.TMR3IP=1;	// high priority
*/	
// --------- Analog -------------------
	analogInit();
	//analogSelect(0,K1);	// assign connector K1 to analog channel 0

// --------- Interrupt pin ------------
	INTCON2bits.RBPU = 0; // PORTB pull-ups on
	pinModeDigitalIn(DETECTOR);
	INTPIN_EDGE = 0;
	INTPIN_IP = 1; // high priority
	INTPIN_IF = 0; // clear flag
	INTPIN_IE = 1; // enable interrupt

  /*GLOBAL INTERRUPTS*/
	/*RCONbits.IPEN = 1;		// Enable interrupt priorities
	INTCONbits.GIEH = 1;	// Turn high priority interrupts on
	INTCONbits.GIEL = 1;	// Turn low priority interrupts on*/
	EEreadMain();

	delayStart(mainDelay, 5000); 	// init the mainDelay to 5 ms
}

void loop() {
		 
	analogService();
	fraiseService();
	
	if(delayFinished(mainDelay)) // when mainDelay triggers :
	{
		delayStart(mainDelay, 5000); 	// re-init mainDelay

		if(MotorPosOnDetect != oldMotorPosOnDetect) {
			oldMotorPosOnDetect = MotorPosOnDetect;
			putchar('B');
			putchar(2);
			putchar(oldMotorPosOnDetect>>24);
			putchar(oldMotorPosOnDetect>>16);
			putchar(oldMotorPosOnDetect>>8);
			putchar(oldMotorPosOnDetect);
			putchar('\n');
		}
			
		if(!t--){
			t=PERIOD;
			//analogSend();
		
			//printf("C M %d %d\n",MotorPos,MotorPosConsign);
			putchar('B');
			putchar(1);
			putchar(MotorPos>>24);
			putchar(MotorPos>>16);
			putchar(MotorPos>>8);
			putchar(MotorPos);
 			putchar(MotorPosConsign>>24);
			putchar(MotorPosConsign>>16);
			putchar(MotorPosConsign>>8);
			putchar(MotorPosConsign);
			putchar('\n');
			

           /*    printf("C V %d %d\n",MotorCurrent,MotorStandbyCurrent);*/
			//printf("C M %ld %ld\n",MotorPos,MotorPosConsign);
				//if(t2%2) printf("C MC %ld %d %d\n",PosC,SpeedConsignC,MotorC.Status.homed);
				//else printf("C MD %ld %d %d\n",PosD,SpeedConsignD,MotorD.Status.homed);
		}		
	}
}


void fraiseReceiveChar()
{
	unsigned char c;
	
	c = fraiseGetChar();
	if(c == 'L'){		//switch LED on/off 
		c = fraiseGetChar();
		digitalWrite(LED, c!='0');		
	}
	else if(c=='E') { 	// echo text (send it back to host)
		printf("C");
		c = fraiseGetLen(); 			// get length of current packet
		while(c--) printf("%c",fraiseGetChar());// send each received byte
		putchar('\n');				// end of line
	}	
}


void fraiseReceive()
{
	unsigned char c ,c2;
	unsigned int i;
	long pos;
	
	c=fraiseGetChar();

	switch(c) {
		//PARAM_CHAR(1,t2); break;
		PARAM_CHAR(2,PERIOD); break;
		
		PARAM_CHAR(3, c2);
			if(c2 == 255) {
				Hold = 1;
			} else if(c2 == 254) {
				Hold = 0;
			} else {
				MotorMaxCurrent = c2;
			}
			break;
		PARAM_CHAR(4,MotorMaxSpeed); break;
		PARAM_LONG(5,MotorPosConsign); break;
		PARAM_LONG(6,pos); __critical { MotorPosConsign=MotorPos=pos; } ; break;
		PARAM_INT(10 , i); analogWrite(LEDPWM, i) ; break;
		//case 255 : EEwriteMain();break;
	}
}

void EEdeclareMain()
{
}
