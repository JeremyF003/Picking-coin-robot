// UPDATE LOG
// v1.0 - april 7th 5:13 pm
// From JFG003's computer

#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

volatile unsigned int servo_counter=0;
volatile unsigned int motor_counter=0;
volatile unsigned char servo1=0, servo2=0;
volatile unsigned char Rwheel = 0;
volatile unsigned char Lwheel = 0;

long int f;
long int base_freq;
long int diff;


static bit automaticMode = 1;

static bit toggleHandled = 0;  


//was 150 for both


//////////////////
// LCD PINS //////
//////////////////
#define LCD_RS P2_6
#define LCD_E P3_0
#define LCD_D4 P1_3
#define LCD_D5 P1_2
#define LCD_D6 P1_1
#define LCD_D7 P1_0
#define CHARS_PER_LINE 16

//////////////////
/// LED PINS /////
//////////////////

#define LED_RIGHT P3_1
#define LED_LEFT P3_2

//////////////////////////
// SERVO MOTORS PINS  ////
//////////////////////////
#define SERVO1     P1_6
#define SERVO2     P1_7

///////////////////////
//PERIOD MEASUREMENT //
///////////////////////

#define PERIOD_PIN P1_5

///////////////////////
//H-BRIDGE OUTPUTS  ///
//////////////////////
#define OUTPIN1    P2_1
#define OUTPIN2    P2_2


#define OUTPIN3    P2_3
#define OUTPIN4    P2_4


////////////////////
//ELECTROMAGNET   //
////////////////////

#define OUTPIN5    P2_5


#define BOOT       P3_7


/////////////////////////////
//Ultra Sonic Sensor ////////
/////////////////////////////

#define LED P0.6
//#define TRIG P1_3
//#define ECHO P1_4
//

//

#define SYSCLK 72000000L // SYSCLK frequency in Hz
#define BAUDRATE 115200L
#define SARCLK 18000000L
#define RELOAD_10us (0x10000L-(SYSCLK/(12L*100000L))) // 10us rate

idata char buff[20];
idata char flagbuff[10];
idata char lcdbuff[17];

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN |= 0x80;
	RSTSRC = 0x02;

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	// Configure the pins used as outputs
	P1MDOUT |= 0b_1100_0000; // SERVO2, SERVO1, OUTPUT1 to OUTPUT5
	// servos are P1_6, P1_7
	P0MDOUT |= 0b_0001_0000; // Configure UART0 TX (P0.4) as push-pull output
	P2MDOUT |= 0x01; // P2.0 in push-pull mode
	//P2MDOUT |= 0x02; // Set P2.1 as output (OUTPIN2)
	P2MDOUT |= 0b_0001_1110;
	
	//////////////////////////////////////////////////////////////////
	P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
	P2MDOUT |= 0x40;  // Set P2.6 (0x40) as push-pull
    P3MDOUT |= 0x01;  // Set P3.0 (0x01) as push-pull
    
    P3MDOUT |= 0x06;  // 0x06 sets P3.1 and P3.2 as output (binary: 00000110)
	//////////////////////////////////////////////////////////////////
	
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00; // 
	XBR2     = 0x41; // Enable crossbar and weak pull-ups

	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	// Configure Uart 0
	SCON0 = 0x10;
	CKCON0 |= 0b_0000_0000 ; // Timer 1 uses the system clock divided by 12.
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready

	// Initialize timer 5 for periodic interrupts
	SFRPAGE=0x10;
	TMR5CN0=0x00;
	TMR5=0xffff;   // Set to reload immediately
	EIE2|=0b_0000_1000; // Enable Timer5 interrupts
	TR5=1;         // Start Timer5 (TMR5CN0 is bit addressable)
	
	EA=1;
	
	SFRPAGE=0x00;
	
	P2_0=1; // 'set' pin to 1 is normal operation mode.
	
	return 0;
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
	unsigned char mask;
	
	mask=1<<pin_num;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

void Timer5_ISR (void) interrupt INTERRUPT_TIMER5
{
	SFRPAGE=0x10;
	TF5H = 0; // Clear Timer5 interrupt flag
	TMR5RL=RELOAD_10us;
	servo_counter++;
	motor_counter++;
	if(servo_counter == 2000) // Currently at 20ms
	{
		servo_counter=0;
	}
	
	if(motor_counter ==250)
	{
	  motor_counter = 0;
	}
	
	if(servo1>=servo_counter)
	{
		SERVO1=1;
	}
	else
	{
		SERVO1=0;
	}
	
	if(servo2>=servo_counter)
	{
		SERVO2=1;
	}
	else
	{
		SERVO2=0;
	}
	
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}

// Measure the period of a square signal at PERIOD_PIN
unsigned long GetPeriod (int n)
{
	unsigned int overflow_count;
	unsigned char i;
	
	TR0=0; // Stop Timer/Counter 0
	TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer

	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1;
	while(PERIOD_PIN!=0) // Wait for the signal to be zero
	{
		if(TF0==1) // Did the 16-bit timer overflow?
		{
			TF0=0;
			overflow_count++;
			if(overflow_count==10) // If it overflows too many times assume no signal is present
			{
				TR0=0;
				return 0; // No signal
			}
		}
	}
	
	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1;
	while(PERIOD_PIN!=1) // Wait for the signal to be one
	{
		if(TF0==1) // Did the 16-bit timer overflow?
		{
			TF0=0;
			overflow_count++;
			if(overflow_count==10) // If it overflows too many times assume no signal is present
			{
				TR0=0;
				return 0; // No signal
			}
		}
	}
	
	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1; // Start the timer
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while(PERIOD_PIN!=0) // Wait for the signal to be zero
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
			}
		}
		while(PERIOD_PIN!=1) // Wait for the signal to be one
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
			}
		}
	}
	TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period in clock cycles!
	
	return (overflow_count*65536+TH0*256+TL0);
}

void eputs(char *String)
{	
	while(*String)
	{
		putchar(*String);
		String++;
	}
}

void PrintNumber(long int val, int Base, int digits)
{ 

	code const char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS 32
	xdata char buff[NBITS+1];
	buff[NBITS]=0;
	
	if(val<0)
	{
		putchar('-');
		val*=-1;
	}

	j=NBITS-1;
	while ( (val>0) | (digits>0) )
	{
		buff[j--]=HexDigit[val%Base];
		val/=Base;
		if(digits!=0) digits--;
	}
	eputs(&buff[j+1]);
	
}

/////////////////////////
//   JDY FUNCTIONS    ///   below
/////////////////////////

void UART1_Init (unsigned long baudrate)
{
    SFRPAGE = 0x20;
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x10;
	SBCON1 =0x00;   // disable baud rate generator
	SBRL1 = 0x10000L-((SYSCLK/baudrate)/(12L*2L));
	TI1 = 1; // indicate ready for TX
	SBCON1 |= 0x40;   // enable baud rate generator
	SFRPAGE = 0x00;
}

void putchar1 (char c) 
{
    SFRPAGE = 0x20;
	while (!TI1);
	TI1=0;
	SBUF1 = c;
	SFRPAGE = 0x00;
}

void sendstr1 (char * s)
{
	while(*s)
	{
		putchar1(*s);
		s++;	
	}
}

char getchar1 (void)
{
	char c;
    SFRPAGE = 0x20;
	while (!RI1);
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

char getchar1_with_timeout (void)
{
	char c;
	unsigned int timeout;
    SFRPAGE = 0x20;
    timeout=0;
	while (!RI1)
	{
		SFRPAGE = 0x00;
		Timer3us(20);
		SFRPAGE = 0x20;
		timeout++;
		if(timeout==25000)
		{
			SFRPAGE = 0x00;
			return ('\n'); // Timeout after half second
		}
	}
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

void getstr1 (char * s, unsigned char n)
{
	char c;
	unsigned char cnt;
	
	cnt=0;
	while(1)
	{
		c=getchar1_with_timeout();
		if(c=='\n')
		{
			*s=0;
			return;
		}
		
		if (cnt<n)
		{
			cnt++;
			*s=c;
			s++;
		}
		else
		{
			*s=0;
			return;
		}
	}
}

// RXU1 returns '1' if there is a byte available in the receive buffer of UART1
bit RXU1 (void)
{
	bit mybit;
    SFRPAGE = 0x20;
	mybit=RI1;
	SFRPAGE = 0x00;
	return mybit;
}

void waitms_or_RI1 (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
	{
		for (k=0; k<4; k++)
		{
			if(RXU1()) return;
			Timer3us(250);
		}
	}
}

void SendATCommand (char * s)
{
	printf("Command: %s", s);
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(5);
	sendstr1(s);
	getstr1(buff, sizeof(buff)-1);
	waitms(10);
	P2_0=1; // 'set' pin to 1 is normal operation mode.
	printf("Response: %s\r\n", buff);
}

static bit dataSent = 0;  // Flag to track if data was already sent

void communicateMaster () 
{
	char c;
	static unsigned int cnt = 0;
	
	if(RXU1()) // Something has arrived - enter the code below
		{
			c=getchar1();
			
			if(c=='!') // Master is sending message
			{
				getstr1(buff, sizeof(buff)-1);
				if(strlen(buff)==19)
				{
					//printf("%s\r\n", buff);
				}
				else
				{
					printf("*** BAD MESSAGE ***: %s\r\n", buff);
				}				
			}
			else if(c=='@') // Master wants slave data
			{
				//sprintf(buff, "%05u\n", f);
				//cnt++;
                //sprintf(flagbuff,"%ld",f);
				//waitms(5); // The radio seems to need this delay...
				//sendstr1(flagbuff);
				//dataSent = 1;  // Set flag after sending
				
			}
		}
	}
	
	
void sendDataToMaster(char* message) 
{
	waitms(5); // The radio seems to need this delay...
	sendstr1(message);
}

void ReceptionOff (void)
{
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	sendstr1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	P2_0=1; // 'set' pin to 1 is normal operation mode.
}


//////////////////////////
//// LCD Functions ///////
//////////////////////////

void LCD_pulse (void)
{
	LCD_E=1;
	Timer3us(40);
	LCD_E=0;
}

void LCD_byte (unsigned char x)
{
	// The accumulator in the C8051Fxxx is bit addressable!
	ACC=x; //Send high nible
	LCD_D7=ACC_7;
	LCD_D6=ACC_6;
	LCD_D5=ACC_5;
	LCD_D4=ACC_4;
	LCD_pulse();
	Timer3us(40);
	ACC=x; //Send low nible
	LCD_D7=ACC_3;
	LCD_D6=ACC_2;
	LCD_D5=ACC_1;
	LCD_D4=ACC_0;
	LCD_pulse();
}

void WriteData (unsigned char x)
{
	LCD_RS=1;
	LCD_byte(x);
	waitms(2);
}

void WriteCommand (unsigned char x)
{
	LCD_RS=0;
	LCD_byte(x);
	waitms(5);
}

void LCD_4BIT (void)
{
	LCD_E=0; // Resting state of LCD's enable is zero
	// LCD_RW=0; // We are only writing to the LCD in this program
	waitms(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	waitms(20); // Wait for clear screen command to finsih.
}

void LCDprint(char * string, unsigned char line, bit clear)
{
	int j;

	WriteCommand(line==2?0xc0:0x80);
	waitms(5);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}

///////////////
void victorydance(void)
{

	OUTPIN1 = 1;  // Activate right motor forward
    OUTPIN3 = 1;  // Activate left motor backward
    OUTPIN2 = 0;  // Ensure right motor backward is off
    OUTPIN4 = 0;  // Ensure left motor forward is off

	LED_RIGHT = 0;
	LED_LEFT = 1;
	
	servo1 = 230;
    waitms(500);
	servo2 = 210;
	waitms(50);
	servo2 = 50;
	
	LED_RIGHT = 1;
	LED_LEFT = 0;
	
	servo1 = 230;
    waitms(50);
	servo2 = 210;
	waitms(50);
	servo2 = 50;
	
	LED_RIGHT = 0;
	LED_LEFT = 1;
	
	servo1 = 230;
    waitms(50);
	servo2 = 210;
	waitms(50);
	servo2 = 50;
	
	LED_RIGHT = 1;
	LED_LEFT = 0;
	
	servo1 = 230;
    waitms(50);
	servo2 = 210;
	waitms(50);
	servo2 = 50;
	
}



//////////////////////////


void pickupcoin(void)
{

    int i;
    int j;
    int k;
//DOWN
    servo1 = 230;
    waitms(500);
    servo2 = 210;
    waitms(500);



    OUTPIN5 = 1;
    waitms(500);
//SEARCH
    for(i = 0; i<120;i++){

    servo1 = (230-i);
    waitms(8);


    }


//Lift

    for(j = 0; j<150;j++){

    servo2 = (210-j);
    waitms(8);


    }

    waitms(500);

    for(k = 0; k<61;k++){

    servo1 = (110-k);
    waitms(8);

    }
    //50
    waitms(500);
    OUTPIN5 = 0;
    waitms(500);
    servo1 = 90;
    //100
    servo2 = 125;

    /*
    // Faster version
    servo1 = 150;
    waitms(500);
    servo2 = 210;
    waitms(500);
    OUTPIN5 = 1;
    waitms(500);
    servo2 = 300;
    waitms(500);
    servo1 = 49;
    //50
    waitms(500);
    OUTPIN5 = 0;
    waitms(500);
    servo1 = 90;
    //100
    servo2 = 125;
    */
}

// For Ultra Sonic Sensor

void Ultrasonic_Trigger()
{
//    TRIG = 0; // Ensure it's low
    Timer3us(2); // Wait 2us
//    TRIG = 1; // High for 10us
    Timer3us(10);
//    TRIG = 0; // Low again
}

unsigned long Measure_Distance()
{
    unsigned long duration;
    
//    while (ECHO == 0); // Wait for echo to go high
    TR0 = 0; // Stop timer
    TL0 = 0; 
    TH0 = 0; 
    TF0 = 0; // Reset timer overflow
    TR0 = 1; // Start timer

//    while (ECHO == 1); // Wait for echo to go low
    TR0 = 0; // Stop timer

    duration = (TH0 << 8) | TL0; // Get the timer value
    return duration * 0.034 / 2; // Convert to cm (Speed of sound = 340m/s)
}

///
// MOVEMENT FUNCTIONS BELOW

// Movement Functions
void goRight() 
{
	LED_RIGHT=0;
	LED_LEFT=1;
    OUTPIN1 = 1;  // Activate right motor forward
    OUTPIN3 = 1;  // Activate left motor backward
    OUTPIN2 = 0;  // Ensure right motor backward is off
    OUTPIN4 = 0;  // Ensure left motor forward is off
}

void goLeft() 
{
	LED_RIGHT=1;
	LED_LEFT=0;
    OUTPIN4 = 1;  // Activate left motor forward
    OUTPIN2 = 1;  // Activate right motor backward
    OUTPIN1 = 0;  // Ensure right motor forward is off
    OUTPIN3 = 0;  // Ensure left motor backward is off
}

void goForward() 
{
	LED_RIGHT=1;
	LED_LEFT=1;
    OUTPIN1 = 1;  // Activate right motor forward (Left wheel)
    OUTPIN4 = 1;  // Activate left motor forward (Right wheel)
    OUTPIN2 = 0;  // Ensure right motor backward is off
    OUTPIN3 = 0;  // Ensure left motor backward is off
}

void goForward_auto()
{

    LED_RIGHT=1;
	LED_LEFT=1;
    OUTPIN1 = 1;  // Activate right motor forward (Left wheel)
    OUTPIN4 = 1;  // Activate left motor forward (Right wheel)
    OUTPIN2 = 0;  // Ensure right motor backward is off
    OUTPIN3 = 0;  // Ensure left motor backward is off
    
    waitms(20);  // Move forward for 'duration_ms' milliseconds

    // Stop motors
    OUTPIN1 = 0;
    OUTPIN4 = 0;
    

}

void goBackward() 
{
	LED_RIGHT=0;
	LED_LEFT=0;
    OUTPIN2 = 1;  // Activate right motor backward
    OUTPIN3 = 1;  // Activate left motor backward
    OUTPIN1 = 0;  // Ensure right motor forward is off
    OUTPIN4 = 0;  // Ensure left motor forward is off
}

void goStop() 
{
    OUTPIN1 = 0;  // Turn off right motor forward
    OUTPIN2 = 0;  // Turn off right motor backward
    OUTPIN3 = 0;  // Turn off left motor backward
    OUTPIN4 = 0;  // Turn off left motor forward
}

void detectPerimeter() 
{
	int r = 250+rand()%(1000-250+1);
	//int r = 250 + (((double)rand() / 250) * (250));
 	// Go backwards
	goBackward();
	waitms(500);  // Adjust delay for proper rotation
	goLeft();
	waitms(r);
	goStop();
}


bit bufferContainsChar(char *buf, char target, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        if (buf[i] == target)
            return 1;  // found it
    }
    return 0;  // not found
}


// below is part of robot base
// WARNING: do not use printf().  It makes the program big and slow!

void main (void)
{    
    long int j1, j2, v1, v2;
	long int count;
	unsigned char LED_toggle=0; // Used to test the outputs
	unsigned int cnt=0;
	//unsigned long distance;
	//char lcdbuff[14];
	
	static int coinPicked =0;
	int coincount;
	coincount = 0;
	
	
	////////////////
	//Get our ref //
	////////////////
	
	//count=GetPeriod(30);	
    //base_freq = (SYSCLK*30.0)/(count*12);
	count=GetPeriod(30);	
    base_freq = (SYSCLK*30.0)/(count*12);
    
	
	//unsigned int automaticMode = 1; // This is a flag for if automatic mode is on
	
	waitms(500);
	printf("\r\nEFM8LB12 JDY-40 Slave Test.\r\n");
	UART1_Init(9600);
	
	ReceptionOff();

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDF4D3\r\n");  
	SendATCommand("AT+RFC111\r\n");
	cnt=0;
	
	//InitPinADC(X, X); // Configure PX.X as analog input
	InitPinADC(0,7); // Configure p0.7 as analog input 
	InitPinADC(0,6); // Configure p0.6 as analog input (this one works)
	InitADC();
	
    
    waitms(1000); // Wait a second to give PuTTy a chance to start
    
	eputs("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
    /*
	eputs("\r\nEFM8LB12 multi I/O example.\r\n");
	eputs("Measures the voltage from pins P2.2 and P2.3\r\n");
	eputs("Measures period on P1.5\r\n");
	eputs("Toggles pins P1.0, P1.1, P1.2, P1.3, P1.4\r\n");
	eputs("Generates servo PWMs on P1.6 and P1.7\r\n");
	eputs("Reads the BOOT push-button on pin P3.7\r\n\r\n");
	*/
		
	OUTPIN1=0;
	OUTPIN2=0;
	OUTPIN3=0;
	OUTPIN4=0;
	OUTPIN5=0;
	
	
	///////////////
	LCD_4BIT();
	LCDprint("     MANUAL     ",1,1);
	LCDprint("      MODE      ",2,1);

	
	
	while (1)
	{
		
	 //////////////////////////////////////////
	// PICK UP COIN FOR FREQUENCY THRESHOLD///
	//////////////////////////////////////////
		 //PrintNumber((long int) automaticMode, 10, 7);
			       
		j1 = ADC_at_Pin(QFP32_MUX_P0_7);
		j2 = ADC_at_Pin(QFP32_MUX_P0_6);
    
	    // Convert ADC values to millivolts
	    v1 = (j1 * 33000) / 0x3FFF;
	    v2 = (j2 * 33000) / 0x3FFF;
	    
		
	    //eputs("ADC[P0.5]=0x");
        //PrintNumber(j2, 16, 4);
        //eputs(", ");
        //PrintNumber(v2,10,1);
        //PrintNumber(v2/10000, 10, 1);
        //putchar('.');
        //PrintNumber(v2%10000, 10, 4);
        //eputs("V ");
        //eputs("          \r");
              
		
		//Ultrasonic_Trigger();
        //distance = Measure_Distance();
        //printf("Distance: %lu cm\n", distance);
        
	    count=GetPeriod(30);	
		
		if(count>0)
		{
			f=(SYSCLK*30.0)/(count*12);
			//eputs("f=");
			//PrintNumber(f, 10, 7);
			//eputs("Hz, count=");
			//PrintNumber(count, 10, 8);
			//eputs("          \r");
			diff = f-base_freq;
		}
		else
		{
			eputs("NO SIGNAL                     \r");
		}
		
		//sprintf(buff,"%lu", f);	
		//waitms(5); // The radio seems to need this delay...
		//sendstr1(buff);
		//dataSent = 1;  // Set flag after sending
		
		
		/////////////////////
		// AUTOMATIC       //
		/////////////////////
		
		
		                 //123456789012345678			
		//if ((strcmp(buff, "444444444444444444@") == 0) || (strcmp(buff, "@444444444444444444") == 0) || (strcmp(buff, "444444444444444444!") == 0) || (strcmp(buff, "!444444444444444444") == 0) || (buff[9] == '4') || (strcmp(buff, "44444@")==0))
		if(bufferContainsChar(buff, '4', 20))
		{
		    if(!toggleHandled)
		    {
		  	automaticMode = !automaticMode;
		  	goStop();
		  	//eputs("  dtt  ");
		  	waitms(500);
		  	toggleHandled = 1;
		  	}
		}		
		
		else
		{
			toggleHandled = 0;
		}
		
		communicateMaster();
		
////////////////////////////////////////
// EITHER AUTOMATIC OR MANUAL BELOW    /
//$/$$$/$/$/$$/$/$///$//////////////////
		
		if (automaticMode == 1) 
		{
		
		eputs("f=");
	    PrintNumber(f, 10, 7);
	    eputs("Hz");
	    eputs("  f_ref=");
	    PrintNumber(base_freq, 10, 7);
	    eputs("  diff=");
	    PrintNumber(diff, 10, 7);
	    eputs("          \r");
		
		LCDprint(" AUTOMATIC MODE ",1,1);
		sprintf(lcdbuff,"COINS:  %d  ",coincount);
		LCDprint(lcdbuff,2,1);
			
			//waitms(500);     // wait for "initialization"
			goForward();
			
			if(diff >200)
			{
					goBackward();
					waitms(350);
			    	goStop();
					pickupcoin();
					coincount++;
										
			}
	
				if (coincount == 20)
			{
					goStop();
					automaticMode = 0;
					coincount = 0;
			}
								
		
			if ( v2 > 18000 || v1 > 18000 ) 
			{
		       detectPerimeter();
		    }
		   
		}
		
		// When automatic mode is off
		else {
			//////////////////
			// SPIN RIGHT   //
			//////////////////
			
	
			if ((strcmp(buff, "100010001000100011@") == 0) || (strcmp(buff, "@100010001000100011") == 0) || (strcmp(buff, "100010001000100011!") == 0) || (strcmp(buff, "!100010001000100011") == 0))
			{
			    goRight();
			}
			//////////////////
			// SPIN LEFT   //
			//////////////////
			else if ((strcmp(buff, "213021001100213001@") == 0) || (strcmp(buff, "@213021001100213001") == 0) || (strcmp(buff, "213021001100213001!") == 0) || (strcmp(buff, "!213021001100213001") == 0))
			{
			    goLeft();
			}
			//////////////////
			// FORWARD     //
			//////////////////
			else if ((strcmp(buff, "989898989898989898@") == 0) || (strcmp(buff, "@989898989898989898") == 0) || (strcmp(buff, "989898989898989898!") == 0) || (strcmp(buff, "!989898989898989898") == 0))
			{
			    goForward();
			}
			
			/////////////////
			// BACKWARDS   //
			/////////////////
			
			else if ((strcmp(buff, "767676767676767676@") == 0) || (strcmp(buff, "@767676767676767676") == 0) || (strcmp(buff, "767676767676767676!") == 0) || (strcmp(buff, "!767676767676767676") == 0))
			{
			  	goBackward();
			}
			                      //123456789012345678
			else if ((strcmp(buff, "555555555555555555@") == 0) || (strcmp(buff, "@555555555555555555") == 0) || (strcmp(buff, "555555555555555555!") == 0) || (strcmp(buff, "!555555555555555555") == 0))
			{
			  if(!coinPicked)
			  {
			  pickupcoin();
			  coinPicked = 1;
			  //waitms(1000);
			  }
			  
			  
			  
			}
			
			
			//////////////////
			// STOP        //
			//////////////////
			else if ((strcmp(buff, "No movement No mov@") == 0) || (strcmp(buff, "@No movement No mov") == 0) || (strcmp(buff, "No movement No mov!") == 0) || (strcmp(buff, "!No movement No mov")) )
			{
			    coinPicked=0;
			    goStop();
			}
			
			else
			{
			
			    coinPicked=0;
			   
			    OUTPIN1 = 0;
			    OUTPIN2 = 0;
			    OUTPIN3 = 0;
			    OUTPIN4 = 0;
			    
			}
			
	
			
			
		} // this signifies the end of the manual part
		
		//j=ADC_at_Pin(QFP32_MUX_P2_2);
		//v=(j*33000)/0x3fff;
		//eputs("ADC[P2.2]=0x");
		//PrintNumber(j, 16, 4);
		//eputs(", ");
		//PrintNumber(v/10000, 10, 1);
		//putchar('.');
		//PrintNumber(v%10000, 10, 4);
		//eputs("V ");

		//j=ADC_at_Pin(QFP32_MUX_P2_3);
		//v=(j*33000)/0x3fff;
		//eputs("ADC[P2.3]=0x");
		//PrintNumber(j, 16, 4);
		//eputs(", ");
		//PrintNumber(v/10000, 10, 1);
		//putchar('.');
		//PrintNumber(v%10000, 10, 4);
		//eputs("V ");
		
		//eputs("BOOT(P3.7)=");
		//if(BOOT)
		//{
		//	eputs("1 ");
		//}
		//else
		//{
		//	eputs("0 ");
		//}

		// Not very good for high frequencies because of all the interrupts in the background
		// but decent for low frequencies around 10kHz.
		
	
		
   
	    
	}
}