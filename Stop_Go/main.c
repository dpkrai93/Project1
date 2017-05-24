// Serial Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE4 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE5 drives an NPN transistor that powers the blue LED
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//#include <string.h>
#include <strings.h>
#include "tm4c123gh6pm.h"
#include <math.h>


#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))

#define cntrlWord 0x3000;
#define dcOff 2006;
#define gain 1917;
#define tableSize 4096;

typedef enum state{output_DC,output_Sine,output_Sweep,output_voltage} State;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA |SYSCTL_RCGC2_GPIOB| SYSCTL_RCGC2_GPIOF| SYSCTL_RCGC2_GPIOE;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button


       // Configure AN1 as an analog input
         SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
        	GPIO_PORTE_AFSEL_R |= 0x0C;                      // select alternative functions for AN0 (PE3)
            GPIO_PORTE_DEN_R &= ~0x0C;                       // turn off digital operation on pin PE3
            GPIO_PORTE_AMSEL_R |= 0x0C;                      // turn on analog operation on pin PE3
            ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
            ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
            ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
            ADC0_SSMUX3_R = 1;                               // set first sample to AN0
            ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
            ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation


    // Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


    // Configure A0 and ~CS for SSI2
       GPIO_PORTB_DIR_R |= 0x42;  // make bits 1 and 6 outputs
       GPIO_PORTB_DR2R_R |= 0x42; // set drive strength to 2mA
       GPIO_PORTB_DEN_R |= 0x42;  // enable bits 1 and 6 for digital

    // Configure SSI2 pins for SPI configuration
      SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
      GPIO_PORTB_DIR_R |= 0xB0;                        // make bits 4 and 7 outputs
      GPIO_PORTB_DR2R_R |= 0xB0;                       // set drive strength to 2mA
      GPIO_PORTB_AFSEL_R |= 0xB0;                      // select alternative functions for MOSI, SCLK pins
      GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK| GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
      GPIO_PORTB_DEN_R |= 0xB0;                        // enable digital operation on TX, CLK pins
      GPIO_PORTB_PUR_R |= 0x10;                        // must be enabled when SPO=1

      // Configure the SSI2 as a SPI master, mode 3, 16bit operation, 1 MHz bit rate
      SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
      SSI2_CR1_R = 0;                                  // select master mode
      SSI2_CC_R = 0;                                   // select system clock as the clock source
      SSI2_CPSR_R = 20;                                // set bit rate to 2 MHz (if SR=0 in CR0)
      SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
      SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2

      // Configure Timer 1 as the time base
         SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
         TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
         TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
         TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
         TIMER1_TAILR_R = 0x190;   		                  // set load value to 40e6 for 1 Hz interrupt rate
         TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
         NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
        // TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer


}

int16_t readAdc()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	uint8_t i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}

char stringg[81]={0};
char str1[81]={0};
char str2[81]={0};
char str3[10];
char field_type[3];
char index1[81];
char index2[81];
int field_count=0;
int field_index[3]={0};
int freq,freq3=0;
uint32_t phase=0,delta_phase;
uint32_t in,val;
uint32_t index3,value;
uint32_t table[4096],l,k,i,s,y;
uint16_t volt1;
//uint16_t sweep_table[]={};

char mode;
float pi=3.1415,amp=0,amp1=0 ;
float volt,instantVolt,iirVolt;
float alpha = 0.99;
int firstUpdate = true;
float freq_temp=0,raw,instant_adc=0,freq1=0,freq2=0;
char vtg_temp[10],fre_temp[10];
uint8_t count=2;


char step2(char *stringg)
	{
			int count=0;
			char c;

			for(i=0;i<81;i++)
			{
				stringg[i]=NULL;
				str1[i]=NULL;
				str2[i]=NULL;
			}
			while(1)

			{

				 c = getcUart0();

					if(c==0x08)
					{
							if (count==0)
							{
								continue;
							}
							else
							{
								count--;
								continue;
							}
					}
					if(c==0x0D)
					{
						stringg[count]='\0';
						break;
					}

					else if(c >= 0x20)
					{

						stringg[count]=c;
						count++;

							if (count==80)
							{
								stringg[count++]='\0';
								break;
							}
							else
							{
								continue;
							}

					}

				}
			putsUart0(stringg);
			return 0;

	}



	void step3(char stringg[])
	{

		int a=0,b=0;
		field_count=0;
		char str2[81]={0};


		char index1[81]={0};
		char index2[81]={20};





		for(i=0; i<strlen(stringg); i++)
			{
				if (stringg[i]>=0x41 && stringg[i]<=0x5A || stringg[i]>=0x61 && stringg[i]<=0x7A)
				{
					str1[i]=stringg[i];
					str2[i]='c';
				}

				else if ((stringg[i]>=0x30 && stringg[i]<=0x39)|| stringg[i]==0x2E ||stringg[i]==0x2B ||stringg[i]==0x2D )
				{
					str1[i]=stringg[i];
					str2[i]='n';
				}

				else
					{
					str1[i]='\0';
					str2[i]='\0';
					}
			}

		for(i=0; i<80; i++)
			{


					index1[b]=str2[i];
					index2[b]=str2[i+1];

					if(index1[b]!=index2[b] && index2[b]!='\0')
					{
							field_index[a]=i+1;
							field_type[a]=index2[b];
							(field_count)++;
							a++;

					}

					b++;
			}

		putsUart0(str1);
		putsUart0(str2);


	}

	void waitMicrosecond(uint32_t us)
	{
		__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
	    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
	    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
	    __asm("             NOP");                  // 5
	    __asm("             NOP");                  // 5
	    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
	    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
	    __asm("             CBZ  R0, WMS_DONE0");   // 1
		__asm("             NOP");                  // 1
	    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
	    __asm("WMS_DONE0:");                        // ---
	                                                // 40 clocks/us + error
	}





float getNumber(int field_number)
			{
			 	float number=0;
				number = atof(&stringg[field_index[field_number]]);
				return number;
			}




_Bool iscommand(char *match_string, int min_arg)
		{

			if ((strcasecmp(&str1[field_index[0]], match_string)==0) && (field_count > min_arg))
				return 1;
			else
				return 0;
		}

void setDC()
{
 	if(volt<(-5)||volt>5)
	{
		putsUart0("\r\nEnter the correct string\r\n");
	}
 	if((field_type[1]!='n')||(field_type[2]!='n'))
 	{
 		putsUart0("\r\nEnter the correct string\r\n");
 	}
	 if(volt>=0x30 && volt<=0x35 && volt==0x2D)
		{
		 	y=((dcOff)+((2089*volt)/5));
		 	volt=(cntrlWord|y);
			SSI2_DR_R =volt;

		}

		else //if(volt>=0x30 && volt<=0x35)
		{
			y=dcOff-((gain*volt)/5);
			volt= 0x3000|y;
			SSI2_DR_R =volt;

		}

}



void LUT()
{
	for(i=0;i<4096;i++)
	{
		s=(sin(2*pi*i)/tableSize)/5);
		y=dcOff+(1917*amp*x);
		table[i]= 0x3000|y; 														//0x3000+2048+(2047*sin((2*pi*i)/4096));

	}

}

void LUT2()
{
	for(i=0;i<4096;i++)
	{
		s=(((2*amp)*i)/((tableSize)*5));
		y=(dcOff)-(gain*s);
		table[i]=0x3000|y;
	}
	for(;i<4096;i++)
	{
		s=((2*amp)*i/(tableSize*5));
		y=(dcOff)-(gain*s);
		table[i]=0x3000|y;
	}
}


void LUT3()
{
/*	float temp=0;
	for(i=0;i<4096;i++)
	{
		temp=0;
		for(k=0;k<25;k++)
		{
			temp=temp-sin(pi*(2*k+1)*i/2006)/(2*k+1);
		}
		//table[i]=temp*(amp)*1907/(5)+2006+0x3000;
	}*/

	for(i=0;i<2048;i++)
	{
		 y=dcOff+((2089*volt)/5);
		table[i]= cntrlWord|y;

	}
	for(;i<4096;i++)
	{
		 y=dcOff-((2089*volt)/5);
		table[i]= cntrlWord | y;

	}

}

void sweep(void)
{
 freq_temp=freq1;
 amp=(amp1);
 LUT();
 count=1;
 putsUart0("\r\nFreq      Amp\r\n");
 while(freq_temp<freq2)
 {
	 for(i=0;i<10;i++)
	 {
		 vtg_temp[i]='\0';
		 fre_temp[i]='\0';
	 }
	 delta_phase=freq_temp*4294967296/100000;
	 TIMER1_CTL_R |= TIMER_CTL_TAEN;
	 if(freq_temp<1000)
	 {
		 ADC0_SSMUX3_R = 1;
		 waitMicrosecond(7000000);
		 raw = readAdc();
		 instant_adc= ((10/2)*(raw+0.5)/4096);
	 }
	 else
	 {
		 ADC0_SSMUX3_R = 0;
		 waitMicrosecond(300000);
		 raw = readAdc();
		 instant_adc= ((9.8/2)*(raw+0.5)/4096);

	 }
//	 raw = readAdc();
//	 instant_adc= (8.5*(raw+0.5)/4096);

	 sprintf(fre_temp,"%0.1f",freq_temp);
	 if(count>1)
	 {
	 putsUart0("\r\n");
	 putsUart0(fre_temp);
	 putsUart0("      ");
	 sprintf(vtg_temp,"%0.2f",instant_adc);
	 putsUart0(vtg_temp);
	 }
	 freq_temp=(freq1+(log10(count)*(freq2-freq1)));
	 count++;
 }
}

void step4()
{


	if(iscommand("reset",0))
	{
		__asm("    .global _c_int00\n"
			  "    b.w     _c_int00");				//ResetIsr
	}

	else	if(iscommand("voltage",0))
	{
		if(field_count>1)
		{
			putsUart0("\r\nEnter the correct string\r\n");

		}
		else
			{
			mode=output_voltage;
			}

	}


	else	if(iscommand("sine",2))
	{
		freq=getNumber(1);
		if((freq<=0)||(field_type[1]!='n'))
		{
			putsUart0("\r\nEnter the correct string\r\n");
		}

		amp=getNumber(2);
		amp1=getNumber(2);
		if((amp<(-5)||amp>5)||(field_type[2]!='n'))
			{
				putsUart0("\r\nEnter the correct string\r\n");
			}

		else
			{
			mode=output_Sine;
			}
	}

	else	if(iscommand("square",2))
		{
			freq=getNumber(1);
			amp=getNumber(2);
			amp1=getNumber(2);
			LUT3();
			delta_phase= (freq*4294967296)/100000;
			TIMER1_CTL_R |= TIMER_CTL_TAEN;
		}

	else	if(iscommand("sawtooth",2))
		{
			freq=getNumber(1);
			amp=getNumber(2);
			amp1=getNumber(2);
			LUT2();
			delta_phase= (freq*4294967296)/100000;
			TIMER1_CTL_R |= TIMER_CTL_TAEN;
		}

	else	if(iscommand("sweep",2))
		{
			freq1=getNumber(1);
			freq2=getNumber(2);

			if((freq1<=0)||(freq2<=0)||(field_type[1]!='n')||(field_type[2]!='n')||(freq1==freq2))
			{
				putsUart0("\r\nEnter the correct string\r\n");

			}

			else
				{
				mode= output_Sweep;
				}

		}

	else if (iscommand("dc",1))
		{
			volt=getNumber(1);
			mode= output_DC;
		}
	else
	{
			putsUart0("\r\nEnter the correct string\r\n");
	}

	switch(mode)
	{
		default:
		{
		putsUart0("\r\nEnter the correct string\r\n");
		break;
		}
		case output_DC:
		{
			setDC();
			break;
		}

		case output_Sine:
		{
			LUT();
			delta_phase= (freq*4294967296)/100000;
			TIMER1_CTL_R |= TIMER_CTL_TAEN;
			break;
		}
		case output_Sweep:
		{
			sweep();
			break;
		}

		case output_voltage:
		{
			waitMicrosecond(500000);
			volt1=readAdc();
			instantVolt=(10.45/2)*(volt1+0.5)/4096;

			iirVolt= instantVolt;

			sprintf(str3, "%3.1f", iirVolt);
			putsUart0(str3);
			iirVolt=0;
			break;
		}



	}

	putsUart0("\r\nEnter the character\r\n");

}



void Timer1Isr()
{
	phase += delta_phase;
	index3= (phase>>20);
	value= table[index3];
	SSI2_DR_R = value;
	TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	initHw();

	// Display greeting
    putsUart0("\r\nEnter the character\r\n");
    GREEN_LED ^= 1;
        waitMicrosecond(500000);

    while(1)
    {

    step2(stringg);
    step3(stringg);
    step4();

    }
}
