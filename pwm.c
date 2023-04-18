#include<htc.h>
#define _XTAL_FREQ 20000000
#define TMR2PRESCALE 4
__CONFIG(FOSC_HS & WDTE_OFF);
long PWM_freq = 5000;

PWM_Initialize()
{
  PR2 = (_XTAL_FREQ/(PWM_freq*4*TMR2PRESCALE)) - 1; //Setting the PR2 using formula 
    CCP1M3 = 1; CCP1M2 = 1;  //Configure the CCP1 module as PWM
    T2CKPS0 = 1;T2CKPS1 = 0;  //Configure the Timer 2 prescaler as 4
    TMR2ON = 1; 	// Turn ON Timer 2
    TRISC2 = 0; // make pin RC2/CCP1 as output 
}

PWM_Duty(unsigned int duty)
{
    duty = ((float)duty/1023)*PR2*4; 
    CCP1X = duty & 2; //Store the 1st bit
    CCP1Y = duty & 1; //Store the 0th bit
    CCPR1L = duty>>2;// Store the remining 8 bit
}

void ADC_Initialize()
{
 ADCON0 = 0b01000001; //ADC ON and Fosc/16 is selected
 ADCON1 = 0b11000000; // Internal reference voltage is selected
}
unsigned int ADC_Read(unsigned char channel)
{
  ADCON0 &= 0x11000101; //Clearing the Channel Selection Bits
  ADCON0 |= channel<<3; //Setting the required Bits
  __delay_ms(2); //Acquisition time to charge hold capacitor
  GO_nDONE = 1; //Initializes A/D Conversion
  while(GO_nDONE); //Wait for A/D Conversion to complete
  return ((ADRESH<<8)+ADRESL); //Returns Result
}

void main()
{
    int adc_value;
  TRISC = 0x00; //PORTC as output
  TRISA = 0xFF; //PORTA as input
  //TRISD = 0x00;
  ADC_Initialize(); //Initializes ADC Module
  PWM_Initialize();  //This sets the PWM frequency of PWM1

  do
  {
    adc_value = ADC_Read(0); //Reading Analog Channel 0 
    PWM_Duty(adc_value);
      __delay_ms(50); 
      
  }while(1); //Infinite Loop
  
}
