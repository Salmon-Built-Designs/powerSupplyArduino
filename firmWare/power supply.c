/*****************************************************
This program was produced by the
CodeWizardAVR V1.25.9 Standard
Automatic Program Generator
© Copyright 1998-2008 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 29.10.2010
Author  : F4CG                            
Company : F4CG                            
Comments: 


Chip type           : ATmega8
Program type        : Application
Clock frequency     : 8,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 256
*****************************************************/

#include <mega8.h>
 #include <lcd.h> 
#include <delay.h>  

// Alphanumeric LCD Module functions
#asm
   .equ __lcd_port=0x12 ;PORTD
#endasm
#include <lcd.h>

eeprom unsigned int set_u=0;
eeprom unsigned int set_i=0;
int pwm_val_a=0,pwm_val_b=0,time_out=0,i=0;   
unsigned char set_mode=0,EncState=0; 
unsigned char Dig[6];
unsigned long int I_ust=0,I_izm=0,U=0; 
char Counter=0,count=5,k=1; 

//Функция  антидребезга
  unsigned char incod(void)
  {
   unsigned char cod0=0;
   unsigned char k,cod1;

   for(k=0;k<30;k++)
      {
       cod1=PINB&0xC0;
        if (cod0!=cod1)
         {
           k=0;
           cod0=cod1;
         }
       }
       return cod1;
   };
   
    void PrepareData( unsigned long int adc_result)      // Ф-ция для разложения числа 
{ 
unsigned char Num1=0,Num2=0, Num3=0,Num4=0;


 while (adc_result >= 1000) //тысяча
  {
  adc_result -= 1000;  
  Num1++; 
  }
  
  while (adc_result >= 100) //сотня
  {
  adc_result -= 100;  
  Num2++;
  }
   while (adc_result >= 10) //десятки
  {
  adc_result -= 10;  
  Num3++;
  }
  
  Num4 = adc_result; //остаток
  Dig[0] = Num1+0x30;
  Dig[1] = Num2+0x30;
  Dig[3] = Num3+0x30;
  Dig[4] = Num4+0x30; 
}  

void Display(void)   // Ф-ция  отображения
{
  int i;
  for(i=0; i<6; i++)
  {
    lcd_putchar(Dig[i]);
  }
} 



#define ADC_VREF_TYPE 0xC0

// Read the AD conversion result
unsigned long int read_adc(unsigned char adc_input)
{
ADMUX=adc_input | (ADC_VREF_TYPE & 0xff);
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=0x40;
// Wait for the AD conversion to complete
while ((ADCSRA & 0x10)==0);
ADCSRA|=0x10;
return ADCW;
}


// Timer 0 overflow interrupt service routine  отображение информации
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
  #asm("wdr")
  TCNT0=0x7F;
  time_out+=1;   
   
if (Counter++ >=count)
  {  
 // lcd_init(16);
   //#asm("wdr")  
    Counter = 0;
    
    lcd_gotoxy(1,0); 
    //напряжение
    PrepareData(U/count);   //       
   
     Dig[2] = '.';    
     Dig[5] = 'V';
     Display();       
    lcd_putsf(", ");
    
    //ток измеренный 
    
    PrepareData(I_izm/count);   //  
     Dig[2] = '.';     
     Dig[5] = 'A';
    Display();    
    
    lcd_gotoxy(1,1);
    
    //ток установленный
    lcd_putsf("Set I=  ");
    
    PrepareData(I_ust/count);   //  
     Dig[2] = '.';
     Dig[5] = 'A';  
    Display();   
    
    if (I_izm>=I_ust)
    { PORTB.0=1; 
    lcd_gotoxy(0xF,1);
      lcd_putchar(0x23);
    } 
    else        
    {
      lcd_putchar(' ');
      PORTB.0=0;
    }  
    
    if (set_mode==1) 
       { lcd_gotoxy(0,0);
        lcd_putchar(0x3E);
         lcd_gotoxy(0,1);
          lcd_putchar(0x20); 
           };     
           
           if (set_mode==2) 
       { lcd_gotoxy(0,1);
        lcd_putchar(0x3E);
         lcd_gotoxy(0,0);
          lcd_putchar(0x20); 
           }; 
       
             if (set_mode==0) 
       { lcd_gotoxy(0,1);
        lcd_putchar(0x20);
         lcd_gotoxy(0,0);
          lcd_putchar(0x20); 
           }; 
    
    U = 0;
    I_izm = 0;
    I_ust = 0;
  }
  else
  {
   U += read_adc(2) * 2;
    I_izm += read_adc(0);
    I_ust += read_adc(1);
  }  
        
      
       


// Place your code here

}

void EncoderScan(void) // Функция опроса энкодера
{
unsigned char New=0;
 
New = incod();	// Берем текущее значение 
			// И сравниваем со старым
 
// Смотря в какую сторону оно поменялось -- увеличиваем
// Или уменьшаем соответствующий параметр
 
  if (set_mode==1)
   {
  
switch(EncState)
	{
	case 128:
		{
		if(New == 193) pwm_val_b++;
		if(New == 0) pwm_val_b--; 
		break;
		}
 
	case 0:
		{
		if(New == 128) pwm_val_b++;
		if(New == 64) pwm_val_b--; 
		break;
		}
	case 64:
		{
		if(New == 0) pwm_val_b++;
		if(New == 193) pwm_val_b--; 
		break;
		}
	case 193:
		{
		if(New == 64) pwm_val_b++;
		if(New == 128) pwm_val_b--; 
		break;
		}
	}; 
	if (pwm_val_b>=1023) {pwm_val_b=1023;  }
else  {
       if (pwm_val_b<=0) {pwm_val_b=0;};
    
    };
	if (New!=EncState) 
	{   OCR1B=pwm_val_b;
	time_out=0; 
	   };

    }
else 
 {
   if (set_mode==2)
   { 
    
switch(EncState)
	{
	case 128:
		{
		if(New == 193) pwm_val_a++;
		if(New == 0) pwm_val_a--; 
		break;
		}
 
	case 0:
		{
		if(New == 128) pwm_val_a++;
		if(New == 64) pwm_val_a--; 
		break;
		}
	case 64:
		{
		if(New == 0) pwm_val_a++;
		if(New == 193) pwm_val_a--; 
		break;
		}
	case 193:
		{
		if(New == 64) pwm_val_a++;
		if(New == 128) pwm_val_a--; 
		break;
		}
	}  
	if (pwm_val_a>=511)
	 {
	     pwm_val_a=511;
	   }
else  {
       if (pwm_val_a<=0) {pwm_val_a=0;};
    
    };

	
	if (New!=EncState) {   OCR1A=pwm_val_a;
	                       time_out=0;   };

	};
	};
 
EncState = New;		// Записываем новое значение 
				// Предыдущего состояния

 };
// Declare your global variables here

void main(void)
{
// Declare your local variables here
OSCCAL=0x9F;
// Input/Output Ports initialization
// Port B initialization
// Func7=In Func6=In Func5=Out Func4=In Func3=In Func2=Out Func1=Out Func0=Out 
// State7=P State6=P State5=0 State4=P State3=P State2=0 State1=0 State0=0 
PORTB=0xD8;
DDRB=0x27;

// Port C initialization
// Func6=In Func5=In Func4=Out Func3=In Func2=In Func1=In Func0=In 
// State6=T State5=T State4=0 State3=T State2=T State1=T State0=T 
PORTC=0x00;
DDRC=0x10;

// Port D initialization
// Func7=Out Func6=Out Func5=Out Func4=Out Func3=Out Func2=Out Func1=Out Func0=Out 
// State7=0 State6=0 State5=0 State4=0 State3=0 State2=0 State1=0 State0=0 
PORTD=0x00;
DDRD=0xFF;

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 7,813 kHz
TCCR0=0x05;
TCNT0=0x7F;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 8000,000 kHz
// Mode: Ph. correct PWM top=03FFh
// OC1A output: Non-Inv.
// OC1B output: Non-Inv.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer 1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=0xA3;
TCCR1B=0x01;
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer 2 Stopped
// Mode: Normal top=FFh
// OC2 output: Disconnected
ASSR=0x00;
TCCR2=0x00;
TCNT2=0x00;
OCR2=0x00;

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
MCUCR=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=0x01;

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
SFIOR=0x00;

// ADC initialization
// ADC Clock frequency: 1000,000 kHz
// ADC Voltage Reference: Int., cap. on AREF
ADMUX=ADC_VREF_TYPE & 0xff;
ADCSRA=0x83;

// LCD module initialization
lcd_init(16);

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/2048k
#pragma optsize-
WDTCR=0x1F;
WDTCR=0x0F;
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif
 pwm_val_a=set_i;
  pwm_val_b=set_u;
  OCR1A=pwm_val_a;
  OCR1B=pwm_val_b;
  lcd_clear();
     i=0;
 lcd_gotoxy(5,0);     
 lcd_putsf("idea");
 lcd_gotoxy(0,1);
 lcd_putsf("hardlock.org.ua");
  while (i<200) {i+=1;
  delay_ms(5);};
  lcd_clear();
     i=0;
 lcd_gotoxy(0,0);     
 lcd_putsf("scheme, firmware");
 lcd_gotoxy(4,1);
 lcd_putsf(" SONATA");
  while (i<200) {i+=1;
  delay_ms(5);};
 i=0;
 lcd_clear();
 lcd_gotoxy(2,0);
lcd_putsf("power supply");
 lcd_gotoxy(4,1);
 lcd_putsf(" ver 1.0");
  while (i<200) {i+=1;
  delay_ms(5);};
  i=0;  
  
  
// Global enable interrupts
#asm("sei")
PORTC.4=1;

while (1)
      {
       if (PINB.4==0) 
       {k=0;
       time_out=0;
        set_mode+=1;
         if (set_mode>2) {set_mode=0;};  
         while (PINB.4==0) {    };       
};

     if (k<1)
     {
        if (time_out>200) 
        { set_mode=0;
          k=1;
        set_i=pwm_val_a;
         set_u=pwm_val_b;
                    };
        }; 
        
        EncoderScan();
      // Place your code here
        
      
      
      
      };
}
