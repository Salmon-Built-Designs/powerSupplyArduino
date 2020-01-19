/*
* powerSupplyArduino.c
*
* Created: 26.12.2019 19:34:45
* Author : Nataly
*/

#define F_CPU 8000000L

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"
#include "version.h"
#include <stdlib.h>
#include "avr/interrupt.h"
#include "string.h"
#include "avr/eeprom.h"

enum modes {NORMAL_MODE, VOLTAGE_SET_MODE, CURRENT_SET_MODE} mode; // 0 - work mode (display cur volumes), 1 - set voltage, 2 - set current
enum errors {NO_ERROR, NO_VOLTAGE, BAD_VOLTAGE, UNKNOWN} ErrCode = UNKNOWN;    
enum AdcInputs {VOLTAGE_ADC = 0b0010, CURRENT_ADC = 0b0000, TERM_ADC = 0b0011} AdcInput = VOLTAGE_ADC;    

#define VAL2PWM10(val, maxval, maxpwm) (uint32_t)val* maxpwm/maxval
#define CUR_LIMIT_ON PORTB |= 0x01
#define CUR_LIMIT_OFF PORTB&=0xFE
#define CUR_LIMIT_INV PORTB^=0x01
#define CUR_LIMIT_DEV 10
#define BLINK_POST_SCAL 25
#define ENCODER_STEP 10

#define  VOLTAGE_MAX 2600
#define  VOLTAGE_MAX_PWM 1023 //up to 5v (pwm 100%)
#define  CUR_MAX 1700 
#define  CUR_MAX_PWM 511 // up to 2.5v because use one value for ADC ( has limit on 2.56v ) and for lm464

// shunt resist 0.04 ohm 



uint8_t blink = 1;

uint8_t blinkPostScal =  BLINK_POST_SCAL;

uint16_t  volatile settedVoltage = 500;
uint16_t  settedVoltageEeprom EEMEM = 0;
uint16_t  volatile settedCurrent = 200;
uint16_t  settedCurrentEeprom EEMEM = 0 ;
uint16_t  volatile CurrentCurrent = 0;
uint16_t  volatile CurrentVoltage = 0;
uint16_t volatile AdcAvgBuf;
uint8_t volatile AdcCount = 0;
uint8_t volatile AdcCountAvg = 0;
uint16_t volatile AdcvBuf;

uint8_t volatile  LcdNeed2refresh = 0;

//uint8_t ErrCode = 0;
uint8_t lastEncoder;

char lcdStr0[16+1];
char lcdStr1[16+1];
char bufStr[8+1];


void readEncoder(void){
    //mode = PORTB;
    uint8_t curEncoder = PINB & (7<<5);
    if (lastEncoder  == 0 || curEncoder == lastEncoder || ErrCode) {lastEncoder = curEncoder; return;}
    // check press button
    if (curEncoder & (1 << PINB7) && !(lastEncoder & (1 << PINB7) )) {
        mode = (mode+1) % 3;
        switch (mode) { 
            case NORMAL_MODE: eeprom_write_word(&settedCurrentEeprom, settedCurrent);
            case CURRENT_SET_MODE: eeprom_write_word(&settedVoltageEeprom, settedVoltage);
            default: break;
            }
    }
    
    // to left
    if ((curEncoder==128 && lastEncoder == 192 )|| (curEncoder == 160 && lastEncoder == 128)) {
        switch (mode) {
            case VOLTAGE_SET_MODE:
            if (settedVoltage>=ENCODER_STEP) settedVoltage-=10;  break;
            case CURRENT_SET_MODE:
            if (settedCurrent>=ENCODER_STEP) settedCurrent-=10; break;
            default: break;
        }

    }
    // to right
    if ((curEncoder == 128 && lastEncoder == 160) || (curEncoder == 192 && lastEncoder == 128)) {
        switch (mode) {
            case VOLTAGE_SET_MODE:
            if (settedVoltage < VOLTAGE_MAX - ENCODER_STEP) settedVoltage+=10; break;
            case CURRENT_SET_MODE:
            if (settedCurrent < CUR_MAX - ENCODER_STEP)  settedCurrent+=10; break;
            default: break;
        }
    }
    // set value to PWM
    OCR1B = VAL2PWM10(settedVoltage, VOLTAGE_MAX, VOLTAGE_MAX_PWM);
    OCR1A = VAL2PWM10(settedCurrent , CUR_MAX, CUR_MAX_PWM);
    
    // refresh Current Led
    if (ErrCode == NO_ERROR){
    if (ErrCode == NO_ERROR && abs(CurrentCurrent - settedCurrent) <= CUR_LIMIT_DEV) CUR_LIMIT_ON; else CUR_LIMIT_OFF;}
    
    // set that display should be refreshed
    LcdNeed2refresh = 1;
    

    lastEncoder = curEncoder;
}

//interrupts
ISR(TIMER0_OVF_vect){
    blinkPostScal--;
    readEncoder();
    if (!blinkPostScal){
        if (ErrCode != NO_ERROR && ErrCode != UNKNOWN)  CUR_LIMIT_INV;
        blink=~blink;
        blinkPostScal = BLINK_POST_SCAL;        
        ++LcdNeed2refresh;
    }
}

ISR(ADC_vect)
{            
    if (AdcCountAvg == 0) {AdcAvgBuf = ADCW;   } else {AdcAvgBuf+=ADCW;}  
    if (AdcInput == VOLTAGE_ADC)   AdcvBuf  =      ADCW; 
    if (AdcCountAvg == 3) {
        
        switch(AdcInput){
            case VOLTAGE_ADC:
                        CurrentVoltage = (uint32_t)AdcAvgBuf * (uint32_t)VOLTAGE_MAX /((uint16_t)1020 << 2) ; break; 
            case CURRENT_ADC: 
                        CurrentCurrent =  (uint32_t)AdcAvgBuf * (uint32_t)CUR_MAX / ((uint16_t)1020 << 2) ; break;  
            case TERM_ADC:     break; // need todo   
            
        }
        // refresh Current Led
        if (ErrCode == NO_ERROR){
        if ( abs(CurrentCurrent - settedCurrent) <= CUR_LIMIT_DEV) CUR_LIMIT_ON; else CUR_LIMIT_OFF;}
        //reset avg count
      AdcCountAvg = 0;  
      // increment conversation
      ++AdcCount;  
      // next conversion preparation
      if (AdcCount == 0xff) { AdcInput = TERM_ADC; } else { AdcInput =  AdcCount % 2 == 0 ? VOLTAGE_ADC: CURRENT_ADC;   }
      // change input pin
      ADMUX = (ADMUX & 0xF0) | AdcInput;
    }  else {++AdcCountAvg;}  
    
    
   
   // Delay needed for the stabilization of the ADC input voltage
   _delay_us(10);
   //start conversation
   ADCSRA |= 0x40; // set Bit 6 � ADSC: ADC Start Conversion 
   
}

void displeyVal(void){
    if (LcdNeed2refresh == 0) {return;} else {LcdNeed2refresh = 0;};
    lcd_return_home();

    itoa(mode==1? settedVoltage:CurrentVoltage, bufStr, 10);

    // display voltage in first line
    for (int i = 0 ; i < LCD_COL_COUNT; i++) lcdStr0[i] = ' ';
    char len;
    if (mode != 1 || blink==1 )
    {
        
        lcdStr0[0] = 'U';
        lcdStr0[1] = ':';
        
        len = strlen(bufStr);
        switch (len) {
            case 4: lcdStr0[2] = bufStr[0]; lcdStr0[3] = bufStr[1]; lcdStr0[4] = ','; lcdStr0[5] = bufStr[2]; lcdStr0[6] = bufStr[3]; break;
            case 3: lcdStr0[2] = bufStr[0]; lcdStr0[3] = ','; lcdStr0[4] = bufStr[1]; lcdStr0[5] = bufStr[2];  break;
            case 1: lcdStr0[2] = '0'; lcdStr0[3] = ','; lcdStr0[4] = '0';  lcdStr0[5] = bufStr[0];  break;
            case 2: lcdStr0[2] = '0'; lcdStr0[3] = ','; lcdStr0[4] = bufStr[0]; lcdStr0[5] = bufStr[1]; break;
            
        }

        lcdStr0[len==4?7:6] = ' ';
        lcdStr0[len==4?8:7] = 'V';
    }
    // display current in second line
    itoa(mode==2 ? settedCurrent: CurrentCurrent, bufStr, 10);
    len = strlen(bufStr);
    for (int i = 0 ; i < LCD_COL_COUNT; i++) lcdStr1[i] = ' ';
    if ( mode != 2  || blink == 1) {
        lcdStr1[0] = 'I';
        lcdStr1[1] = ':';

        switch (strlen(bufStr)) {
            case 4: lcdStr1[2] = bufStr[0]; lcdStr1[3] = bufStr[1]; lcdStr1[4] = ','; lcdStr1[5] = bufStr[2]; lcdStr1[6] = bufStr[3];  break;
            case 3: lcdStr1[2] = bufStr[0]; lcdStr1[3] = ','; lcdStr1[4] = bufStr[1]; lcdStr1[5] = bufStr[2];  break;
            case 1: lcdStr1[2] = '0'; lcdStr1[3] = ','; lcdStr1[4] = '0';  lcdStr1[5] = bufStr[0];  break;
            case 2: lcdStr1[2] = '0'; lcdStr1[3] = ','; lcdStr1[4] = bufStr[0]; lcdStr1[5] = bufStr[1];  break;
        }
        lcdStr1[len==4?7:6] = ' ';
        lcdStr1[len==4?8:7] = 'A';
    }
    lcd_puts(lcdStr0);
    lcd_set_cursor(0,1);
    lcd_puts(lcdStr1);

    //debug info
  /*  
     lcd_set_cursor(12,0);
     itoa(OCR1B, bufStr, 10);
     lcd_puts(bufStr);
    
    lcd_set_cursor(12,1);
    itoa(OCR1A, bufStr, 10);
    lcd_puts(bufStr);
    
   */
    
    

}

void initPorts(void){
    // current limit led  (pb0), pwm pins pb1, pb2 set as output  
    DDRB  = 0x07;
    
    // encoder pins as input
    DDRB &= ~(7<<5);
    PORTB|= (7<<5); //pull up
    //ADT
    //http://narodstream.ru/avr-urok-22-izuchaem-acp-chast-2/
    ADCSRA |= (1<<ADEN) // ���������� ������������� ���
    |(1<<ADPS2)|(1<<ADPS1);//�������� 64 = 128 ���
    ADMUX |= (1<<REFS1)|(1<<REFS0); //���������� �������� �� 2,56�, ���� ADC0

}

void initTimers(void){
    TIMSK =(1<<TOIE0);  // timer0 OF interrupt enable
    TCCR0 = (1<<CS02); // prescaler for timer 0  = 1/256
    // timer 1 pwm for 2 channel
    TCCR1A = 0b10100011; //COM1x1 + COM1x2 = 10b (non invert fast pwm) FOC1A = 0, FOC1B = 0, WGM11 = 1, WGM10 = 1
    TCCR1B = 0b00001001; //ICNC1 =0, ICES1 =0, -, WGM13 = 0, WGM12 = 1, SC12 = 0, SC11=0, SC10 = 1 // 10 bit fast PWM , no prescaling

}
// should be call after interrupt enable
void initADC(void){
    ADMUX = 0b11000010; // REFS1:0 =11 use internam 2.56 v, ARLAR = 0 the result is right adjusted, 0, MUX3:0 = 0010 (pc2 - voltage)
    ADCSRA = 0b11001111; // ADEN = 1 ADC enable, ADSC = 1 - ADC Start Conversion, ADFR = 0 - no free run mode, ADIF - no change, ADIE = 1: ADC Interrupt Enable, ADPS2:0= 000  - ADC Prescaler = 2  
    // set to 0 value
    OCR1B = 0;
    OCR1A = 0;    
}



enum errors startTest()
{
	// display test
    lcd_return_home();
    lcd_puts("Firmware: ");
    lcd_puts(VERSION);
    lcd_set_cursor(0,1);
    lcd_puts("Starting test...");
    CUR_LIMIT_ON;
    // set value to PWM 2v .5A
    OCR1B = VAL2PWM10(200, VOLTAGE_MAX, VOLTAGE_MAX_PWM);
    OCR1A = VAL2PWM10(50, CUR_MAX, CUR_MAX_PWM);
    _delay_ms(500);
    //check current voltage
   if (CurrentVoltage == 0) {
        lcd_set_cursor(0,1);
        lcd_puts("U=0v Err");
        return NO_VOLTAGE;
        }  
     if (abs (CurrentVoltage - 200) > 50) {
         lcd_set_cursor(0,1);
         lcd_puts("U is wrong. Err");
         return BAD_VOLTAGE;
     }
  _delay_ms(3000);
  lcd_clear();
  return NO_ERROR;
}

int main(void)
{
    
    initPorts();
    initTimers();
    
    lcd_clear();
    lcd_init();    
    lcd_on(); 
    
    
    
    
    sei();    
    initADC();
    
    ErrCode = startTest();
    
    if (ErrCode == NO_ERROR) {
    
    settedVoltage =  eeprom_read_word(&settedVoltageEeprom);
    settedCurrent = eeprom_read_word(&settedCurrentEeprom);
    
    if (!(settedVoltage >=0 && settedVoltage <= VOLTAGE_MAX)) settedVoltage = 500; //default 5v
    if (! (settedCurrent >=0 && settedCurrent <= CUR_MAX)) settedCurrent = 200; //default 2A
    
    // set value to PWM
    OCR1B = VAL2PWM10(settedVoltage, VOLTAGE_MAX, VOLTAGE_MAX_PWM);
    OCR1A = VAL2PWM10(settedCurrent , CUR_MAX, CUR_MAX_PWM);
    }    
     
    while (1)
    {
        if (ErrCode != NO_ERROR) continue;
        displeyVal();
        
    }
}

