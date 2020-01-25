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

enum modes {NORMAL_MODE, VOLTAGE_SET_MODE, CURRENT_SET_MODE, CHARGE_SET, CHARGE_PROGRESS, CHARGE_STOP} mode; // 0 - work mode (display cur volumes), 1 - set voltage, 2 - set current
enum errors {NO_ERROR, NO_VOLTAGE, BAD_VOLTAGE, UNKNOWN} ErrCode = UNKNOWN;    
enum adcInputs {VOLTAGE_ADC = 0b0010, CURRENT_ADC = 0b0000, TERM_ADC = 0b0011} AdcInput = VOLTAGE_ADC;    
enum ledModes {OFF, ON, BLINK, FAST_BLINK} ledMode;

#define VAL2PWM10(val, maxval, maxpwm) (uint32_t)val* maxpwm/maxval
#define CUR_LIMIT_ON PORTB |= 0x01
#define CUR_LIMIT_OFF PORTB&=0xFE
#define CUR_LIMIT_INV PORTB^=0x01
#define CUR_LIMIT_DEV 5
#define BLINK_POST_SCAL 25
#define BLINK_POST_SCAL2 10
#define ENCODER_STEP 10
#define LONG_PUSH_COUNT 10

#define  VOLTAGE_MAX 2600
#define  VOLTAGE_MAX_PWM 1023 //up to 5v (pwm 100%)
#define  CUR_MAX 1700 
#define  CUR_MAX_PWM 511 // up to 2.5v because use one value for ADC ( has limit on 2.56v ) and for lm464

#define  MAX_POWER 3500000 // 350 W  

#define MAX_CHARDGE_VOLTAGE 1450

// shunt resist 0.04 ohm 



uint8_t textBlink = 1;

uint8_t blinkPostScal =  BLINK_POST_SCAL;
uint8_t blinkPostScal2 = BLINK_POST_SCAL2;
uint16_t  volatile settedVoltage = 500;
uint16_t  settedVoltageEeprom EEMEM = 0;
uint16_t  volatile settedCurrent = 200;
uint16_t  settedChargeCurrent;
uint16_t  settedCurrentEeprom EEMEM = 0 ;
uint16_t  volatile CurrentCurrent = 0;
uint16_t  volatile CurrentVoltage = 0;
uint16_t volatile AdcAvgBuf;
uint8_t volatile AdcCount = 0;
uint8_t volatile AdcCountAvg = 0;
uint16_t volatile AdcvBuf;

uint8_t volatile  LcdNeed2refresh = 0;

uint8_t volatile  longPush = 0;


uint16_t  settedChargeCurrentEeprom EEMEM = 0 ;
uint16_t  settedChargeCurrent = 200;

//uint8_t ErrCode = 0;
uint8_t lastEncoder;

char lcdStr[16+1];
char bufStr[8+1];


void readEncoder(void){
    //mode = PORTB;
    uint8_t curEncoder = PINB & (7<<5);
    if (lastEncoder  == 0 || curEncoder == lastEncoder || ErrCode) {lastEncoder = curEncoder; return;}
        
    // check press button
    if (curEncoder & (1 << PINB7) && !(lastEncoder & (1 << PINB7) )) {
        ++longPush;
       // mode = (mode+1) % 3;
       /* switch (mode) { 
            case NORMAL_MODE: eeprom_write_word(&settedCurrentEeprom, settedCurrent);
            case CURRENT_SET_MODE: eeprom_write_word(&settedVoltageEeprom, settedVoltage);
            default: break;
            }*/
    }
    
    //check return button
    if (lastEncoder & (1 << PINB7) && !(curEncoder & (1 << PINB7) )){
      if (longPush >= LONG_PUSH_COUNT){
          switch (mode) {
              case NORMAL_MODE: mode = CHARGE_SET; 
                                settedVoltage = 0;
                                settedChargeCurrent = eeprom_read_word(&settedChargeCurrentEeprom);
                                if (settedCurrent > CUR_MAX) settedCurrent = 50;
                                break;
              case CHARGE_SET: mode = CHARGE_PROGRESS; 
                               eeprom_write_word(&settedChargeCurrentEeprom, settedChargeCurrent); 
                               settedVoltage = MAX_CHARDGE_VOLTAGE;
                               settedCurrent = settedChargeCurrent;
                               break;
              case CHARGE_PROGRESS: mode = CHARGE_STOP;
                                    settedVoltage = 0; settedCurrent = 0;
                                    break;
              default: break;
          }
          
      }
          else {
              switch (mode) {
                  case NORMAL_MODE: mode = VOLTAGE_SET_MODE; break;
                  case VOLTAGE_SET_MODE: mode = CURRENT_SET_MODE; eeprom_write_word(&settedVoltageEeprom, settedVoltage); break; 
                  case CURRENT_SET_MODE: mode = NORMAL_MODE; eeprom_write_word(&settedCurrentEeprom, settedCurrent); break;                  
                  default: break;
              }
              
          }
      
      longPush = 0;  
        
        
    }
    
    
    // to left
    if ((curEncoder==128 && lastEncoder == 192 )|| (curEncoder == 160 && lastEncoder == 128)) {
        switch (mode) {
            case VOLTAGE_SET_MODE:
            if (settedVoltage>=ENCODER_STEP) settedVoltage-=ENCODER_STEP;             
             break;
            case CURRENT_SET_MODE:
            if (settedCurrent>=ENCODER_STEP) settedCurrent-=ENCODER_STEP; break;
            case CHARGE_SET:
            if (settedChargeCurrent>=ENCODER_STEP) settedChargeCurrent-=ENCODER_STEP; break;
            default: break;
        }

    }
    // to right
    if ((curEncoder == 128 && lastEncoder == 160) || (curEncoder == 192 && lastEncoder == 128)) {
        switch (mode) {
            case VOLTAGE_SET_MODE:
            if (settedVoltage < VOLTAGE_MAX - ENCODER_STEP) settedVoltage+=ENCODER_STEP; 
            // check power
            if (settedVoltage * settedCurrent > MAX_POWER) settedCurrent = MAX_POWER/settedVoltage;
            break;
            case CURRENT_SET_MODE:
            if (settedCurrent < CUR_MAX - ENCODER_STEP)  settedCurrent+=ENCODER_STEP; 
            // check power
            if (settedVoltage * settedCurrent > MAX_POWER) settedVoltage = MAX_POWER/settedCurrent;            
            break;
            
            case CHARGE_SET: 
            if (settedChargeCurrent < CUR_MAX)  settedChargeCurrent+=ENCODER_STEP; 
            default: break;
        }
    }
    // set value to PWM
    OCR1B = VAL2PWM10(settedVoltage, VOLTAGE_MAX, VOLTAGE_MAX_PWM);
    OCR1A = VAL2PWM10(settedCurrent , CUR_MAX, CUR_MAX_PWM);
    
    // refresh Current Led
   /* 
   Disabled because blinked then encoder is moving
   if (ErrCode == NO_ERROR){
    if (ErrCode == NO_ERROR && abs(CurrentCurrent - settedCurrent) <= CUR_LIMIT_DEV) CUR_LIMIT_ON; else CUR_LIMIT_OFF;}*/
    
    // set that display should be refreshed
    LcdNeed2refresh = 1;
    

    lastEncoder = curEncoder;
}

//interrupts
ISR(TIMER0_OVF_vect){
    if (ledMode == ON) CUR_LIMIT_ON;
    if (ledMode == OFF) CUR_LIMIT_OFF;
    --blinkPostScal;
    readEncoder();
    if (!blinkPostScal){
        if (ledMode == FAST_BLINK)  CUR_LIMIT_INV;
        textBlink=~textBlink;
        blinkPostScal = BLINK_POST_SCAL;        
        ++LcdNeed2refresh;
        
        if (longPush) ++longPush;
        
        --blinkPostScal2;
        if (!blinkPostScal2){
            if (ledMode == BLINK) CUR_LIMIT_INV;
            blinkPostScal2 = BLINK_POST_SCAL2;
            
        }
    }
}

ISR(ADC_vect)
{            
    if (AdcCountAvg == 0) {AdcAvgBuf = ADCW;   } else {AdcAvgBuf+=ADCW;}  
    if (AdcInput == VOLTAGE_ADC)   AdcvBuf  =      ADCW; 
    if (AdcCountAvg == 3) {
        
        switch(AdcInput){
            case VOLTAGE_ADC:
                        CurrentVoltage = (uint32_t)AdcAvgBuf * (uint32_t)VOLTAGE_MAX /((uint16_t)VOLTAGE_MAX_PWM << 2) ; break; 
            case CURRENT_ADC: 
                        CurrentCurrent =  (uint32_t)AdcAvgBuf * (uint32_t)CUR_MAX / ((uint16_t)CUR_MAX_PWM << 2) ; break;  
            case TERM_ADC:     break; // need todo   
            
        }
        // in charge mode check that process complete
        if (CurrentCurrent < 5) {mode = CHARGE_STOP, settedVoltage = 0; settedCurrent = 0; ledMode = ON;}  
        // refresh Current Led
        if (ErrCode == NO_ERROR && (mode == NORMAL_MODE || mode == VOLTAGE_SET_MODE || mode == CURRENT_SET_MODE)){
        if ( abs(CurrentCurrent - settedCurrent) <= CUR_LIMIT_DEV) ledMode = ON; else ledMode = OFF;}
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
   ADCSRA |= 0x40; // set Bit 6 – ADSC: ADC Start Conversion 
   
}

void copyValtoStr(char* lcdStr, uint16_t val )
{
    char bufStr[8+1];
    itoa(val, bufStr, 10);
    uint8_t  i;  
    //char* last = lcdStr+4;
    lcdStr[5] = 0; // end of string 
    i  = strlen(bufStr);
    
    for (char* last = lcdStr+4; last >= lcdStr; last--) {
        if (last == lcdStr+2) {last[0] = '.'; continue;}
        
                if (i)  *last = bufStr[--i];
                    else    last[0] = '0';               
        }         
        
}

void displeyVal(void){
    if (LcdNeed2refresh == 0) return; else LcdNeed2refresh = 0;
    lcd_return_home();
    lcd_clear();
    switch (mode) {
    // in charge setting display only charge setting
    // Charge current:
    // I:00.00 A
    case CHARGE_SET: lcd_puts("Charge current:"); 
                     if(!textBlink) break;
                     lcdStr[0] = 'I'; lcdStr[1] = ':';
                     copyValtoStr(lcdStr+2, settedChargeCurrent);
                     lcdStr[7] = ' '; lcdStr[8] = 'A'; lcdStr[9] = 0;
                     lcd_set_cursor(0,1);
                     lcd_puts(lcdStr);
                     break;
   
        
        // in charging progress  display only current value
        // Charging...
        // I:00.00 U:00.00
        case CHARGE_PROGRESS:   lcd_puts("Charging...");
                                lcdStr[0] = 'I'; lcdStr[1] = ':';
                                copyValtoStr(lcdStr+2, CurrentCurrent);
                                lcdStr[7] = ' '; 
                                lcdStr[8] = 'U';
                                lcdStr[9] = ':';
                                copyValtoStr(lcdStr+10, CurrentCurrent);
                                lcd_set_cursor(0,1);
                                lcd_puts(lcdStr);
                                break;
        
        // in mode charging complete display only info message
        // CHARGING COMPLETE
        // !!!!!!!!!!!!!!!!!
        
        case CHARGE_STOP:   lcd_puts("CHARGING COMPLETE");
                            lcd_set_cursor(0,1);
                            lcd_puts("!!!!!!!!!!!!!!!!!");
                            break;

    //itoa(mode==VOLTAGE_SET_MODE? settedVoltage:CurrentVoltage, bufStr, 10);

    // in mode NORMAL and VOLTAGE_SET and CURRENT_SET
    //display voltage in first line : U:00.00->00.00 V
    case NORMAL_MODE:
    case  VOLTAGE_SET_MODE:
    case  CURRENT_SET_MODE: lcdStr[0] = 'U';
                            lcdStr[1] = ':';
                            copyValtoStr(lcdStr+2, CurrentVoltage);
                            lcdStr[7] = '-'; lcdStr[8] = '>';
                            if (mode == VOLTAGE_SET_MODE  && !textBlink)  for (uint8_t i = 9; i < 14; i++) lcdStr[i] = ' '; else copyValtoStr(lcdStr+9, settedVoltage);
                            lcdStr[14] = ' '; lcdStr[15] = 'V'; lcdStr[16] = 0;
                            lcd_puts(lcdStr);   

    // display current in second line : I:00.00->00.00 A
    
                            lcdStr[0] = 'I';
                            copyValtoStr(lcdStr+2, CurrentCurrent);
                            if (mode == CURRENT_SET_MODE  && !textBlink)  for (uint8_t i = 9; i < 14; i++) lcdStr[i] = ' '; else copyValtoStr (lcdStr+9, settedCurrent);
                             lcdStr[15] = 'I'; lcdStr[16] = 0;
                            lcd_set_cursor(0,1);
                            lcd_puts(lcdStr);  
    
    }
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
    ADCSRA |= (1<<ADEN) // Разрешение использования АЦП
    |(1<<ADPS2)|(1<<ADPS1);//Делитель 64 = 128 кГц
    ADMUX |= (1<<REFS1)|(1<<REFS0); //Внутренний Источник ОН 2,56в, вход ADC0

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
    OCR1B = VAL2PWM10(100, VOLTAGE_MAX, VOLTAGE_MAX_PWM);
    OCR1A = VAL2PWM10(30, CUR_MAX, CUR_MAX_PWM);
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
    
  /*  uint16_t test = 25;
    char ts[16+1]; ts[0] = 'U'; ts[1] = ':';
    copyValtoStr(ts+2, test);*/
    
    
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

