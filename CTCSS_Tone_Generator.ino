/*
 Copyright 2018 by Dennis Cabell
 KE8FZX

 This program is generates CTCSS tones.

# CTCSS_Tone_Generator

 This program is generates CTCSS tones.

 Circuit info:
 The output is PWM and must be passed though low pass filters (I used 2 RC filters) to create a sine wave, then buffered with an op amp, followed by a 
 10k trim put to set the final output level.
 
 Suggested filter cut-off frequency is 300Hz
 
 Frequency response can be flattened by adjusting variables in the generateSineWave() procedure.


 The Rotary Encoder files are attributed to the original authors.  Please see those files for 
 more information
 
 * The Timer2 setup and interrupt routines are from following source.  Copyright for these parts is 
 * by the original author.
 * DDS Sine Generator mit ATMEGS 168
 * Timer2 generates the  31250 KHz Clock Interrupt
 *
 * KHM 2009 /  Martin Nawrath
 * Kunsthochschule fuer Medien Koeln
 * Academy of Media Arts Cologne

 
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.


 */
#include "RotaryEncoder.h"

#define ENCODER_PINA     3
#define ENCODER_PINB     4
#define ENCODER_BTN      2

  #include <LiquidCrystal_I2C.h>

  #define LCD_I2C_ADDR     0x3F
  //#define LCD_I2C_ADDR     0x27
  #define I2C_EN_PIN              2
  #define I2C_RW_PIN              1
  #define I2C_RS_PIN              0
  #define I2C_D4_PIN              4
  #define I2C_D5_PIN              5
  #define I2C_D6_PIN              6
  #define I2C_D7_PIN              7
  #define I2C_BL_PIN              3   //Back light
  #define I2C_BACKLIGHT_POLARITY  POSITIVE
  
  LiquidCrystal_I2C lcd(LCD_I2C_ADDR,   // Set the LCD I2C address
                        I2C_EN_PIN,
                        I2C_RW_PIN,
                        I2C_RS_PIN,
                        I2C_D4_PIN,
                        I2C_D5_PIN,
                        I2C_D6_PIN,
                        I2C_D7_PIN,
                        I2C_BL_PIN,
                        I2C_BACKLIGHT_POLARITY);  

#include "avr/pgmspace.h"

// table of 256 sine values / one sine period
volatile byte sine256[256];

// table of ctcss frequencies / stored in flash memory
const float ctcssFrequencies[]  = {
                67, 69.3, 71, 71.9, 74.4, 77, 79.7, 82.5, 85.4, 88.5,
                91.5, 94.8, 97.4, 100, 103.5, 107.2, 110.9, 114.8, 118.8, 123,
                127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 159.8, 162.2, 165.5,
                167.9, 171.3, 173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6,
                199.5, 203.5, 206.5, 210.7, 218.1, 225.7, 229.1, 233.6, 241.8, 250.3,
                254.1};
const int maxFrequencyIndex = 50;

int frequenceIndex = 13;
                
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

double dfreq = ctcssFrequencies[frequenceIndex];
volatile bool on_off_state = true;
 const double refclk=31372.549;  // =16MHz / 510   -- Raise this number to lower frequency
//const double refclk=31076.6;      // measured
//const double refclk=30976.6;      // measured

// variables used inside interrupt service declared as voilatile
volatile byte c4ms = 255;              // counter incremented all 4ms
volatile unsigned long phaccu;   // pahse accumulator
volatile unsigned long tword_m;  // dds tuning word m

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder2(ENCODER_PINB, ENCODER_PINA);

void setup()
{
  //Serial.begin(74880);
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  pinMode(ENCODER_PINB, INPUT_PULLUP);
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  pinMode(11, OUTPUT);     // pin11= PWM  output / frequency output

  encoder2.setPosition(frequenceIndex);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);lcd.print(F("CTCSS GENERATOR"));

  generateSineWave(dfreq);

  Setup_timer2();

  // disable interrupts to avoid timing distortion
  cbi (TIMSK0,TOIE0);              // disable Timer0 !!! delay() is now not available
  sbi (TIMSK2,TOIE2);              // enable Timer2 Interrupt
  
  tword_m=pow(2,32)*dfreq/refclk;  // calulate DDS new tuning word 
  
  PCICR |= (1 << PCIE2);    // This enables Pin Change Interrupt 2 that covers Port D.
  PCMSK2 |= (1 << PCINT19) | (1 << PCINT20);  // This enables the interrupt for pin 2 and 3 of Port D.
}

void loop()
{
  while(1) {
    static double old_dfreq = -1;
    if (c4ms >= 10) {                 // timer / wait for debounce
      c4ms = 0;
      
      static int prevFrequenceIndex = 0;
      frequenceIndex = encoder2.getPosition();
      if (frequenceIndex != prevFrequenceIndex) {
        frequenceIndex = constrain(frequenceIndex,0,maxFrequencyIndex);
        encoder2.setPosition(frequenceIndex);
        if (frequenceIndex != prevFrequenceIndex) {
          prevFrequenceIndex = frequenceIndex;
          dfreq = ctcssFrequencies[frequenceIndex];
          //value = 0;
          cbi (TIMSK2,TOIE2);              // disble Timer2 Interrupt
          generateSineWave(dfreq);
          tword_m=pow(2,32)*dfreq/refclk;  // calulate DDS new tuning word 
          sbi (TIMSK2,TOIE2);              // enable Timer2 Interrupt 
        }
      }  
      if (old_dfreq != dfreq) {
        lcd.setCursor(0, 1);lcd.print("       ");
        lcd.setCursor(0, 1);lcd.print(dfreq,1); 
        old_dfreq = dfreq;     
      }
      checkButton();
    }
  }
}

void checkButton() {
  static bool old_on_off_state = !on_off_state;
  static bool buttonDown = false;
  static byte PrevButtonState = HIGH;
  
  byte buttonState = digitalRead(ENCODER_BTN);

  if ((buttonState == HIGH) && buttonDown)   {    // make sure button was down at leasst 1 full cycle before registering click
    on_off_state = !on_off_state;
    buttonDown = false;    
  } 
  if ((buttonState == LOW) && PrevButtonState == LOW) {
    buttonDown = true;
  }
  PrevButtonState = buttonState;
  
  if (old_on_off_state != on_off_state) {
    if (on_off_state) {
      lcd.setCursor(12, 1);lcd.print(" ON ");  
    } else {
      lcd.setCursor(12, 1);lcd.print("OFF ");
    }
    old_on_off_state = on_off_state;     
  }  
}

//******************************************************************
// timer2 setup
// set prscaler to 1, PWM mode to phase correct PWM,  16000000/510 = 31372.55 Hz clock
void Setup_timer2() {

// Timer2 Clock Prescaler to : 1
  sbi (TCCR2B, CS20);
  cbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);

  // Timer2 PWM Mode set to Phase Correct PWM
  cbi (TCCR2A, COM2A0);  // clear Compare Match
  sbi (TCCR2A, COM2A1);

  sbi (TCCR2A, WGM20);  // Mode 1  / Phase Correct PWM
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);
}

//******************************************************************
// Timer2 Interrupt Service at 31372,550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
// runtime : 8 microseconds ( inclusive push and pop)
ISR(TIMER2_OVF_vect) {
  static byte icnt1 = 0;
  static byte icnt;

  phaccu=phaccu+tword_m; // soft DDS, phase accu with 32 bits
  icnt=phaccu >> 24;     // use upper 8 bits for phase accu as frequency information
                         // read value fron ROM sine table and send to PWM DAC
  if (on_off_state) {
    OCR2A=sine256[icnt];   
  } 

  if(icnt1++ == 31) {  // increment variable c4ms all ~1 milliseconds
    c4ms++;
    icnt1=0;
  }   
}

void generateSineWave(float freq) {

  float lowFreqAmplitude = 330;
  float HighFreqAmplitude = 120;
  float MidFreqAdjustPct = -.10;

  float lowBound = 0;
  float MaxUpBound = 255;

  float MinUpBound = HighFreqAmplitude/lowFreqAmplitude * MaxUpBound;

  float upBound = mapf(freq,ctcssFrequencies[0],ctcssFrequencies[maxFrequencyIndex],MinUpBound,MaxUpBound);

  // Take out mid freuencey hump in the response
  float midFrequence = (ctcssFrequencies[0] + ctcssFrequencies[maxFrequencyIndex]) / 2.0;  
  if  (freq < midFrequence) {
    upBound = (1 + mapf(freq,ctcssFrequencies[0],midFrequence,0,MidFreqAdjustPct)) * upBound;
  } else {
    upBound = (1 + mapf(freq,midFrequence,ctcssFrequencies[maxFrequencyIndex],MidFreqAdjustPct,0)) * upBound;
  }


  for (int x = 0; x <= 255; x++) {
      float sineValue = sin(mapf((float)x, 0.0, 256.0, 0.0, 2.0 * PI));
      sine256[x] = (byte)constrain((int)(mapf(sineValue, -1.0, 1.0, lowBound, upBound) + .5),0,255);
      //Serial.println(sine256[x]);
  }

}


float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// The Interrupt Service Routine for Pin Change Interrupt 2
// This routine will only be called on any signal change on 3 and 4: exactly where we need to check.
ISR(PCINT2_vect) {
  encoder2.tick(); // just call tick() to check the state.
}
