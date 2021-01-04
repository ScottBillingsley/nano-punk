/*
                          Nano Punk
                          Vernon Billingsley 2021

                        A version of an Atari Punk on a nano

                        Connect pin D8 to D2
                        Output on D7
                        0 to 5 volts on 
                        A0      Freq
                        A1      Pulse Width

   Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission
    notice shall be included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.                        
                           
 */
#include <elapsedMillis.h>

/************************* Defines ********************************/
#define DEBUG 1

#if DEBUG == 1
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
#define dshow(expression) Serial.println( expression )
#else
#define dprint(expression)
#define dshow(expression)
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

elapsedMicros pulse_time;
elapsedMillis adc_time;
elapsedMicros sqr_time;

/************************** Variables *****************************/
/*About the pulse */
volatile uint16_t pulse_freq = 255;
volatile uint16_t pulse_base = 125;
volatile uint16_t pulse_width = 125;   /* 125 uS width */
volatile uint16_t pulse_count = 0;
volatile boolean pulse_high = true;

/*About the ADC */
volatile uint16_t adc_check = 10; /* 10 mS */
volatile boolean next_adc = false;
uint8_t adc_count = 0;
uint16_t adc_array[3];
boolean update_adc0 = false;
boolean update_adc1 = false;

/*About the square wave */
volatile boolean sqr_high = false;
volatile uint16_t sqr_base = 500;  /* 500 uS */
volatile uint16_t sqr_width = 0;  


/**************************  Functions ****************************/


/******************************************************************/
/*************************** Setup ********************************/
/******************************************************************/
void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }
  /************************* Setup Pins ***************************/
  /*Pin D8 ,pulse pin as OUTPUT */
  DDRB |= _BV (0);
  /*Set HIGH to start */
  PORTB |= _BV (0);

  /*Interrupt pin 2 as INPUT */
  DDRD &= ~_BV (2);

  /*Pin D7 as square wave OUTPUT */
  DDRD |= _BV (7);

  /************************ Setup Interrupt 0 *********************/
  /*Interrupt 0 , rising edge */
  cbi(EICRA, ISC00);
  sbi(EICRA, ISC01);

  /*enable interrupt */
  sbi(EIMSK, INT0);


  /*************************  Setup ADC ***************************/
  /*Set to Right Adjust for 1024 precision */
  cbi(ADMUX, ADLAR);

  /*Set to VRef to AVCC */
  cbi(ADMUX, REFS1);
  sbi(ADMUX, REFS0);

  /*Set to ADC1 to start */
  cbi(ADMUX, MUX3);
  cbi(ADMUX, MUX2);
  cbi(ADMUX, MUX1);
  sbi(ADMUX, MUX0);

  /*Set prescaler to 64 */
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  /*Turn off Auto Trigger */
  cbi(ADCSRA, ADATE);

  /*Turn the ADC ON  */
  sbi(ADCSRA, ADEN);

  /*Start the first conversion */
  sbi(ADCSRA, ADSC);

  /*************************  Setup Timer1 ************************/
  cli();                //stop interrupts
  //set timer1
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register 16000000 / 256
  OCR1A = 100;
  // turn on CTC mode
  sbi(TCCR1B, WGM12);
  /*Set prescaler to 64 */
  cbi(TCCR1B, CS12);
  sbi(TCCR1B, CS11);
  sbi(TCCR1B, CS10);
  // enable timer compare interrupt
  sbi(TIMSK1, OCIE1A);
  sei();                //allow interrupts
}/**************************  End Setup **************************/

ISR(TIMER1_COMPA_vect) {
  /*Set pulse pin low */
  PORTB &= ~_BV (0);
  pulse_high = false;
  pulse_time = 0;
  if (update_adc0) {
    OCR1A = pulse_base + map(adc_array[0], 0, 1024, 3072, 125);
    update_adc0 = false;
  }
}

/*ISR to handle pulse on pin 2 */
ISR(INT0_vect) {
  if (!sqr_high){
      /*Set the pin HIGH */
      PORTD |= _BV (7);
      /*Reset the count */
      sqr_time = 0;
      /*Set the boolean */
      sqr_high = true;
      sqr_width = sqr_base + map(adc_array[1], 0, 1024, 0, 24567) ;
  }

  }


/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {
  if (!pulse_high && pulse_time > pulse_width) {
    /*Set pulse pin high */
    PORTB |= _BV (0);
    pulse_high = true;
  }

  if (adc_time > adc_check) {
    /*Check to see if ADC has finished */
    if (!(bitRead(ADCSRA, ADSC))) {
      /*Read and store the results  */
      uint8_t temp_adcl = ADCL;
      uint16_t temp_adc = (ADCH << 8) + temp_adcl;
      /*Keep a running average */
      adc_array[adc_count] = (adc_array[adc_count] + temp_adc) / 2;

      /*Increment the ADC count */
      adc_count ++;
      if (adc_count > 1) {
        adc_count = 0;
      }
      /*Increment the ADC MUX */
      cli();                //stop interrupts
      switch (adc_count) {
        case 0:
          cbi(ADMUX, MUX3);
          cbi(ADMUX, MUX2);
          cbi(ADMUX, MUX1);
          cbi(ADMUX, MUX0);
          update_adc1 = true;
          break;
        case 1:
          cbi(ADMUX, MUX3);
          cbi(ADMUX, MUX2);
          cbi(ADMUX, MUX1);
          sbi(ADMUX, MUX0);
          update_adc0 = true;
          break;
      }
      /*Start the next conversion */
      sbi(ADCSRA, ADSC);
      sei();                //allow interrupts

    }
    adc_time = 0;
  }

  if (sqr_high && sqr_time > sqr_width) {
    /*Set the pin LOW */
    PORTD &= ~_BV (7);
    /*reset the boolean */
    sqr_high = false;
  }


}/*************************** End Loop *****************************/
