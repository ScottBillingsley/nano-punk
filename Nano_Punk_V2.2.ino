/*
                          Nano Punk
                          Vernon Billingsley 2021

                        A version of an Atari Punk on a nano

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

/************************** Variables *****************************/
/*About the pulse */
volatile uint16_t pulse_freq = 255;
volatile uint16_t pulse_base = 125;
volatile uint16_t pulse_width = 125;   /* 125 uS width */
volatile uint16_t pulse_count = 0;
volatile boolean pulse_high = true;

/*About the ADC */
volatile uint16_t adc_check = 64; /* 8 mS */
volatile uint8_t adc_count = 1;
uint16_t adc_array[3];
uint8_t adc_mask = 0b01000001;
boolean update_adc = false;
boolean new_sample = false;

boolean update_adc0 = false;
boolean update_adc1 = false;

/*About filter1 */
const float alpha = .0625;
uint32_t f_v[3];
/*Define average array size */
const uint8_t a_size = 5;
/*Define average array */
uint32_t a[a_size];
uint8_t a_index = 0;
uint32_t a_sum = 0;
volatile uint16_t filter1_sample;
uint16_t filter1_sample_old;

/*About filter2 */
const float alpha2 = .0625;
uint32_t f_b[3];
/*Define average array size */
const uint8_t b_size = 5;
/*Define average array */
uint32_t b[b_size];
uint8_t b_index = 0;
uint32_t b_sum = 0;
volatile uint16_t filter2_sample;
uint16_t filter2_sample_old;

/*About the square wave */
volatile boolean sqr_high = false;
volatile uint16_t sqr_base = 500;  /* 500 uS */
volatile uint16_t sqr_width = 0;

volatile uint32_t sqr_time = 0;
volatile uint16_t adc_time = 0;


unsigned long enter;

/**************************  Functions ****************************/
uint16_t filter1(uint16_t samp) {
  /*Map the samp to a new range */
  const float x_min = 0;
  const float x_max = 1024.0;
  const float y_min = 3572.0;
  const float y_max = 25.0;

  /*Filters and averages a sample in aprox 120 uS */
  f_v[1] = (float)alpha * samp + (1 - alpha) * f_v[0] ;
  f_v[0] = f_v[1];
  /*Remove the oldest sample*/
  a_sum -= a[a_index];
  /*Add newest sample */
  a[a_index] = f_v[0];
  /*Add new sample to sum */
  a_sum += f_v[0];
  /*Increment the index */
  a_index = (a_index + 1) % a_size;
  float temp = (a_sum / a_size);

  /* Return the filtered and averaged value and remapped sample */
  return (uint16_t) (y_min + (((y_max - y_min) / (x_max - x_min)) * (temp - x_min)));
}

uint16_t filter2(uint16_t samp) {
  /*Map the samp to a new range */
  const float x_min = 0;
  const float x_max = 1024.0;
  const float y_min = 0;
  const float y_max = 32768.0;

  /*Filters and averages a sample in aprox 120 uS */
  f_b[1] = (float)alpha2 * samp + (1 - alpha2) * f_b[0] ;
  f_b[0] = f_b[1];
  /*Remove the oldest sample*/
  b_sum -= b[b_index];
  /*Add newest sample */
  b[b_index] = f_b[0];
  /*Add new sample to sum */
  b_sum += f_b[0];
  /*Increment the index */
  b_index = (b_index + 1) % b_size;
  float temp = (b_sum / b_size);

  /* Return the filtered, averaged, and remapped sample */
  return (uint16_t) (y_min + (((y_max - y_min) / (x_max - x_min)) * (temp - x_min)));
}

/******************************************************************/
/*************************** Setup ********************************/
/******************************************************************/
void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }
  /************************* Setup Pins ***************************/

  /*Interrupt pin 2 as OUTPUT */
  DDRD |= _BV (2);
  /*Set HIGH to start */
  PORTD |= _BV (2);

  /*Pin D7 as square wave OUTPUT */
  DDRD |= _BV (7);

  /************************ Setup Interrupt 0 *********************/
  /*Interrupt 0 , falling edge */
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
  // set compare match register
  OCR1A = 100;
  // turn on CTC mode
  sbi(TCCR1B, WGM12);
  /*Set prescaler to 64 */
  cbi(TCCR1B, CS12);
  sbi(TCCR1B, CS11);
  sbi(TCCR1B, CS10);
  // enable timer compare interrupt
  sbi(TIMSK1, OCIE1A);

  /***********************  Setup Timer2  ************************/
  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register
  OCR2A = 249;
  // turn on CTC mode
  sbi(TCCR2A, WGM21);
  /*Set prescaler to 8 */
  cbi(TCCR2B, CS22);
  sbi(TCCR2B, CS21);
  cbi(TCCR2B, CS20);
  // enable timer compare interrupt
  sbi(TIMSK2, OCIE2A);

  sei();                //allow interrupts
}/**************************  End Setup **************************/

ISR(TIMER1_COMPA_vect) {
  /*Set pulse pin low */
  PORTD &= ~_BV (2);
  pulse_high = false;
  if (update_adc0) {
    OCR1A = pulse_base + filter1_sample;
    update_adc0 = false;
  }
}

ISR(TIMER2_COMPA_vect) {
    sqr_time += 100;
  /********** Check the time of the output square wave */
  if (sqr_high && sqr_time >= sqr_width) {
    /*Set the pin LOW */
    PORTD &= ~_BV (7);
    /*reset the boolean */
    sqr_high = false;
  }    
    adc_time ++;
}

/*ISR to handle pulse on pin 2 */
ISR(INT0_vect) {
  if (!sqr_high) {
    /*Set the pin HIGH */
    PORTD |= _BV (7);
    /*Reset the count */
    sqr_time = 0;
    /*Set the boolean */
    sqr_high = true;
    if (update_adc1) {
      sqr_width = sqr_base + filter2_sample;
      update_adc1 = false;
    }
  }

}


/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {

  /*If the trigger pulse is LOW, return it to HIGH */
  if (!pulse_high ) {
    /*Set pulse pin high */
    PORTD |= _BV (2);
    /*Reset the boolean */
    pulse_high = true;
  }

  /*Aprox 8 uS per read every 8 mS */
  if (adc_time >= adc_check) {
    /*Check to see if ADC has finished */
    if (!(bitRead(ADCSRA, ADSC))) {
      /*Read and store the results  */
      uint8_t temp_adcl = ADCL;
      uint16_t temp_adc = (ADCH << 8) + temp_adcl;
      /*Keep a running average */
      adc_array[adc_count] = (adc_array[adc_count] + temp_adc) / 2;

      /*Increment the ADC count */
      adc_count ^= 0x01;

      /*Increment the ADC MUX */
      cli();                //stop interrupts
      /* XOR reverses logic so count 0 needs to be MUX 1 */
      ADMUX = (adc_mask ^ !adc_count);
      /*Start the next conversion */
      sbi(ADCSRA, ADSC);
      sei();                //allow interrupts
      /*Update ADC 0 when reading ADC 1 */
      update_adc = !adc_count;
      /*Set the boolean */
      new_sample = true;
    }
    /*Reset the count */
    adc_time = 0;
  }



  /************ Update the new sample *****************/
  if (new_sample && update_adc) {
    /*Send the sample to the filter */
    filter1_sample = filter1(adc_array[0]);
    /*If the new sample is different than the old */
    /*Update the timer */
    if (filter1_sample != filter1_sample_old) {
      update_adc0 = true;
    }
    /*Store the last sample */
    filter1_sample_old = filter1_sample;
    new_sample = false;
  }
  if (new_sample && !update_adc) {
    filter2_sample = filter2(adc_array[1]);
    if (filter2_sample != filter2_sample_old) {
      update_adc1 = true;
    }
    filter2_sample_old = filter2_sample;

    new_sample = false;
  }

}/*************************** End Loop *****************************/
