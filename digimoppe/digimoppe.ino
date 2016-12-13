/*
Pinout for Arduino Nano:
A0 (PC0): Filtered and amplified 455 kHz IF from radio
A2 (PC2): Less significant bit for 2-bit DAC controlling modulator
A3 (PC3): More significant bit
A5 (PC5): PTT control. 1 to transmit, 0 to receive

Debugging outputs:
12 (PB4): Square wave at frequency half of the ADC sample rate
11 (PB3): Pulses high if UART is still transmitting when new sample arrives. UART is too slow.
10 (PB2): Pulses high if modulator buffer gets full. PC sends data too fast.
*/



/* mbuf is a buffer for the symbols to be sent to the modulator.
   The size is 0x100 so we can conveniently use wrap-around of unsigned 8-bit integers
   when dealing with indexes to the buffer.
   mwp is the index where buffer is being written,
   mrp is the index where buffer is being read.
   Buffer is empty when mwp == mrp.
   MBUFFILL sets how many bytes to try to keep in the buffer.
   We ask PC for more data when there's less bytes waiting. */
uint8_t mbuf[0x100];
volatile uint8_t mwp = 0, mrp = 0;
#define MBUFFILL 100

volatile uint16_t ad=0;
volatile uint8_t  adfull=0, mhalffull=0, uarttest=0;

#define MODULATORBIT 2
#define PTTBIT 5

void setup() {
  pinMode(2, INPUT_PULLUP);
  DDRB |= (1<<4) | (1<<3) | (1<<2); // PB4 = digital pin 12, PB3 = digital pin 11
  DDRC |= (3<<MODULATORBIT) | (1<<PTTBIT);
  PORTB = 0;

  //Serial.begin(1000000);
  // Serial.begin can't be used if we have own USART interrupt handler
  UCSR0A = 0; // don't double speed
  UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1<<TXEN0); // enable UART receive interrupt
  UCSR0C = (0<<UMSEL00) | (0<<UPM00) | (0<<USBS0) | (3<<UCSZ00); // 8N1
  UBRR0 = 0; // 1 Mbaud

  if(!digitalRead(2)) { // UART test mode
    uarttest = 1;
  } else {
    /* ADC prescaler is 32 for 500 kHz clock.
       Timer1 prescaler is 8, so counting to 4*14 results in 14 ADC clocks
       for a sample rate of 16 MHz / 32 / 14 = 35714 Hz. */
    TCCR1A = 0; // WGM10=0, WGM11=0
    TCCR1B = 1<<WGM12; // WGM12=1: clear timer on compare match A
    OCR1A = 4 * 14 -1;
    OCR1B = 0;
    TIMSK1 = 1<<OCIE1B; // interrupt on match B (it also triggers ADC)
    TCNT0 = 0;


    // ADC settings
    ADMUX = (3<<REFS0) | (0<<ADLAR) | (0<<MUX0); // right aligned
    ADCSRB = (5<<ADTS0); // trigger on timer1 compare match B
    DIDR0 = (1<<ADC0D);
    ADCSRA = (1<<ADATE) | (5<<ADPS0); // no interrupt, auto trigger, prescaler 32

    sei();
    // enable ADC and start conversion last
    ADCSRA |= (1<<ADEN);
    TCCR1B |= (2<<CS10); // start timer1: prescale by 8
  }
}

uint8_t testi=0;
void loop() {
  if(uarttest) {
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = testi;
    testi++;
  } else
  if(adfull) {
    uint8_t txbyte0, txbyte1;
    // send lowest 5 bits first, then highest 5 bits
    // highest bit of first byte is always 1
    txbyte0 = 0x80 | (ad & 0x1F);
    txbyte1 = 0x1F & (ad >> 5);
    adfull=0;
    if(mhalffull) {
      // flow control: tell the PC to stop sending data
      txbyte0 |= 0x40;
      txbyte1 |= 0x40;
    }
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = txbyte0;
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = txbyte1;
  }
}


uint16_t symbolcounter = 0;
/* Result is read in TIMER1 ISR instead of ADC ISR.
   This is done because ADC is triggered by the timer interrupt flag
   and it's not triggered again before the interrupt flag is cleared.
   It can be cleared manually in ADC ISR but then it's cleared too late
   resulting in every second trigger to be missed.
   Using the TIMER1 ISR clears it automatically and we can use it to also
   read the ADC result to avoid the need for separate ADC interrupt,
   saving some time. */
ISR(TIMER1_COMPB_vect) {
  uint8_t rp, n;
  PORTB ^= 1<<4; // PB4 = digital pin 12

  if(!adfull) {
    PORTB &= ~(1<<3);
    ad = ADCW;
    adfull = 1;
  } else {
    PORTB |= 1<<3; // UART too slow, put PB3 high as "error message"
  }

  /* This interrupt occurs (16e6 / 32 / 14) times per second.
     We want to send a symbol 1200 times per second on average.
     The ratio of these values is 0.0336, or 21/625. */
  symbolcounter += 21;
  if(symbolcounter >= 625) {
    symbolcounter -= 625;
    rp = mrp;
    n = mwp - rp; // number of bytes in buffer
    if(n) {
      PORTC = ((3 & mbuf[rp])<<MODULATORBIT) | (1<<PTTBIT);
      mrp = rp+1;
    } else {
      // empty buffer: turn transmitter off
      PORTC = 0;
    }
    mhalffull = (n >= MBUFFILL);
  }
}


ISR(USART_RX_vect) {
  uint8_t b, nextwp;
  b = UDR0;
  nextwp = mwp+1;
  if(nextwp == mrp) {
    // full buffer!
    PORTB |= 1<<2;
  } else {
    mbuf[nextwp] = b;
    mwp = nextwp;
    PORTB &= ~(1<<2);
  }
}

