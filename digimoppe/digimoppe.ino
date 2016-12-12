uint8_t uarttest = 0;

uint8_t mbuf[0x100]; // buffer for modulator data
volatile uint8_t mwp = 0, mrp = 0; // write and read pointers

volatile uint16_t ad=0;
volatile uint8_t  adfull=0, mhalffull=0;

#define MODULATORBITS 0b111110
void setup() {
  pinMode(2, INPUT_PULLUP);
  DDRB |= (1<<4) | (1<<3) | (1<<2); // PB4 = digital pin 12, PB3 = digital pin 11
  DDRC |= MODULATORBITS;
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
      txbyte0 |= 0x40;
      txbyte1 |= 0x40;
    }
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = txbyte0;
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = txbyte1;
  }
}


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

  rp = mrp;
  n = mwp - rp; // number of bytes in buffer
  if(n) {
    PORTC = MODULATORBITS & mbuf[rp];
    mrp = rp+1;
  }
  mhalffull = (n >= 0x80);
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

