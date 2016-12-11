uint8_t uarttest = 0;

void setup() {
  DDRB |= (1<<4) | (1<<3); // PB4 = digital pin 12, PB3 = digital pin 11
  PORTB = 0;
  Serial.begin(2000000);
  pinMode(2, INPUT_PULLUP);
  if(!digitalRead(2)) { // UART test mode
    uarttest = 1;
  } else {
    //ADMUX = (3<<REFS0) | (1<<ADLAR) | (0<<MUX0); // left aligned: 8-bit result on ADCH
    ADMUX = (3<<REFS0) | (0<<ADLAR) | (0<<MUX0); // right aligned
    ADCSRB = (0<<ADTS0); // free running mode
    DIDR0 = (1<<ADC0D);
    ADCSRA = (1<<ADIE) | (1<<ADATE) | (4<<ADPS0); // interrupt, auto trigger, prescaler 16

    sei();
    // enable ADC and start conversion last
    ADCSRA |= (1<<ADEN);
    ADCSRA |= (1<<ADSC);
  }
}

volatile uint8_t txbyte0=0, txbyte1=0, txfull=0;

void loop() {
  if(uarttest) {
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = txbyte0;
    txbyte0++;
  } else
  if(txfull) {
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = txbyte0;
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = txbyte1;
    txfull=0;
  }
}

ISR(ADC_vect) {
  uint16_t ad;
  PORTB ^= 1<<4; // PB4 = digital pin 12
  ad = ADCW;
  
  if(!txfull) {
    PORTB &= ~(1<<3);
    // send lowest 7 bits in first byte, highest 3 bits in second byte
    // highest bit of first byte is 1 to allow simple synchronization
    txbyte0 = 0x80 | (ad & 0x7F);
    txbyte1 = 0x07 & (ad >> 7);
    txfull = 1;
  } else {
    PORTB |= 1<<3; // UART too slow, put PB3 high as "error message"
  }
}

