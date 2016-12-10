void setup() {
  DDRB |= (1<<4) | (1<<3); // PB4 = digital pin 12, PB3 = digital pin 11
  PORTB = 0;
  Serial.begin(1000000);
  ADMUX = (3<<REFS0) | (1<<ADLAR) | (0<<MUX0); // left aligned: 8-bit result on ADCH
  ADCSRB = (0<<ADTS0); // free running mode
  DIDR0 = (1<<ADC0D);
  ADCSRA = (1<<ADIE) | (1<<ADATE) | (5<<ADPS0); // interrupt, auto trigger, prescaler 32

  sei();
  // enable ADC and start conversion last
  ADCSRA |= (1<<ADEN);  
  ADCSRA |= (1<<ADSC);
}

void loop() {
  //PORTB ^= 1<<3; // PB3
}

ISR(ADC_vect) {
  PORTB ^= 1<<4; // PB4 = digital pin 12
#if 0
  uint8_t adl, adh;
  adl = ADCL;
  adh = ADCH; // ADCH must be read after ADCL
  UDR0 = adl;
  while(! (UCSR0A & UDRE0) ); // wait for empty USART data register
  UDR0 = adh;
#else
  ADCL;
  UDR0 = ADCH;
#endif
}

