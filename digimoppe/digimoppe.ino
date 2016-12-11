void setup() {
  DDRB |= (1<<4) | (1<<3); // PB4 = digital pin 12, PB3 = digital pin 11
  PORTB = 0;
  Serial.begin(1000000);
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

void loop() {
  //PORTB ^= 1<<3; // PB3
}

ISR(ADC_vect) {
  PORTB ^= 1<<4; // PB4 = digital pin 12

  uint16_t ad;
  uint8_t a;
  ad = ADCW;
  // clip to 8 bits:
  if(ad <= 0x180) a = 0;
  else if(ad >= 0x280) a = 0xFF;
  else a = ad - 0x180;
  UDR0 = a;
}

