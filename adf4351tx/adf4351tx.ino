/*
Pinout from Arduino Nano to ADF4351 board:
D2:  LE
D11: DATA
D13: CLK
Put voltage dividers in between for level conversion.
Connect CE on PLL board to its 3.3 V regulator.
*/

#include "SPI.h"


/* mbuf is a buffer for the commands to send to ADF4351.
   The size is 0x100 so we can conveniently use wrap-around of unsigned 8-bit integers
   when dealing with indexes to the buffer.
   mwp is the index where buffer is being written,
   mrp is the index where buffer is being read.
   4-byte commands are aligned to 4 bytes. mwp and mrp are always multiples of 4
   and are always incremented by 4 at once.
   Buffer is empty when mwp == mrp.
   MBUFFILL sets how many bytes to try to keep in the buffer.
   We ask PC for more data when there's less bytes waiting. */
uint8_t mbuf[0x100];
volatile uint8_t mwp = 0, mrp = 0;
#define MBUFFILL 100

volatile uint8_t /*mhalffull=0, */txflag=0;

#define LE_PORT PORTD
#define LE_PORT_DDR DDRD
#define LE_BIT 2

#define DEBUG_PORT PORTD
#define DEBUG_PORT_DDR DDRD
#define DEBUG_OVERFLOW_BIT 3
#define DEBUG_FREQ_BIT 4

void setup() {
  SPI.begin();
  SPI.setClockDivider(16);
  SPI.setDataMode(SPI_MODE0);
  LE_PORT_DDR |= 1<<LE_BIT;
  DEBUG_PORT_DDR |= (1 << DEBUG_FREQ_BIT) | (1<<DEBUG_OVERFLOW_BIT);

  //Serial.begin(1000000);
  // Serial.begin can't be used if we have own USART interrupt handler
  UCSR0A = 0; // don't double speed
  UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1<<TXEN0); // enable UART receive interrupt
  UCSR0C = (0<<UMSEL00) | (0<<UPM00) | (0<<USBS0) | (3<<UCSZ00); // 8N1
  UBRR0 = 0; // 1 Mbaud

  /* Timer1 prescaler is 8, so counting to 56 results in
     a sample rate of 16 MHz / 8 / 56 = 35714 Hz. */
  TCCR1A = 0; // WGM10=0, WGM11=0
  TCCR1B = 1<<WGM12; // WGM12=1: clear timer on compare match A
  OCR1A = 56 -1;
  OCR1B = 0;
  TIMSK1 = 1<<OCIE1B; // interrupt on match B
  TCNT0 = 0;

  sei();
  TCCR1B |= (2<<CS10); // start timer1: prescale by 8
}


void loop() {
  uint8_t rp, n;
  uint8_t mhalffull;
  if(txflag) {
    txflag = 0;
    rp = mrp;
    rp &= 0xFC; // ensure it's aligned to 4 (it should be anyway)

    n = mwp - rp; // number of bytes in buffer
    if(n) {
      // transmit command to ADF4351
      LE_PORT &= ~(1<<LE_BIT);
      SPI.transfer(mbuf[rp+0]);
      SPI.transfer(mbuf[rp+1]);
      SPI.transfer(mbuf[rp+2]);
      SPI.transfer(mbuf[rp+3]);
      LE_PORT |= 1<<LE_BIT;

      mrp = rp+4;
      // could turn transmitter on here
    } else {
      // could turn transmitter off here
    }
    mhalffull = (n >= MBUFFILL);
    if(mhalffull) UDR0 = 19; // xoff
    else UDR0 = 17; // xon
  }
}


uint16_t symbolcounter = 0;
ISR(TIMER1_COMPB_vect) {
  DEBUG_PORT ^= 1<<DEBUG_FREQ_BIT; // PB4 = digital pin 12


  /* This interrupt occurs (16e6 / 8 / 56) times per second.
     We want to send a symbol 1200 times per second on average.
     The ratio of these values is 0.0336, or 21/625. */
  symbolcounter += 21;
  if(symbolcounter >= 625) {
    symbolcounter -= 625;
    txflag = 1; // send the command in main loop
  }
}


uint8_t nb = 4;
uint8_t commandrxbuf[4]; // buffer for command currently being received from UART

ISR(USART_RX_vect) {
  uint8_t b, nextwp;
  b = UDR0;
  if(nb <= 3) {
    // receiving one 4-byte command
    commandrxbuf[nb] = b;
    nb++;
    if(nb == 4) {
      // command ready: put it in circular buffer
      nextwp = mwp+4;
      nextwp &= 0xFC; // ensure it's aligned to 4 (it should be anyway)
      if(nextwp == mrp) {
        // full buffer!
        DEBUG_PORT |= 1<<DEBUG_OVERFLOW_BIT;
        UDR0 = 'O';
      } else {
        mbuf[nextwp+0] = commandrxbuf[0];
        mbuf[nextwp+1] = commandrxbuf[1];
        mbuf[nextwp+2] = commandrxbuf[2];
        mbuf[nextwp+3] = commandrxbuf[3];
        mwp = nextwp;
        DEBUG_PORT &= ~(1<<DEBUG_OVERFLOW_BIT);
      }
    }
  } else if(b == 'R') {
    // start receiving 4-byte command in next byte
    nb = 0;
  }
}

