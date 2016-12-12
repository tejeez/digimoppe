/*
Protocol in the UART:
First byte contains the lowest 5 bits of the 10-bit ADC result.
Highest bit is always 1.
Second byte contains the highest 5 bits of the ADC result.
Highest bit is always 0.
Second highest bit is 1 is modulator buffer is half full.
*/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>

#define BUFSIZE 256
int main() {
	ssize_t wr, r, i, bytecount=0;
	uint8_t buf[BUFSIZE];
	uint8_t b0 = 0, b1 = 0;
	int prev = 2;
	uint16_t adczero = 0x200;
	for(;;) {
		uint8_t b;
		r = read(0, buf, BUFSIZE);
		if(r <= 0) break;
		for(i = 0; i < r; i++) {
			b = buf[i];
			if(b & 0x80) {
				b0 = b;
				if(prev == 0) {
					fprintf(stderr, "Lost byte at %zd\n", bytecount+i);
					wr = write(1, &adczero, sizeof(uint16_t));
					if(wr <= 0) break;
				}
				prev = 0;
			} else {
				uint16_t adcword;
				b1 = b;
				adcword = ((0x1F & b1) << 5) | (0x1F & b0);
				wr = write(1, &adcword, sizeof(uint16_t));
				if(wr <= 0) break;

				if(prev == 1) {
					fprintf(stderr, "Lost byte at %zd\n", bytecount+i);
				}
				prev = 1;
			}
			if(b & 0x40) {
				fprintf(stderr, "f");
			}
		}
		bytecount += r;
	}
	return 0;
}

