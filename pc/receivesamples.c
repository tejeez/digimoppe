/*
Protocol in the UART:
First byte contains the lowest 7 bits of the 10-bit ADC result.
Highest bit is always 1.
Second byte contains the highest 3 bits of the ADC result.
Highest bit is always 0. Other bits will be used for flow control and framing.
*/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>

#define BUFSIZE 256
int main() {
	ssize_t r, i, bytecount=0;
	uint8_t buf[BUFSIZE];
	uint8_t b0 = 0, b1 = 0;
	int prev = 2;
	for(;;) {
		r = read(0, buf, BUFSIZE);
		if(r <= 0) break;
		for(i = 0; i < r; i++) {
			uint8_t b = buf[i];
			if(b & 0x80) {
				b0 = b;
				if(prev == 0) {
					fprintf(stderr, "Lost byte at %zd\n", bytecount+i);
				}
				prev = 0;
			} else {
				int wr;
				uint16_t adcword;
				b1 = b;
				adcword = ((0x07 & b1) << 7) | (0x7F & b0);
				wr = write(1, &adcword, sizeof(uint16_t));
				if(wr <= 0) break;

				if(prev == 1) {
					fprintf(stderr, "Lost byte at %zd\n", bytecount+i);
				}
				prev = 1;
			}
		}
		bytecount += r;
	}
	return 0;
}

