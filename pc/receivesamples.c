/*
Protocol in the UART:
First byte contains the lowest 5 bits of the 10-bit ADC result.
Highest bit is always 1.
Second byte contains the highest 5 bits of the ADC result.
Highest bit is always 0.
Second highest bit is 1 if modulator buffer is half full.
*/
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <asm/termbits.h>
#include <assert.h>

#define RXBUF 64
#define TXBUF 2
#define OUTBUF RXBUF
int main(int argc, char *argv[]) {
	ssize_t wr, r, rxbytes, i, bytecount=0;
	uint8_t rxbuf[RXBUF], txbuf[TXBUF];
	uint16_t outbuf[OUTBUF];
	uint8_t b0 = 0, b1 = 0;
	int prev = 2;
	int flags, serialspeed;
	uint16_t adczero = 0x200;
	int serialport, inpipe=0, outpipe=1;
	struct termios2 term;

	if(argc < 3) return 1;
	serialspeed = atoi(argv[2]);
	serialport = open(argv[1], O_RDWR);
	if(serialport < 0) return 2;

	ioctl(serialport, TCGETS2, &term);
	term.c_cflag &= ~CBAUD;
	term.c_cflag |= BOTHER;
	term.c_ispeed = serialspeed;
	term.c_ospeed = serialspeed;
	r = ioctl(serialport, TCSETS2, &term);
	if(r < 0) perror("TCSETS2");

	// make input and output pipes nonblocking
	flags = fcntl(inpipe, F_GETFL, 0);
	fcntl(inpipe, F_SETFL, flags | O_NONBLOCK);
	flags = fcntl(outpipe, F_GETFL, 0);
	fcntl(outpipe, F_SETFL, flags | O_NONBLOCK);

	for(;;) {
		uint8_t b;
		int txready = 1;
		size_t outp = 0;
		rxbytes = read(serialport, rxbuf, RXBUF);
		if(rxbytes <= 0) {
			perror("read from serial port");
			goto err;
		}
		for(i = 0; i < rxbytes; i++) {
			b = rxbuf[i];
			if(b & 0x80) {
				b0 = b;
				if(prev == 0) {
					fprintf(stderr, "Lost second byte at %zd\n", bytecount+i);
					outbuf[outp++] = adczero;
				}
				prev = 0;
			} else {
				uint16_t adcword;
				b1 = b;
				adcword = ((0x1F & b1) << 5) | (0x1F & b0);
				outbuf[outp++] = adcword;

				if(prev == 1) {
					fprintf(stderr, "Lost  first byte at %zd\n", bytecount+i);
				}
				prev = 1;
			}
			if(b & 0x40) txready = 0; // buffer getting full
		}

		assert(outp <= OUTBUF);
		wr = write(outpipe, &outbuf, outp*sizeof(int16_t));
		if(wr <= 0) {
			perror("write to RX pipe");
			goto err;
		} else if((size_t)wr < outp*sizeof(int16_t)) {
			fprintf(stderr, "Wrote only %zd bytes to output pipe\n", wr);
		}

		if(txready) {
			r = read(inpipe, txbuf, TXBUF);
			if(r > 0) {
				wr = write(serialport, txbuf, r);
				if(wr <= 0) {
					perror("write to serial port");
					goto err;
				}
			}
		}
		bytecount += rxbytes;
	}
	err:
	return 0;
}

