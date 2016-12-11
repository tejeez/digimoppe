#include <stdint.h>
#include <unistd.h>
#include <stdio.h>

#define BUFSIZE 256
int main() {
	ssize_t r, i, bytecount=0;
	uint8_t buf[BUFSIZE];
	uint8_t pb=0;
	for(;;) {
		r = read(0, buf, BUFSIZE);
		if(r <= 0) break;
		for(i = 0; i < r; i++) {
			uint8_t b, l;
			b = buf[i];
			l = b - (pb+1);
			if(l) {
				fprintf(stderr, "Lost >=%d bytes at %zd\n", l, bytecount+i);
			}
			pb = b;
		}
		bytecount += r;
	}
	return 0;
}

