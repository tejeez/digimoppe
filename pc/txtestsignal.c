#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
int main() {
	unsigned int t = 0;
	uint8_t s = 0;
	for(t=0;;t++) {
		if(t & 0x1000)
			s = 3 & t >> 9;
		else
			s = 3 & rand();
		putchar(s);
	}
}
