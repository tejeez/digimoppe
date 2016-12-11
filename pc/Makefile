%: %.c
	gcc $^ -o $@ -Wall -Wextra -O3

all: receivesamples uarttest

receivesamples: receivesamples.c

uarttest: uarttest.c

clean:
	rm receivesamples uarttest

.PHONY: all clean
