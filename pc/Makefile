%: %.c
	gcc $^ -o $@ -Wall -Wextra -O3

all: receivesamples uarttest txtestsignal

receivesamples: receivesamples.c

uarttest: uarttest.c

txtestsignal: txtestsignal.c

clean:
	rm receivesamples uarttest txtestsignal

.PHONY: all clean
