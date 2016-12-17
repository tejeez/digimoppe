TARGETS=receivesamples uarttest txtestsignal adf4351pc

%: %.c
	gcc $^ -o $@ -Wall -Wextra -O3


all: $(TARGETS)

receivesamples: receivesamples.c

uarttest: uarttest.c

txtestsignal: txtestsignal.c

adf4351pc: adf4351pc.c

clean:
	rm $(TARGETS)

.PHONY: all clean
