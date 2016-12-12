#!/bin/bash
export DEV=/dev/ttyUSB0
export SPEED=1000000
TXFIFO=/tmp/txsamples
RXFIFO=/tmp/rxsamples
stty -F $DEV $SPEED raw
mkfifo $TXFIFO
mkfifo $RXFIFO
# For some reason the stty must be run again after opening the serial port.
# Here's a workaround until I learn how the serial port should actually be used.
./receivesamples $DEV $SPEED < $TXFIFO > $RXFIFO
