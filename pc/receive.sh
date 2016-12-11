#!/bin/bash
DEV=/dev/ttyUSB0
PARAM=2000000 raw
FIFO=/tmp/rxsamples
stty -F $DEV $PARAM
mkfifo $FIFO
# For some reason the stty must be run again after opening the serial port.
# Here's a workaround until I learn how the serial port should actually be used.
(sleep 2; stty -F $DEV $PARAM) &
./receivesamples < $DEV > $FIFO
