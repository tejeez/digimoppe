#!/bin/bash
export DEV=/dev/ttyUSB0
export PARAM=2000000 raw
stty -F $DEV $PARAM
# For some reason the stty must be run again after opening the serial port.
# Here's a workaround until I learn how the serial port should actually be used.
(sleep 1; stty -F $DEV $PARAM) &
./uarttest < $DEV
