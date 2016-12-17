#!/bin/bash
export DEV=/dev/ttyUSB0
export SPEED=115200
TXFIFO=/tmp/txsamples
stty -F $DEV $SPEED raw -echo
mkfifo $TXFIFO
./adf4351pc $DEV $SPEED < $TXFIFO
