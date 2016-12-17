#!/bin/bash
export DEV=/dev/ttyUSB0
export SPEED=1000000
TXFIFO=/tmp/txsamples
stty -F $DEV $SPEED raw -echo ixon 
mkfifo $TXFIFO
./adf4351pc $DEV $SPEED < $TXFIFO
