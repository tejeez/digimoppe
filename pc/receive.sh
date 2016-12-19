#!/bin/bash
CODEC2_PATH=~/freedv/code/codec2-dev/build/src
DEV=/dev/ttyUSB0
SPEED=1000000
TXFIFO=/tmp/txsamples
RESAMPFIFO=/tmp/resampled

stty -F $DEV $SPEED raw -echo

[ -e $TXFIFO ] || mkfifo $TXFIFO
[ -e $RESAMPFIFO ] || mkfifo $RESAMPFIFO

# writing to stdout using gnuradio file sink didn't work the whole pipeline for some reason
#./receivesamples $DEV $SPEED < $TXFIFO | ../grc/top_block.py | $CODEC2_PATH/freedv_rx 2400A - - | play -t raw -r 8000 -e signed-integer -b 16 -

# using named pipe $RESAMPFIFO in between works better
./receivesamples $DEV $SPEED < $TXFIFO | ../grc/resample/top_block.py & PID=$!
$CODEC2_PATH/freedv_rx 2400A $RESAMPFIFO - | play -t raw -r 8000 -e signed-integer -b 16 -
wait $PID
