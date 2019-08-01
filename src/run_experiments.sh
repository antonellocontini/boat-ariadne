#!/bin/bash

# assuming this script is launched in the path containing the executable
EXEC_PATH="$(pwd)"

MILLIS_FILE="$EXEC_PATH/../experiments/wind/time_millis.txt"

# wind experiment
#rm -f $MILLIS_FILE

#echo "Run one wind streak experiment"
#T="$(date +%s%N | cut -b1-13)"
#cd ../experiments/wind/one_strike && $EXEC_PATH/boat
#let "T = $(date +%s%N | cut -b1-13) - $T"
#echo "wind strikes: 1 elapsed time: $T" >> $MILLIS_FILE

echo "Run two wind streaks experiment"
T="$(date +%s%N | cut -b1-13)"
cd ../experiments/wind/two_strikes && $EXEC_PATH/boat
let "T = $(date +%s%N | cut -b1-13) - $T"
echo "wind strikes: 2 elapsed time: $T" >> $MILLIS_FILE

echo "Press enter key to continue to proportional gain experiments..."; read

MILLIS_FILE="$EXEC_PATH/../experiments/prop_gain/time_millis.txt"

# proportional gain constant experiments
rm -f $MILLIS_FILE
echo "Run K=0.1 experiment"
T="$(date +%s%N | cut -b1-13)"

cd ../experiments/prop_gain/k01 && $EXEC_PATH/boat
let "T = $(date +%s%N | cut -b1-13) - $T"
echo "k01: $T" >> $MILLIS_FILE


echo "Run K=0.2 experiment"
T="$(date +%s%N | cut -b1-13)"

cd ../k02 && $EXEC_PATH/boat
let "T = $(date +%s%N | cut -b1-13) - $T"
echo "k02: $T" >> $MILLIS_FILE


echo "Run K=0.5 experiment"
T="$(date +%s%N | cut -b1-13)"

cd ../k05 && $EXEC_PATH/boat
let "T = $(date +%s%N | cut -b1-13) - $T"
echo "k05: $T" >> $MILLIS_FILE


echo "Run K=1.0 experiment"
T="$(date +%s%N | cut -b1-13)"

cd ../k10 && $EXEC_PATH/boat
let "T = $(date +%s%N | cut -b1-13) - $T"
echo "k10: $T" >> $MILLIS_FILE


echo "Run K=2.0 experiment"
T="$(date +%s%N | cut -b1-13)"

cd ../k20 && $EXEC_PATH/boat
let "T = $(date +%s%N | cut -b1-13) - $T"
echo "k20: $T" >> $MILLIS_FILE

cd $EXEC_PATH