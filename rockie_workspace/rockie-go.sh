#!/bin/sh

# This script is executed on Rockie power-on, and will call the roslanch
# file required to start rockie.
#
# RPI Rock Raiders
# 2/19/15
#
# Last Updated: Bryant Pong - 2/26/15 - 5:00 PM

while :
do
	if ps -ef | grep "[r]oslaunch" > /dev/null
		then
			echo "Roslaunch has not died!"
		else
			echo "Roslaunch has died! Attempting to reboot Rockie."
			# roslaunch rockie launch
	fi
done    
