# Description #

This projects aims to provide a reference implemention for OpenDroneID in
C language. It contains:

## sender ##

The sender is a sample program to be used on a drone. It emits
OpenDroneID WiFi messages in regular intervals. The location and movement
information is taken from a GPS device which is connected using gpsd.

## scanner ##

The wifi drone scanner receives OpenDrone ID WiFi messages, parses them and
writes a list of seen Drones on the command line.

# Author #

This software has been written by Simon Wunderlich <sw@simonwunderlich.de>
and Marek Sobe <ms@simonwunderlich.de>.
It was sponsored by Doodle Labs ( https://doodlelabs.com )

For any questions, please contact:
	Simon Wunderlich <sw@simonwunderlich.de>

# TODO #

 * wifi drone scanner
