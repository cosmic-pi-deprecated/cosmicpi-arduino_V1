# cosmicpi-arduino

Files:

ArduinoPiInterface.txt 

	doccuments the various out put strings. 
	The default output format is CSV, here there is a reduced output to be used by the main client/server
	If the command "JSON 1" is issued to the serial input the output switches to json, this is for debugging
	there is much more data available in this mode for hardware testing, adjusting variables and monitoring

LPS.cpp and LPS.h

	This is a modified version of the LM25 library modified to address the LM25 on the main board on bus ONE

bmp085.cpp

	This handles the adafruit BMP085 barrometer breakout. Its not used in the standard hardware configuration,
	however ot is possible to plug this breakout on the main board to replace the LM25

cosmicpi-arduino.cc

	This is the main firmware. The firmware runs in the CSV simple mode by default. However it can also run in
	the JSON mode that accepts commands to change settings, outputs debug and monitoring information. This mode
	is chosen if the test program "pi-client.py" is run. This permits advanced interaction with the firmware for
	people who want to investigate whats going on under the hood.

cosmicpi-arduino.ino

	This is just a symbolic link to cosmicpi-arduino.cc for the Arduino IDE. If you point the Arduino IDE at
	this link you can compile and upload the firmware accross USB

pi-client.py

	This is an interactive test program for interacting with the JSON mode of the firmware. It talks to the 
	cosmicpi accross a USB port. To see the options type "./pi-client --help". To launch it you must specify
	which USB port to use, typically in Ubuntu linux it would be /dev/ttyACM0 or /dev/ttyACM1. To determin
	the port issue the command "ls /dev/tty*" before and after plugging in the cosmicpi to the USB. The test
	program requires write access to the /dev node, and depending on your system configuration you may need 
	to run with sudo privaledges. Once the test program is running type the ">" character to get a command
	prompt, the type "h" for help. 

	example:

		sudo ./pi-client -u /dev/ttyACM0 

		sudo ./pi-client -u /dev/ttyACM0 -i 31.194.73.171 -p 15443-o .

pi-server.py

	The client pi-client.py can be configured to send events and other information such as siesmic events, 
	weather station events, changes in magnetic field, and cosmic ray events to the server through UDP
	TCP/IP datagrams to the server. This can be useful for test involving multiple cosmicpi devices. These
	devices may be local or at remote locations. The server is able to corrolate and log data for futher
	analysis. See example above for specifying an ip address and port.

	example:

		./pi-client.py -p 15443


