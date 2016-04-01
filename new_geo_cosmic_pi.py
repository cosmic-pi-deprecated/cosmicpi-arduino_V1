#!	/usr/bin/python
#	coding: utf8

"""
Talk to the CosmicPi Arduino DUE accross the serial USB link
This program has the following functions ...

1) Build event messages and send them to a server or local port

	Events are any combination of Vibration, Weather and CosmicRays
	Hence the Arduino can behave as a weather station, as a vibration/Siesmic monitor
	and as a cosmic ray detector. 
	There is a gyroscope available but I don't use it

2) Perform diagnostics and monitoring of the Arduino via commands

3) Log events to the log file

Typing the '>' character turns on command input

It is important to keep the Python dictionary objects synchronised with the Arduino firmware
otherwise this monitor will not understand the data being sent to it

julian.lewis lewis.julian@gmail.com 29/Mar/2016

"""

import sys
import socket
import select
import serial
import time
import traceback
import os
import termios
import fcntl
import re
import ast
from optparse import OptionParser

# Handle keyboard input, this tests to see if a '>' was typed

class KeyBoard(object):

	def __init__(self):
		self.fd = sys.stdin.fileno()

	def echo_off(self):
		self.oldterm = termios.tcgetattr(self.fd)
		self.newattr = termios.tcgetattr(self.fd)
		self.newattr[3] = self.newattr[3] & ~termios.ICANON & ~termios.ECHO
		termios.tcsetattr(self.fd, termios.TCSANOW, self.newattr)
		self.oldflags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
		fcntl.fcntl(self.fd, fcntl.F_SETFL, self.oldflags | os.O_NONBLOCK)

	def echo_on(self):
		termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.oldterm)
		fcntl.fcntl(self.fd, fcntl.F_SETFL, self.oldflags)

	def test_input(self):
		res = False
		try:
			c = sys.stdin.read(1)
			if c == '>':
				res = True
		except IOError: pass
		return res

# This is the event object, it builds a dictionary from incomming jsom strings 
# and provides access to the dictionary entries containing the data for each field.

class Event(object):

	def __init__(self):

		# These are the json strings we are expecting from the arduino

		self.HTU = { "Tmh":"f","Hum":"f" }
		self.BMP = { "Tmb":"f","Prs":"f","Alb":"f" }
		self.VIB = { "Vax":"i","Vcn":"i" }
		self.MAG = { "Mgx":"f","Mgy":"f","Mgz":"f" }
		self.MOG = { "Mox":"f","Moy":"f","Moz":"f" }
		self.ACL = { "Acx":"f","Acy":"f","Acz":"f" }
		self.AOL = { "Aox":"f","Aoy":"f","Aoz":"f" }
		self.LOC = { "Lat":"f","Lon":"f","Alt":"f" }
		self.TIM = { "Upt":"i","Frq":"i","Sec":"i" }
		self.STS = { "Qsz":"i","Mis":"i","Ter":"i","Htu":"i","Bmp":"i","Acl":"i","Mag":"i" }
		self.EVT = { "Evt":"i","Frq":"i","Tks":"i","Etm":"f","Adc":"[[i,i,i,i,i,i,i,i],[i,i,i,i,i,i,i,i]]" }
		self.DAT = { "Dat":"s" }

		# Now build the main dictionary with one entry for each json string we will process

		self.recd = {	"HTU":self.HTU, "BMP":self.BMP, "VIB":self.VIB, "MAG":self.MAG, "MOG":self.MOG,
				"ACL":self.ACL, "AOL":self.AOL, "LOC":self.LOC, "TIM":self.TIM, "STS":self.STS,
				"EVT":self.EVT, "DAT":self.DAT }

		self.oetm = "f"
		self.ovib = "i"

	# Convert the incomming json strings into entries in the dictionary 

	def parse(self, line):					# parse the incomming json strings from arduino
		nstr = line.replace('\n','')			# Throw away <crtn>, we dont want them
		try:
			dic = ast.literal_eval(nstr)		# Build a dictionary entry
			kys = dic.keys()			# Get key names, the first is the address
			if self.recd.has_key(kys[0]):		# Check we know about records with this key
				self.recd[kys[0]] = dic[kys[0]]	# and put it in the dictionary at that address
			
		except Exception, e:
			pass					# Didnt understand, throw it away

	def extract(self, entry):
		if self.recd.has_key(entry):
			if entry is "DAT":
				self.recd["DAT"] = time.asctime(time.gmtime(time.time()))
			nstr = "{\'%s\':%s}" % (entry,str(self.recd[entry]))
			return nstr
		else:
			return ""

	# build weather, cosmic ray and vibration event strings suitable to be sent over the network to server
	# these strings are self describing json format for easy decoding at the server end

	def get_weather(self):

		# Build a list of dictionary entries as a string

		self.weather =		self.extract("HTU") + \
				"*" +	self.extract("BMP") + \
				"*" +	self.extract("LOC") + \
				"*" +	self.extract("TIM") + \
				"*" +	self.extract("DAT")
 
		return self.weather

	def get_event(self):
		if self.oetm == self.recd["EVT"]["Etm"]:
			return ""

		self.oetm = self.recd["EVT"]["Etm"]

		self.evt =		self.extract("EVT") + \
				"*" +	self.extract("BMP") + \
				"*" +	self.extract("ACL") + \
				"*" +	self.extract("MAG") + \
				"*" +	self.extract("HTU") + \
				"*" +	self.extract("STS") + \
				"*" +	self.extract("LOC") + \
				"*" +	self.extract("TIM") + \
				"*" +	self.extract("DAT")
		return self.evt

	def get_vibration(self):
		if self.ovib == self.recd["VIB"]["Vcn"]:
			return ""

		self.ovib = self.recd["VIB"]["Vcn"]

		self.vib =		self.extract("VIB") + \
				"*" +	self.extract("ACL") + \
				"*" +	self.extract("MAG") + \
				"*" +	self.extract("LOC") + \
				"*" +	self.extract("TIM") + \
				"*" +	self.extract("DAT")
		return self.vib

	# Here we just return dictionaries

	def get_vib(self):
		return self.recd["VIB"]

	def get_tim(self):
		return self.recd["TIM"]

	def get_loc(self):
		return self.recd["LOC"]

	def get_sts(self):
		return self.recd["STS"]

	def get_bmp(self):
		return self.recd["BMP"]

	def get_acl(self):
		return self.recd["ACL"]

	def get_mag(self):
		return self.recd["MAG"]

	def get_htu(self):
		return self.recd["HTU"]

# Send UDP packets to the remote server

class Socket_io(object):

	def __init__(self,ipaddr,ipport):
		try:
			self.sok = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		except Exception, e:
			msg = "Exception: Can't open Socket: %s" % (e)
			print "Sending OFF:%s" % msg
			udpflg = False	

	def send_event_pkt(self,pkt,ipaddr,ipport):
		try:
			self.sok.sendto(pkt, (ipaddr, ipport))

		except Exception, e:
			msg = "Exception: Can't sendto: %s" % (e)
			print "Sending OFF:%s" % msg
			udpflg = False


	def close(self):
		self.sok.close()		

def main():
	use = "Usage: %prog [--ip=cosmicpi.ddns.net --port=4901 --usb=/dev/ttyACM0 --debug --dirnam=/tmp]"
	parser = OptionParser(usage=use, version="cosmic_pi version 1.0")
	
	parser.add_option("-i", "--ip",    help="Server IP address or name", dest="ipaddr", default="localhost")
	parser.add_option("-p", "--port",  help="Server portnumber", dest="ipport", type="int", default="4901")
	parser.add_option("-u", "--usb",   help="USB device name", dest="usbdev", default="/dev/ttyACM0")
	parser.add_option("-d", "--debug", help="Debug Option", dest="debug", default=False, action="store_true")
	parser.add_option("-o", "--odir",  help="Path to log directory", dest="logdir", default="/tmp")
	parser.add_option("-n", "--noip",  help="IP Sending", dest="udpflg", default=True, action="store_false")
	parser.add_option("-l", "--log",   help="Event Logging", dest="logflg", default=False, action="store_true")
	parser.add_option("-v", "--vib",   help="Vibration monitor", dest="vibflg", default=False, action="store_true")
	parser.add_option("-w", "--ws",    help="Weather station", dest="wstflg", default=False, action="store_true")

	options, args = parser.parse_args()

	ipaddr = options.ipaddr
	ipport = options.ipport
	usbdev = options.usbdev
	logdir = options.logdir
	debug  = options.debug
	udpflg = options.udpflg
	logflg = options.logflg
	vibflg = options.vibflg
	wstflg = options.wstflg

	print "\n\ncosmic_pi monitor running, hit '>' for commands\n\n"

	if debug:
		print "\n"
		print "options (Server IP address)	ip:  %s" % ipaddr
		print "options (Server Port number)	port:%d" % ipport
		print "options (USB device name)	usb: %s" % usbdev
		print "options (Logging directory)	odir:%s" % logdir
		print "options (Event logging)          log: %s" % logflg
		print "options (UDP sending)            udp: %s" % udpflg
		print "options (Vibration monitor)      vib: %s" % vibflg
		print "options (Weather Station)        wst: %s" % wstflg
	

	now = time.asctime(time.localtime(time.time()))
	now = now.replace(" ","-")
	now = now.replace(":","-")

	lgf = "%s/cosmicpi-logs/%s.log" % (logdir,now)
	dir = os.path.dirname(lgf)
	if not os.path.exists(dir):
		os.makedirs(dir)
	try:
		log = open(lgf, "w");
	except Exception, e:
		msg = "Exception: Cant open log file: %s" % (e)
		print "Fatal: %s" % msg
		sys.exit(1)

	if options.debug:
		print "\n"
		print "Log file is: %s" % lgf

  	try:
		ser = serial.Serial(port=usbdev, baudrate=9600, timeout=60)
		ser.flush()
	except Exception, e:
		msg = "Exception: Cant open USB device: %s" % (e)
		print "Fatal: %s" % msg
		sys.exit(1)
	
	if debug:
		print "\n"
		print "USB device is: %s" % usbdev

	kbrd = KeyBoard()
	kbrd.echo_off()

	evt = Event()
	events = 0
	vbrts = 0
	dweather = 0

	sio = Socket_io(ipaddr,ipport)

	try:
		while(True):
			if kbrd.test_input():
				kbrd.echo_on()
				print "\n"
				cmd = raw_input(">")
        
				if cmd.find("q") != -1:
       					break
 
				elif cmd.find("d") != -1:
         				if debug:
						debug = False
					else:
						debug = True
					print "Debug:%s\n" % debug

				elif cmd.find("v") != -1:
         				if vibflg:
						vibflg = False
					else:
						vibflg = True
					print "Vibration:%s\n" % vibflg

				elif cmd.find("w") != -1:
         				if wstflg:
						wstflg = False
					else:
						wstflg = True
					print "WeatherStation:%s\n" % wstflg

				elif cmd.find("s") != -1:
					tim = evt.get_tim()
					sts = evt.get_sts()
					loc = evt.get_loc()
					acl = evt.get_acl()
					mag = evt.get_mag()
					bmp = evt.get_bmp()
					htu = evt.get_htu()
					vib = evt.get_vib()

					print "ARDUINO STATUS"
					print "Status........: Upt:%s Frq:%s Qsz:%s Mis:%s" % (tim["Upt"],tim["Frq"],sts["Qsz"],sts["Mis"])
					print "HardwareStatus: Htu:%s Bmp:%s Acl:%s Mag:%s" % (sts["Htu"],sts["Bmp"],sts["Acl"],sts["Mag"])
					print "Location......: Lat:%s Lon:%s Alt:%s" % (loc["Lat"],loc["Lon"],loc["Alt"])
					print "Accelarometer.: Acx:%s Acy:%s Acz:%s" % (acl["Acx"],acl["Acy"],acl["Acz"])
					print "Magnatometer..: Mgx:%s Mgy:%s Mgz:%s" % (mag["Mgx"],mag["Mgy"],mag["Mgz"])
					print "Barometer.....: Tmb:%s Prs:%s Alb:%s" % (bmp["Tmb"],bmp["Prs"],bmp["Alb"])
					print "Humidity......: Tmh:%s Hum:%s" % (htu["Tmh"],htu["Hum"])
					print "Vibration.....: Vax:%s Vcn:%s\n" % (vib["Vax"],vib["Vcn"])

					print "MONITOR STATUS"
					print "USB device....: %s" % (usbdev)
					print "Remote........: Ip:%s Port:%s UdpFlag:%s" % (ipaddr,ipport,udpflg)
					print "Vibration.....: Sent:%d Flag:%s" % (vbrts,vibflg)
					print "WeatherStation: Flag:%s" % (wstflg)
					print "Events........: Sent:%d LogFlag:%s" % (events,logflg)
					print "LogFile.......: %s\n" % (lgf)

				elif cmd.find("h") != -1:
					print "MONITOR COMMANDS"
					print "   q=quit, s=status, d=toggle_debug, n=toggle_send, l=toggle_log"
					print "   v=vibration, w=weather, h=help\n"
					print "ARDUINO COMMANDS"
					print "   NOOP, Do nothing"
					print "   HELP, Display commands"
					print "   HTUX, Reset the HTH chip"
					print "   HTUD, HTU Temperature-Humidity display rate, <rate>"
					print "   BMPD, BMP Temperature-Altitude display rate, <rate>"
					print "   LOCD, Location latitude-longitude display rate, <rate>"
					print "   TIMD, Timing uptime-frequency-etm display rate, <rate>"
					print "   STSD, Status info display rate, <rate>"
					print "   EVQT, Event queue dump threshold, <threshold 1..32>"
					print "   ACLD, Accelerometer display rate, <rate>"
					print "   MAGD, Magomagnatometer display rate, <rate>"
					print "   ACLT, Accelerometer event trigger threshold, <threshold 0..127>"
					print ""

					if debug:
						ser.write("HELP")

				elif cmd.find("n") != -1:
					if udpflg:
						udpflg = False
					else:
						udpflg = True
					print "Send:%s\n" % udpflg

				elif cmd.find("l") != -1:
					if logflg:
						logflg = False
					else:
						logflg = True
					print "Log:%s\n" % logflg

				else:
					print "Arduino < %s\n" % cmd 
					ser.write(cmd.upper())
			
				kbrd.echo_off()
				 
			rc = ser.readline()

			if len(rc) == 0:
				print "Serial input buffer empty"
				ser.close()
				time.sleep(1)
				ser = serial.Serial(port=usbdev, baudrate=9600, timeout=60)
				rc = ser.readline()
				if len(rc) == 0:
					break
				print "Serial Reopened OK"
				continue
			else:
				evt.parse(rc)

				if vibflg:
					vbuf = evt.get_vibration()
					if len(vbuf) > 0:
						vbrts = vbrts + 1
						vib = evt.get_vib()
						tim = evt.get_tim()
						acl = evt.get_acl()
						mag = evt.get_mag()
						print "Vibration.....: Cnt:%d Vax:%s Vcn:%s " % (vbrts,vib["Vax"],vib["Vcn"])
						print "Accelarometer.: Acx:%s Acy:%s Acz:%s" % (acl["Acx"],acl["Acy"],acl["Acz"])
						print "Magnatometer..: Mgx:%s Mgy:%s Mgz:%s" % (mag["Mgx"],mag["Mgy"],mag["Mgz"])
						print "Time..........: Sec:%s\n" % (tim["Sec"])
						
						if udpflg:
							sio.send_event_pkt(vbuf,ipaddr,ipport)
						if logflg:
							log.write(vbuf)


				if wstflg:
					dweather = dweather + 1
					if (dweather % 60) == 0:
						tim = evt.get_tim()
						bmp = evt.get_bmp()
						htu = evt.get_htu()
						loc = evt.get_loc()
						print "Barometer.....: Tmb:%s Prs:%s Alb:%s" % (bmp["Tmb"],bmp["Prs"],bmp["Alb"])
						print "Humidity......: Tmh:%s Hum:%s Alt:%s" % (htu["Tmh"],htu["Hum"],loc["Alt"])
						print "Time..........: Sec:%s\n" % (tim["Sec"])

				ebuf = evt.get_event()
				if len(ebuf) > 1:
					events = events + 1
					sys.stdout.write("Cosmic ray events flushed:%d - %s" % (events,time.asctime(time.gmtime(time.time()))))
					sys.stdout.write("                               \n")	# Clean text off screen !!

					if udpflg:
						sio.send_event_pkt(ebuf,ipaddr,ipport)
					if logflg:
						log.write(ebuf)
				else:
					if debug:
						sys.stdout.write(rc)
					else:
						ts = time.strftime("%d/%b/%Y %H:%M:%S",time.gmtime(time.time()))
						tim = evt.get_tim();
						sts = evt.get_sts();
						s = "cosmic_pi:Upt:%s :Qsz:%s Tim:[%s] %s    \r" % (tim["Upt"],sts["Qsz"],ts,tim["Sec"])
						sys.stdout.write(s)
						sys.stdout.flush()

	except Exception, e:
		msg = "Exception: main: %s" % (e)
		print "Fatal: %s" % msg

	finally:
		kbrd.echo_on()
		tim = evt.get_tim()
		print "\nUp time:%s Quitting ..." % tim["Upt"]
		ser.close()
		log.close()
		sio.close()
		time.sleep(1)
		sys.exit(0)
		
if __name__ == '__main__':

    main()
