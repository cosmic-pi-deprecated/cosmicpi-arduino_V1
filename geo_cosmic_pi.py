#!	/usr/bin/python
#	coding: utf8

"""
Talk to the CosmicPi Arduino DUE accross the serial USB link
This program has four main functions
1) Build event messages and send them to a server
2) Perform diagnostics and monitoring
3) Log events to the log file
4) Execute commnads on the Arduino 

Typing the '>' character turns on command input

julian.lewis lewis.julian@gmail.com 23/Feb/2016

Added support for magnatometer and accelerometer chips

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
from optparse import OptionParser

# Handle keyboard input

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

class Event(object):

	# Well it grew on me as I hacked away, this is not good python code
	# Its sreaming out for me to use asociative arrays, will do soon. 
	# Then the method would look like ... val = evt.get_field("<fieldname>")
	# and the class would add the entries on the fly as they come in from
	# the Arduino automatically syncing field names to what it recieves
	# Then a bit of introspection at caller level to see what's available	
	# something like (list) = evt.get_accelerometer_fields() ...
	# Next version sorry

	def __init__(self):
		self.evt = "0"
		self.upt = "0"
		self.frq = "0"
		self.tks = "0"
		self.etm = "0"
		self.lat = "0"
		self.lon = "0"
		self.tmb = "0"
		self.alt = "0"
		self.hum = "0"
		self.sec = "0"
		self.qsz = "0"
		self.mis = "0"
		self.prs = "0"
		self.bmp = "0"
		self.htu = "0"
		self.acl = "0"
		self.mag = "0"
		self.acx = "0"
		self.acy = "0"
		self.acz = "0"
		self.mgx = "0"
		self.mgy = "0"
		self.mgz = "0"
		self.vax = "0"
		self.vcn = "0"

	def parse(self, line):
		nstr = line.split(':')
		for i in range(0,len(nstr)):
			nstr[i] = nstr[i].replace('\n','')

		for i in range(0,len(nstr) -1):
			j = i + 1
			
			if nstr[i].find("Upt") != -1:
				self.upt = "%s:%s " % (nstr[i],nstr[j])
 
			elif nstr[i].find("Frq") != -1:
				self.frq = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Tks") != -1:
				self.tks = "%s:%s " % (nstr[i],nstr[j])
        
			elif nstr[i].find("Etm") != -1:
				ts = time.asctime(time.gmtime(time.time()))
				self.etm = "%s:[%s]%s " % (nstr[i],ts,nstr[j])
				return self.get()
        
			elif nstr[i].find("Lat") != -1:
				self.lat = "%s:%s " % (nstr[i], nstr[j])
        
			elif nstr[i].find("Lon") != -1:
				self.lon = "%s:%s " % (nstr[i],nstr[j])
        
			elif nstr[i].find("Tmb") != -1:
				self.tmb = "%s:%s " % (nstr[i],nstr[j])
			
			elif nstr[i].find("Alt") != -1:
				self.alt = "%s:%s " % (nstr[i],nstr[j])
        
			elif nstr[i].find("Hum") != -1:
				self.hum = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Sec") != -1:
				ts = time.asctime(time.gmtime(time.time()))
				self.sec = "%s:[%s]%s " % (nstr[i],ts,nstr[j])

			elif nstr[i].find("Qsz") != -1:
				self.qsz = nstr[j]

			elif nstr[i].find("Mis") != -1:
				self.mis = nstr[j]

			elif nstr[i].find("Prs") != -1:
				self.prs = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Bmp") != -1:
				self.bmp = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Htu") != -1:
				self.htu = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Acl") != -1:
				self.acl = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Mag") != -1:
				self.mag = "%s:%s " % (nstr[i],nstr[j])
			
			elif nstr[i].find("Acx") != -1:
				self.acx = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Acy") != -1:
				self.acy = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Acz") != -1:
				self.acz = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Mgx") != -1:
				self.mgx = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Mgy") != -1:
				self.mgy = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Mgz") != -1:
				self.mgz = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Vax") != -1:
				self.vax = "%s:%s " % (nstr[i],nstr[j])

			elif nstr[i].find("Vcn") != -1:
				self.vcn = "%s:%s " % (nstr[i],nstr[j])


		return ""

	def get(self):
		self.evt =	"{" + self.upt + \
				"," + self.frq + \
				"," + self.tks + \
				"," + self.get_loc() + \
				"," + self.prs + \
				"," + self.get_acl() + \
				"," + self.get_mag() + \
				"," + self.tmb + \
				"," + self.hum + \
				"," + self.get_hws() + \
				"," + self.etm + \
				"}\n"
		return self.evt

	def get_time(self):
		return self.sec

	def get_qsz(self):
		return self.qsz

	def get_upt(self):
		return self.upt

	def get_mis(self):
		return self.mis
	
	def get_frq(self):
		return self.frq

	def get_loc(self):
		return "%s %s %s" % (self.lat,self.lon,self.alt)

	def get_hws(self):
		return "%s %s %s %s" % (self.bmp,self.htu,self.acl,self.mag)

	def get_prs(self):
		return self.prs

	def get_acl(self):
		return "%s %s %s" % (self.acx,self.acy,self.acz)

	def get_mag(self):
		return "%s %s %s" % (self.mgx,self.mgy,self.mgz)

	def get_vib(self):
		return "%s %s" % (self.vax,self.vcn)

class Socket_io(object):

	def __init__(self,ipaddr,ipport):
		try:
			self.sok = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.sik = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.sik.setblocking(0)
			self.sik.bind((ipaddr,ipport))

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

	def recv_event_pkt(self):
		try:
			available = select.select([self.sik], [], [], 1)
			if available[0]:
				recv = self.sik.recvfrom(1024)
				return recv

		except Exception, e:
			msg = "Exception: Can't recvfrom: %s" % (e)
			print "Sending OFF:%s" % msg
			udpflg = False

		return False	# I love Python

	def close(self):
		self.sok.close()		

def main():
	use = "Usage: %prog [--ip=cosmicpi.ddns.net --port=4901 --usb=/dev/ttyACM1 --debug --dirnam=/tmp]"
	parser = OptionParser(usage=use, version="cosmic_pi version 1.0")
	
	parser.add_option("-i", "--ip",    help="Server IP address or name", dest="ipaddr", default="localhost")
	parser.add_option("-p", "--port",  help="Server portnumber", dest="ipport", type="int", default="4901")
	parser.add_option("-u", "--usb",   help="USB device name", dest="usbdev", default="/dev/ttyACM1")
	parser.add_option("-d", "--debug", help="Debug Option", dest="debug", default=False, action="store_true")
	parser.add_option("-o", "--odir",  help="Path to log directory", dest="logdir", default="/tmp")
	parser.add_option("-n", "--noip",  help="IP Sending", dest="udpflg", default=True, action="store_false")
	parser.add_option("-l", "--log",   help="Event Logging", dest="logflg", default=False, action="store_true")

	options, args = parser.parse_args()

	ipaddr = options.ipaddr
	ipport = options.ipport
	usbdev = options.usbdev
	logdir = options.logdir
	debug  = options.debug
	udpflg = options.udpflg
	logflg = options.logflg

	print "\n\ncosmic_pi monitor running, hit '>' for commands\n\n"

	if debug:
		print "\n"
		print "options (Server IP address)	ip:  %s" % ipaddr
		print "options (Server Port number)	port:%d" % ipport
		print "options (USB device name)	usb: %s" % usbdev
		print "options (Logging directory)	odir:%s" % logdir
		print "options (Event logging)          log: %s" % logflg
		print "options (UDP sending)            udp: %s" % udpflg
	

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

				elif cmd.find("s") != -1:
					print "Status: Upt:%s Frq:%s Qsz:%s Mis:%s" \
					% (evt.get_upt(),evt.get_frq(),evt.get_qsz(),evt.get_mis())
					print "Location: %s %s" % (evt.get_loc(),evt.get_prs())
					print "Accelarometer: %s" % (evt.get_acl())
					print "Magomagnatometer: %s" % (evt.get_mag())
					print "Events:%d Log:%s Logging:%s" % (events,lgf,logflg)
					print "USB device:%s" % usbdev
					print "Hardware status:%s" % evt.get_hws()
					print "Ipaddr:%s Port:%s Sending:%s" % (ipaddr,ipport,udpflg)
					print "Vibration:%s\n" % evt.get_vib()

				elif cmd.find("h") != -1:
					print "Monitor commands:"
					print "   q=quit, s=status, d=toggle_debug, n=toggle_send, l=toggle_log, h=help\n"
					print "Arduino DUE detector commands:"
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
				 
			rc = ser.readline();
			if len(rc) == 0:
				print 'Serial input buffer empty\n'
				break
			else:
				ebuf = evt.parse(rc);
				if len(ebuf) > 1:
					if udpflg:
						sio.send_event_pkt(ebuf,ipaddr,ipport)
						if ipaddr == 'localhost':
							recv = sio.recv_event_pkt()
							if recv != False:
								print recv
							else:
								print "\nRecvFrom: TimeOut Sending:OFF"
								udpflg = False
					if logflg:
						log.write(ebuf)
					events = events + 1
					if debug:
						sys.stdout.write(rc)
				else:
					if debug:
						sys.stdout.write(rc)
					else:
						s = "cosmic_pi:Qsz:%s %s\r" % (evt.get_qsz(),evt.get_time())
						sys.stdout.write(s)
						sys.stdout.flush()

	except Exception, e:
		msg = "Exception: main: %s" % (e)
		print "Fatal: %s" % msg

	finally:
		kbrd.echo_on()
		print "Quitting ..."
		ser.close()
		log.close()
		sio.close()
		time.sleep(1)
		sys.exit(0)
		
if __name__ == '__main__':

    main()
