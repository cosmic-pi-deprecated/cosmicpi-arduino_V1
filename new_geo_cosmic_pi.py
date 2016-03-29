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

julian.lewis lewis.julian@gmail.com 13/Mar/2016

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

	def __init__(self):

		# Keep this dictionary in sync with the Arduino firmware

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
		self.EVT = { "Evt":"i","Frq":"i","Tks":"i","Etm":"f","Adc0":"[i,i,i,i,i,i,i,i]","Adc1":"[i,i,i,i,i,i,i,i]"  }

		self.recd = {	"HTU":self.HTU, "BMP":self.BMP, "VIB":self.VIB, "MAG":self.MAG, "MOG":self.MOG,
				"ACL":self.ACL, "AOL":self.AOL, "LOC":self.LOC, "TIM":self.TIM, "STS":self.STS,
				"EVT":self.EVT }

		self.oetm = "f"
		self.ovib = "i"

	def parse(self, line):
		nstr = line.split(':')
		for i in range(0,len(nstr)):
			nstr[i] = nstr[i].replace('\n','')

		for i in range(1,len(nstr)-1,2):
			j = i + 1
			try:
				self.recd[nstr[0]][nstr[i]]=nstr[j]

			except Exception, e:
				pass		# Didnt understand, throw it away

	# For weather stations

	def get_weather(self):
		self.weather =	"{" + str(self.HTU) + \
				"," + str(self.BMP) + \
				"," + str(self.LOC) + \
				"," + str(self.TIM) + \
				"}"
		return self.weather

	def get_evt(self):
		if self.oetm == self.EVT["Etm"]:
			return ""

		self.oetm = self.EVT["Etm"]

		# Here is where you can play around with the format of the message strings
		# that will be sent to the server.
		# I am just sending everything in dictionary format, so if there is python
		# program at the recieving end, it trivial to convert back to dictionary form

		self.evt =	"{" + str(self.recd["EVT"]) + \
				"," + str(self.recd["TIM"]) + \
				"," + str(self.recd["LOC"]) + \
				"," + str(self.recd["BMP"]) + \
				"," + str(self.recd["ACL"]) + \
				"," + str(self.recd["MAG"]) + \
				"," + str(self.recd["HTU"]) + \
				"," + str(self.recd["STS"]) + \
				"," + time.asctime(time.gmtime(time.time())) + \
				"}\n"
		return self.evt

	def get_tim(self):
		return self.TIM

	def get_loc(self):
		return self.LOC

	def get_sts(self):
		return self.STS

	def get_bmp(self):
		return self.BMP

	def get_acl(self):
		return self.ACL

	def get_mag(self):
		return self.MAG

	def get_vib(self,flg):
		if flg:
			if self.ovib == self.VIB["Vcn"]:
				return False

			self.ovib = self.VIB["Vcn"]

		return self.VIB

	def get_htu(self):
		return self.HTU

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
					vib = evt.get_vib(0)

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
					print "Vibration.....: Flag:%s" % (vibflg)
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
					vib = evt.get_vib(1)
					if (vib):
						tim = evt.get_tim()
						acl = evt.get_acl()
						mag = evt.get_mag()
						print "Vibration.....: Vax:%s Vcn:%s " % (vib["Vax"],vib["Vcn"])
						print "Accelarometer.: Acx:%s Acy:%s Acz:%s" % (acl["Acx"],acl["Acy"],acl["Acz"])
						print "Magnatometer..: Mgx:%s Mgy:%s Mgz:%s" % (mag["Mgx"],mag["Mgy"],mag["Mgz"])
						print "Time..........: Sec:%s\n" % (tim["Sec"])

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

				ebuf = evt.get_evt()
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
						ts = time.strftime("%d/%b/%Y %H:%M:%S",time.gmtime(time.time()))
						tim = evt.get_tim();
						sts = evt.get_sts();
						s = "cosmic_pi:Upt:%s :Qsz:%s Tim:[%s] %s\r" % (tim["Upt"],sts["Qsz"],ts,tim["Sec"])
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
