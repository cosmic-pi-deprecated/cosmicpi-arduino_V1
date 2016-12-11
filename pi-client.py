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

julian.lewis lewis.julian@gmail.com 11/December/2016 17:00

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

# This is the event object, it builds a dictionary from incomming json strings 
# and provides access to the dictionary entries containing the data for each field.

class Event(object):

	def __init__(self):

		# These are the json strings we are expecting from the arduino

		# N.B. Python interprets leading zeros as octal, so the Sec
		# parameter Sec:hhmmss will screw up at midnight, hence always force
		# base 10 via the int() function when using it: 000900 -> Run time error !!

		self.HTU = { "Tmh":"0.0","Hum":"0.0"             }
		self.BMP = { "Tmb":"0.0","Prs":"0.0","Alb":"0.0" }
		self.VIB = { "Vax":"0"  ,"Vcn":"0"               }
		self.MAG = { "Mgx":"0.0","Mgy":"0.0","Mgz":"0.0" }
		self.MEV = { "Mev":"0"  ,"Met":"0"  ,"Mdx":"0.0" ,"Mdy":"0.0", "Mdz":"0.0" }
		self.ACL = { "Acx":"0.0","Acy":"0.0","Acz":"0.0" }
		self.LOC = { "Lat":"0.0","Lon":"0.0","Alt":"0.0" }
		self.TIM = { "Upt":"0"  ,"Frq":"0"  ,"Sec":"0"   }
		self.DTG = { "Yer":"0"  ,"Mnt":"0"  ,"Day":"0"   }
		self.STS = { "Qsz":"0"  ,"Mis":"0"  ,"Ter":"0","Tmx":"0","Htu":"0","Bmp":"0","Acl":"0","Mag":"0","Gps":"0","Adn":"0","Gri":"0","Eqt":"0","Chm":"0" }
		self.EVT = { "Evt":"0"  ,"Frq":"0"  ,"Tks":"0","Etm":"0.0","Adc":"[[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]]" }
		self.CMD = { "Cmd":"0"  ,"Res":"0"  ,"Msg":"0" }
		self.HLP = { "Idn":"0"  ,"Nme":"0"  ,"Hlp":"0" }
		self.TXT = { "Txt":"0" }
		self.BER = { "Ber":"0"  ,"Adr":"0"  ,"Reg":"0","Bus":"0" }
		self.HPU = { "Ato":"0"  ,"Hpu":"0"  ,"Th0":"0","Th1":"0"  ,"Thr":"0","Abr":"0" }
		self.UID = { "Uid":"0" }
		self.VER = { "Ver":"0" }

		# Add ons

		self.DAT = { "Dat":"s" }		# Date
		self.SQN = { "Sqn":"0" }		# Sequence number
		self.PAT = { "Pat":"s","Ntf":"0" }	# Pushover application token

		# Now build the main dictionary with one entry for each json string we will process

		self.recd = {	"HTU":self.HTU, "BMP":self.BMP, "VIB":self.VIB, "MAG":self.MAG,
				"ACL":self.ACL, "LOC":self.LOC, "TIM":self.TIM, "STS":self.STS,
				"EVT":self.EVT, "DAT":self.DAT, "SQN":self.SQN, "PAT":self.PAT, 
				"DTG":self.DTG, "CMD":self.CMD, "HLP":self.HLP, "TXT":self.TXT,
				"MEV":self.MEV, "BER":self.BER, "HPU":self.HPU, "UID":self.UID,
				"VER":self.VER }

		self.newvib = 0	# Vibration
		self.newmev = 0 # Magnetic event
		self.newevt = 0	# Cosmic ray
		self.newhtu = 0	# Weather report
		self.newcmd = 0 # Command completion available
		self.newhlp = 0 # Help text available
		self.newtxt = 0 # Text to printi
		self.newber = 0 # New bus error
		self.newhpu = 0 # New high voltage setting
	
		self.sqn = 0	# Packet sequenc number

		self.ohum = 0.0	# Old humidity value
		self.otmb = 0.0	# Old barometric temperature value
		self.oprs = 0.0	# Old barometric presure value

	# Convert the incomming json strings into entries in the dictionary 

	def parse(self, line):					# parse the incomming json strings from arduino
		nstr = line.replace('\n','')			# Throw away <crtn>, we dont want them
		try:
			dic = ast.literal_eval(nstr)		# Build a dictionary entry
			kys = dic.keys()			# Get key names, the first is the address
			if self.recd.has_key(kys[0]):		# Check we know about records with this key
				self.recd[kys[0]] = dic[kys[0]]	# and put it in the dictionary at that address

				if kys[0] == "VIB":
					self.newvib = 1

				if kys[0] == "MEV":
					self.newmev = 1
					
				if kys[0] == "EVT":
					self.newevt = 1

				if kys[0] == "HTU":
					self.newhtu = 1

				if kys[0] == "CMD":
					self.newcmd = 1

				if kys[0] == "HLP":
					self.newhlp = 1

				if kys[0] == "TXT":
					self.newtxt = 1

				if kys[0] == "BER":
					self.newber = 1

				if kys[0] == "HPU":
					self.newhpu = 1

		except Exception, e:
			#print e
			#print "BAD:%s" % line
			pass					# Didnt understand, throw it away

	def extract(self, entry):
		if self.recd.has_key(entry):
			nstr = "{\'%s\':%s}" % (entry,str(self.recd[entry]))
			return nstr
		else:
			return ""

	# build weather, cosmic ray and vibration event strings suitable to be sent over the network to server
	# these strings are self describing json format for easy decoding at the server end

	def get_weather(self):
		if self.newhtu:
			self.newhtu = 0
			try:
				hum = float(self.recd["HTU"]["Hum"])
				tmb = float(self.recd["BMP"]["Tmb"])
				prs = float(self.recd["BMP"]["Prs"])

			except Exception, e:
				hum = 0.0
				tmb = 0.0
				prs = 0.0
				pass

			tol = abs(hum - self.ohum) + abs(tmb - self.otmb) + abs(prs - self.oprs)
			if tol > 1.0:
				self.ohum = hum
				self.otmb = tmb
				self.oprs = prs

				self.weather =		self.extract("HTU") + \
						"*" +	self.extract("BMP") + \
						"*" +	self.extract("LOC") + \
						"*" +	self.extract("TIM") + \
						"*" +	self.extract("DAT") + \
						"*" +	self.extract("SQN")
                 
				return self.weather
			
		return ""

	def get_event(self):
		if self.newevt:
			self.newevt = 0
			self.evt =		self.extract("EVT") + \
					"*" +	self.extract("BMP") + \
					"*" +	self.extract("ACL") + \
					"*" +	self.extract("MAG") + \
					"*" +	self.extract("HTU") + \
					"*" +	self.extract("STS") + \
					"*" +	self.extract("LOC") + \
					"*" +	self.extract("TIM") + \
					"*" +	self.extract("DAT") + \
					"*" +	self.extract("SQN")
			return self.evt

		return ""

	def get_vibration(self):
		if self.newvib:
			self.newvib = 0
			self.vib =		self.extract("VIB") + \
					"*" +	self.extract("ACL") + \
					"*" +	self.extract("MAG") + \
					"*" +	self.extract("LOC") + \
					"*" +	self.extract("TIM") + \
					"*" +	self.extract("DAT") + \
					"*" +	self.extract("SQN")
			return self.vib

		return ""

	def get_notification(self):
		if len(self.recd["PAT"]["Pat"]) > 1:
			return self.extract("PAT")
		return ""

	def get_status(self):
		return self.extract("STS")

	# Here we just return dictionaries

	def get_vib(self):
		return self.recd["VIB"]

	def get_mev(self):
		return self.recd["MEV"]

	def get_tim(self):
		return self.recd["TIM"]

	def get_dtg(self):
		return self.recd["DTG"]

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

	def get_evt(self):
		return self.recd["EVT"]

	def get_dat(self):
		self.recd["DAT"]["Dat"] = time.asctime(time.gmtime(time.time()))
		return self.recd["DAT"]

	def get_sqn(self):
		return self.recd["SQN"]

	def nxt_sqn(self):
		self.recd["SQN"]["Sqn"] = self.sqn
		self.sqn = self.sqn + 1

	def get_pat(self):
		return self.recd["PAT"]

	def set_pat(self,token,flag):
		self.recd["PAT"]["Pat"] = token
		self.recd["PAT"]["Ntf"] = flag

	def get_cmd(self):
		return self.recd["CMD"]

	def get_hlp(self):
		return self.recd["HLP"]

	def get_txt(self):
		return self.recd["TXT"]

	def get_ber(self):
		return self.recd["BER"]

	def get_hpu(self):
		return self.recd["HPU"]

	def get_uid(self):
		return self.recd["UID"]

	def get_ver(self):
		return self.recd["VER"]

	def new_cmd(self):
		if self.newcmd:
			self.newcmd = 0
			return 1
		return 0

	def new_hlp(self):
		if self.newhlp:
			self.newhlp = 0
			return 1
		return 0

	def new_txt(self):
		if self.newtxt:
			self.newtxt = 0
			return 1
		return 0

	def new_mev(self):
		if self.newmev:
			self.newmev = 0
			return 1
		return 0

	def new_ber(self):
		if self.newber:
			self.newber = 0
			return 1
		return 0

	def new_hpu(self):
		if self.newhpu:
			self.newhpu = 0
			return 1
		return 0

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
			sent = 0
			while sent < len(pkt):
				sent = sent + self.sok.sendto(pkt[sent:], (ipaddr, ipport))

		except Exception, e:
			msg = "Exception: Can't sendto: %s" % (e)
			print "Sending OFF:%s" % msg
			udpflg = False


	def close(self):
		self.sok.close()

def Daemon():
        """Detach a process from the controlling terminal and run it in the background as a daemon """

        try:
                pid = os.fork()
        except Exception, e:
                msg = "Exception: Background fork: %s" % (e)
                print "Fatal: Can't detach process: %s" % msg
                sys.exit(1)

        if pid == 0:
                os.setsid()

                try:
                        pid = os.fork()

                except Exception, e:
                        msg = "Exception: Background fork: %s" % (e)
                        print "Fatal: Can't detach process: %s" % msg
                        sys.exit(1)

                if pid == 0:
                        os.umask(0)
                else:
                        sys.exit(0)

        else:
                sys.exit(0)
		

def main():
	use = "Usage: %prog [--ip=cosmicpi.ddns.net --port=4901 --usb=/dev/ttyACM0 --debug --dirnam=/tmp]"
	parser = OptionParser(usage=use, version="cosmic_pi version 1.0")
	
	parser.add_option("-i", "--ip",    help="Server IP address or name", dest="ipaddr", default="localhost")
	parser.add_option("-p", "--port",  help="Server portnumber", dest="ipport", type="int", default="15443")
	parser.add_option("-u", "--usb",   help="USB device name", dest="usbdev", default="/dev/ttyACM0")
	parser.add_option("-d", "--debug", help="Debug Option", dest="debug", default=False, action="store_true")
	parser.add_option("-o", "--odir",  help="Path to log directory", dest="logdir", default="/tmp")
	parser.add_option("-n", "--noip",  help="IP Sending", dest="udpflg", default=True, action="store_false")
	parser.add_option("-l", "--log",   help="Event Logging", dest="logflg", default=False, action="store_true")
	parser.add_option("-v", "--vib",   help="Vibration monitor", dest="vibflg", default=False, action="store_true")
	parser.add_option("-w", "--ws",    help="Weather station", dest="wstflg", default=False, action="store_true")
	parser.add_option("-c", "--cray",  help="Cosmic ray sending", dest="evtflg", default=True, action="store_false")
	parser.add_option("-k", "--patk",  help="Server push notification token", dest="patok", default="")
	parser.add_option("-b", "--back",  help="Run in background", dest="back", default=False, action="store_true")

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
	evtflg = options.evtflg
	patok  = options.patok
	back   = options.back

	ptsflg  = False
	display = False	
	pushflg = False
	monflg  = False

	if back:
		Daemon()

	print "\n"
	print "options (Server IP address)     ip   : %s" % ipaddr
	print "options (Server Port number)    port : %d" % ipport
	print "options (USB device name)       usb  : %s" % usbdev
	print "options (Logging directory)     odir : %s" % logdir
	print "options (Event logging)         log  : %s" % logflg
	print "options (UDP sending)           udp  : %s" % udpflg
	print "options (Vibration monitor)     vib  : %s" % vibflg
	print "options (Weather Station)       wst  : %s" % wstflg
	print "options (Cosmic Ray Station)    cray : %s" % evtflg
	print "options (Push notifications)    patk : %s" % patok
	print "options (Debug Flag)            debug: %s" % debug
	print "options (Background Flag)       back : %s" % back 

	print "\ncosmic_pi monitor running, hit '>' for commands\n"

	ts = time.strftime("%d-%b-%Y-%H-%M-%S",time.gmtime(time.time()))
	lgf = "%s/cosmicpi-logs/%s.log" % (logdir,ts)
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
		ser = serial.Serial(port=usbdev, baudrate=9600, timeout=40)
		ser.flush()
	except Exception, e:
		msg = "Exception: Cant open USB device: %s" % (e)
		print "Fatal: %s" % msg
		sys.exit(1)
	
	if back == False:
		kbrd = KeyBoard()
		kbrd.echo_off()

	evt = Event()
	events = 0
	vbrts = 0
	weathers = 0

	sio = Socket_io(ipaddr,ipport)

	time.sleep(1)
	ser.write("JSON 1\n")	
	time.sleep(1)
	ser.write("VERS\n")
	
	try:
		while(True):
			if (back == False) and kbrd.test_input():
				kbrd.echo_on()
				print "\n"
				cmd = raw_input(">")

				if len(cmd) == 1: 
					if cmd.find("q") != -1:
       	       					break
         
					elif cmd.find("d") != -1:
                 				if debug:
							debug = False
						else:
							debug = True
						print "Debug:%s\n" % debug

					elif cmd.find("x") != -1:
                 				if display:
							display = False
						else:
							display = True
						print "Display:%s\n" % display

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

					elif cmd.find("r") != -1:
						if len(patok) > 0:
							if pushflg:
								pushflg = False
								print "Unregister server notifications"
							else:
								pushflg = True
								print "Register for server notifications"
                                                
							if udpflg:
								evt.set_pat(patok,pushflg)
								pbuf = evt.get_notification()
								sio.send_event_pkt(pbuf,ipaddr,ipport)
								sbuf = evt.get_status()
								sio.send_event_pkt(sbuf,ipaddr,ipport)
								print "Sent notification request:%s" % pbuf 
							else:
								print "UDP sending is OFF, can not register with server"
								pbuf = ""
						else:
							print "Token option is not set"
 	 	
					elif cmd.find("s") != -1:
						tim = evt.get_tim()
						dtg = evt.get_dtg()
						sts = evt.get_sts()
						loc = evt.get_loc()
						acl = evt.get_acl()
						mag = evt.get_mag()
						bmp = evt.get_bmp()
						htu = evt.get_htu()
						vib = evt.get_vib()
						mev = evt.get_mev()
						ber = evt.get_ber()
						hpu = evt.get_hpu()
						uid = evt.get_uid()
						ver = evt.get_ver()

						s = "ARDUINO STATUS"
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "FirmwareVer...: Ver:%s" % (ver["Ver"])
						print s
						if ptsflg:
							log.write(s + "\n")
						
						s = "UniqueId......: Uid:%s" % (uid["Uid"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "Status........: Upt:%s Frq:%s Qsz:%s Mis:%s" % (tim["Upt"],tim["Frq"],sts["Qsz"],sts["Mis"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "BusError......: Ber:%s Adr:%s Reg:%s Bus:%s" % (ber["Ber"],ber["Adr"],ber["Reg"],ber["Bus"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "GPS date......: Yer:%s Mnt:%s Day:%s" % (dtg["Yer"],dtg["Mnt"],dtg["Day"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "Parameters....: Adn:%s Gri:%s Eqt:%s Chm:%s" % (sts["Adn"],sts["Gri"],sts["Eqt"],sts["Chm"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "HardwareStatus: Htu:%s Bmp:%s Acl:%s Mag:%s Gps:%s" % (sts["Htu"],sts["Bmp"],sts["Acl"],sts["Mag"],sts["Gps"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "Location......: Lat:%s Lon:%s Alt:%s" % (loc["Lat"],loc["Lon"],loc["Alt"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "Accelarometer.: Acx:%s Acy:%s Acz:%s" % (acl["Acx"],acl["Acy"],acl["Acz"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "Magnatometer..: Mgx:%s Mgy:%s Mgz:%s" % (mag["Mgx"],mag["Mgy"],mag["Mgz"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "Barometer.....: Tmb:%s Prs:%s Alb:%s" % (bmp["Tmb"],bmp["Prs"],bmp["Alb"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "Humidity......: Tmh:%s Hum:%s" % (htu["Tmh"],htu["Hum"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "Vibration.....: Vax:%s Vcn:%s" % (vib["Vax"],vib["Vcn"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "Magnetic Event: Mev:%s Met:%s Mdx:%s Mdy:%s Mdz:%s" % (mev["Mev"],mev["Met"],mev["Mdx"],mev["Mdy"],mev["Mdz"])
						print s
						if ptsflg:
							log.write(s + '\n')

						s = "HT power......: Ato:%s Hpu:%s Th0:%s Th1:%s Thr:%s Abr:%s\n" % (hpu["Ato"],hpu["Hpu"],hpu["Th0"],hpu["Th1"],hpu["Thr"],hpu["Abr"])
						print s
						if ptsflg:
							log.write(s + '\n')

						print "MONITOR STATUS"
						print "USB device....: %s" % (usbdev)
						print "Remote........: Ip:%s Port:%s UdpFlag:%s" % (ipaddr,ipport,udpflg)
						print "Notifications.: Flag:%s Token:%s" % (pushflg, patok)
						print "Vibration.....: Sent:%d Flag:%s" % (vbrts,vibflg)
						print "WeatherStation: Flag:%s" % (wstflg)
						print "Events........: Sent:%d LogFlag:%s" % (events,logflg)
						print "LogFile.......: %s\n" % (lgf)
						print "Display Events: %s\n" % (display)

					elif cmd.find("h") != -1:
						print "MONITOR COMMANDS"
						print "   q=quit, s=status, d=toggle_debug, n=toggle_send, l=toggle_log"
						print "   v=vibration, w=weather, r=toggle_notifications, x=toggle_display" 
						print "   m=monitor_ht, p=ptslog h=help\n"
						print "ARDUINO COMMANDS"
						print ""
						ser.write("HELP")

					elif cmd.find("p") != -1:
						if ptsflg:
							ptsflg = False
						else:
							ptsflg = True

							cms = "%s\n" % "I2CS 0" 
							print cms
							log.write(cms)
							ser.write(cms)
							time.sleep(1)

							cms = "%s\n" % "I2CS 1" 
							print cms
							log.write(cms)
							ser.write(cms)
							time.sleep(1)

							cms = "%s\n" % "GPID" 
							print cms
							log.write(cms)
							ser.write(cms)
							time.sleep(1)

							cms = "%s\n" % "BMID" 
							print cms
							log.write(cms)
							ser.write(cms)
							time.sleep(1)

							cms = "%s\n" % "DHTU" 
							print cms
							log.write(cms)
							ser.write(cms)
							time.sleep(1)

						print "PtsLog:%s\n" % ptsflg

					elif cmd.find("n") != -1:
						if udpflg:
							udpflg = False
						else:
							udpflg = True
						print "Send:%s\n" % udpflg

					elif cmd.find("m") != -1:
						if monflg:
							monflg = False
						else:
							monflg = True;
						print "Monitor HT:%s\n" % monflg

					elif cmd.find("l") != -1:
						if logflg:
							logflg = False
						else:
							logflg = True
						print "Log:%s\n" % logflg

				else:
					cms = "Arduino < %s\n" % cmd 
					print cms
					log.write(cms)
					ser.write(cmd.upper())
					
				if back == False:
					kbrd.echo_off()
			
			# Process Arduino data json strings
			
			try: 
				rc = ser.readline()
				if len(rc) == 0:
					raise Exception("Empty buffer") 

			except Exception, e:
				msg = "Exception: Serial input: %s" % (e)
				print "%s\n"  % msg
				ser.close()
				time.sleep(1)
				ser = serial.Serial(port=usbdev, baudrate=9600, timeout=5)
				rc = ser.readline()
				if len(rc) == 0:
					break
				
				ser.flush()
				rc = ""

				print "Serial Reopened OK"
				time.sleep(1)
				ser.write("JSON 1\n")	
				pass

			if len(rc):
				evt.parse(rc)
				
				if evt.new_cmd():
					acm = evt.get_cmd()
					print ""
					cms = "Cmd:%s->%s %s\n" % (acm["Cmd"],acm["Res"],acm["Msg"])
					print cms
					log.write(cms);

				if evt.new_hlp():
					hlp = evt.get_hlp()
					try:
						print "Hlp:%2d %s %s                                 " % (hlp["Idn"],hlp["Nme"],hlp["Hlp"])

					except Exception, e:
						print "\nData error:%s\n" % (e)
						pass

				if evt.new_txt():
					txt = evt.get_txt()
					print "%s" % (txt["Txt"])

				if evt.new_ber():
					ber = evt.get_ber()
					print "\nBUS ERROR:%s ADDRESS:%s REG:%s BUS:%s" % (ber["Ber"],ber["Adr"],ber["Reg"],ber["Bus"])

				if monflg:
					if evt.new_hpu():
						hpu = evt.get_hpu()
						s = "HPU:Ato:%s Th0:%s Th1:%s" % (hpu["Ato"],hpu["Th0"],hpu["Th1"])
						print "\n%s" % (s)
						log.write(s + "\n")
						if udpflg:
							s = evt.extract("HPU")
							sio.send_event_pkt(s,ipaddr,ipport)

				if vibflg:
					if evt.new_mev():
						mev = evt.get_mev()
						mag = evt.get_mag()
						print ""
						print "Magnetic Event: Mev:%s Met:%s Mdx:%s Mdy:%s Mdz:%s" % (mev["Mev"],mev["Met"],mev["Mdx"],mev["Mdy"],mev["Mdz"])
						print "Magnatometer..: Mgx:%s Mgy:%s Mgz:%s\n" % (mag["Mgx"],mag["Mgy"],mag["Mgz"])
						
				if vibflg:
					vbuf = evt.get_vibration()
					if len(vbuf) > 0:
						vbrts = vbrts + 1
						evt.nxt_sqn()
						dat = evt.get_dat()
						vib = evt.get_vib()
						tim = evt.get_tim()
						acl = evt.get_acl()
						mag = evt.get_mag()
						sqn = evt.get_sqn()
						print ""
						print "Vibration.....: Cnt:%d Vax:%s Vcn:%s " % (vbrts,vib["Vax"],vib["Vcn"])
						print "Accelarometer.: Acx:%s Acy:%s Acz:%s" % (acl["Acx"],acl["Acy"],acl["Acz"])
						print "Magnatometer..: Mgx:%s Mgy:%s Mgz:%s" % (mag["Mgx"],mag["Mgy"],mag["Mgz"])
						print "Time..........: Upt:%s Sec:%s Sqn:%s\n" % (tim["Upt"],tim["Sec"],sqn["Sqn"])
							
						if udpflg:
							sio.send_event_pkt(vbuf,ipaddr,ipport)
						if logflg:
							log.write(vbuf)

						continue
				if wstflg:
					wbuf = evt.get_weather()
					if len(wbuf) > 0:
						weathers = weathers + 1
						evt.nxt_sqn()
						dat = evt.get_dat()
						tim = evt.get_tim()
						bmp = evt.get_bmp()
						htu = evt.get_htu()
						loc = evt.get_loc()
						sqn = evt.get_sqn()
						print ""
						print "Barometer.....: Tmb:%s Prs:%s Alb:%s" % (bmp["Tmb"],bmp["Prs"],bmp["Alb"])
						print "Humidity......: Tmh:%s Hum:%s Alt:%s" % (htu["Tmh"],htu["Hum"],loc["Alt"])
						print "Time..........: Upt:%s Sec:%s Sqn:%s\n" % (tim["Upt"],tim["Sec"],sqn["Sqn"])
							
						if udpflg:
							sio.send_event_pkt(wbuf,ipaddr,ipport)
						if logflg:
							log.write(wbuf)
						
						continue
				if evtflg:
					ebuf = evt.get_event()
					if len(ebuf) > 1:
						events = events + 1
						evt.nxt_sqn()
						dat = evt.get_dat()
						evd = evt.get_evt()
						tim = evt.get_tim()
						sqn = evt.get_sqn()
						if display:
							print ""
							print "Cosmic Event..: Evt:%s Frq:%s Tks:%s Etm:%s" % (evd["Evt"],evd["Frq"],evd["Tks"],evd["Etm"])
							print "Adc[[Ch0][Ch1]: Adc:%s" % (str(evd["Adc"]))
							print "Time..........: Upt:%s Sec:%s Sqn:%s\n" % (tim["Upt"],tim["Sec"],sqn["Sqn"])
        
						if udpflg:
							sio.send_event_pkt(ebuf,ipaddr,ipport)
						if logflg:
							l0 = "\n-----\n"
							l1 = "Evt:%s Frq:%s Tks:%s Etm:%s\n" % (evd["Evt"],evd["Frq"],evd["Tks"],evd["Etm"])
							l2 = "Upt:%s Sec:%s Sqn:%s\n" % (tim["Upt"],tim["Sec"],sqn["Sqn"])
							log.write(l0)
							log.write(l1)
							log.write(l2)
						continue
				if debug:
					sys.stdout.write(rc)
				else:
					ts = time.strftime("%d/%b/%Y %H:%M:%S",time.gmtime(time.time()))
					tim = evt.get_tim();
					sts = evt.get_sts();
					try:
						s = "cosmic_pi:Upt:%s :Qsz:%s Tim:[%s] %s    \r" % (tim["Upt"],sts["Qsz"],ts,tim["Sec"])
					except Exception, e:
						print "\nData error:%s\n" % (e)
						s = ""
						pass

					sys.stdout.write(s)
					sys.stdout.flush()

	except Exception, e:
		msg = "Exception: main: %s" % (e)
		print "Fatal: %s" % msg
		traceback.print_exc()


	finally:
		if back == False:
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
