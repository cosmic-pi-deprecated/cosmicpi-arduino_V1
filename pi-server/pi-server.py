#!	/usr/bin/python
#	coding: utf8

"""
Handle UDP packets from the cosmic pi and log then
julian.lewis lewis.julian@gmail.com 26/Feb/2016
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

class Socket_io(object):

	def __init__(self,ipport):
		try:
			self.sik = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.sik.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
			self.sik.setblocking(0)
			self.sik.bind(("",ipport))

		except Exception, e:
			msg = "Exception: Can't open Socket: %s" % (e)
			print msg
			sys.exit(1)

	def recv_event_pkt(self):
		try:
			available = select.select([self.sik], [], [], 1)
			if available[0]:
				recv = self.sik.recvfrom(2048)
				return recv

		except Exception, e:
			msg = "Exception: Can't recvfrom: %s" % (e)
			print msg

		return ["",""]

	def close(self):
		self.sik.close()

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

	def get_evt(self):
		return self.recd["EVT"]

	def get_dat(self):
		return self.recd["DAT"]

def main():
	use = "Usage: %prog [--port=4901 --odir=/tmp]"
	parser = OptionParser(usage=use, version="cosmic_pi_server version 1.0")
	parser.add_option("-p", "--port",  help="Server portnumber", dest="ipport", type="int", default="15443")
	parser.add_option("-d", "--debug", help="Debug Option", dest="debug", default=False, action="store_true")
	parser.add_option("-o", "--odir",  help="Path to log directory", dest="logdir", default="/tmp")
	parser.add_option("-n", "--nolog", help="Event Logging", dest="logflg", default=True, action="store_false")

	options, args = parser.parse_args()

	ipport = options.ipport
	logdir = options.logdir
	debug  = options.debug
	logflg = options.logflg

	print ""
	print "cosmic_pi server running, hit '>' for commands\n"

	print "options (Server Port number)	port:%d" % ipport
	print "options (Logging directory)	odir:%s" % logdir
	print "options (Event logging)		log: %s" % logflg

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

	kbrd = KeyBoard()
	kbrd.echo_off()

	sio = Socket_io(ipport)

	evt = Event()

	try:
		while(True):

			recv = sio.recv_event_pkt()
			if len(recv[0]):

				if debug:
					print "FromIP:%s" % (str(recv[1]))

					nstr = recv[0].split('*')
					for i in range(0,len(nstr)):
						nstr[i] = nstr[i].replace('\n','')
						#print "Parse:%s" % nstr[i]
						evt.parse(nstr[i])

					if nstr[0].find("EVT") != -1:
						evd = evt.get_evt()
						tim = evt.get_tim()
						dat = evt.get_dat()
						print
						print "Cosmic Event..: Evt:%s Frq:%s Tks:%s Etm:%s" % (evd["Evt"],evd["Frq"],evd["Tks"],evd["Etm"])
						print "Adc[[Ch0][Ch1]: Adc:%s" % (str(evd["Adc"]))
						print "Time..........: Upt:%s Sec:%s" % (tim["Upt"],tim["Sec"])
						print "Date..........: Dat:%s" % (dat["Dat"])

					elif nstr[0].find("VIB") != -1:
						mag = evt.get_mag()
						vib = evt.get_vib()
						tim = evt.get_tim()
						acl = evt.get_acl()
						print
						print "Vibration.....: Vax:%s Vcn:%s " % (vib["Vax"],vib["Vcn"])
						print "Time..........: Sec:%s" % (tim["Sec"])
						print "Accelarometer.: Acx:%s Acy:%s Acz:%s" % (acl["Acx"],acl["Acy"],acl["Acz"])
						print "Magnatometer..: Mgx:%s Mgy:%s Mgz:%s" % (mag["Mgx"],mag["Mgy"],mag["Mgz"])

					elif nstr[0].find("HTU") != -1:
						tim = evt.get_tim()
						bmp = evt.get_bmp()
						htu = evt.get_htu()
						loc = evt.get_loc()
						print
						print "Barometer.....: Tmb:%s Prs:%s Alb:%s" % (bmp["Tmb"],bmp["Prs"],bmp["Alb"])
						print "Humidity......: Tmh:%s Hum:%s Alt:%s" % (htu["Tmh"],htu["Hum"],loc["Alt"])
						print "Time..........: Sec:%s\n" % (tim["Sec"])

				if logflg:
					line = "%s - %s" % (str(recv[0]),str(recv[1]))
					log.write(line)
					log.write("\n\n")

			if kbrd.test_input():
				kbrd.echo_on()
				print "\n"
				cmd = raw_input(">")

				if cmd.find("q") != -1:
					break

				kbrd.echo_off()

			ts = time.strftime("%d/%b/%Y %H:%M:%S",time.gmtime(time.time()))
			s = "cosmic_pi_server:[%s]\r" % (ts)
			sys.stdout.write(s)
			sys.stdout.flush()

	except Exception, e:
		msg = "Exception: main: %s" % (e)
		print "Fatal: %s" % msg

	finally:
		kbrd.echo_on()
		print "Quitting ..."
		log.close()
		sio.close()
		time.sleep(1)
		sys.exit(0)

if __name__ == '__main__':

    main()

