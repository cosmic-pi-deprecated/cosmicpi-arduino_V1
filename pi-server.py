#!	/usr/bin/python
#	coding: utf8

"""
Handle UDP packets from the cosmic pi and log them
Output can be csv or json depending on command line option -c
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
import httplib, urllib

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

# Send notifications to registered mobile phones
# Currently I am using pushover and the client has to install the
# pushover app and register a user and application token.
# The client supplies the keys on the cosmic pi client launch command line

class Notifications(object):

	def send_ntf(self,kyp,msg):

		nstr = kyp.split('-')

		self.conn = httplib.HTTPSConnection("api.pushover.net:443")
		
		self.conn.request(	"POST", "/1/messages.json",
					urllib.urlencode({	"token"  : nstr[1],
    								"user"   : nstr[0],
    								"sound"  : "Cosmic",
    								"message": msg}), 
				{ "Content-type": "application/x-www-form-urlencoded" })

		self.conn.getresponse()

# Compare time stamps

class Compare(object):

	def __init__(self):

		self.nms = ["Et1","Et2","Et3","Et4","Et5"]

	def cmp(self,reg,evn):
		if evn > 5:
			return

		k = reg.get_len()
		if k > 1:
			i = evn -1;
			r1 = reg.get_reg_by_index(0)
			r2 = reg.get_reg_by_index(1)
			t1 = r1[self.nms[i]]
			t2 = r2[self.nms[i]]
				
			df = abs(t1 - t2)
			if (df > 0.0) and (df < 0.0001):
				print "Muon:%f (%f-%f) %s" % (df,t1,t2,self.nms[i])
 
# Each cosmic pi client can register with the server
# We check package sequence numbers, hardware status

class Registrations(object):

	def __init__(self):
		
		self.reg = {"Ipa":"s","Sqn":0,"Pat":"s","Ntf":False,"Htu":"0","Bmp":"0","Acl":"0","Mag":"0","Gps":"0","Et1":1.0,"Et2":2.0,"Et3":3.0,"Et4":4.0,"Et5":5.0}
		self.regs = []

	def get_len(self):
		return len(self.regs)

	def get_index_by_value(self,knam,kval):
		if self.reg.has_key(knam):
			for i in range(0,len(self.regs)):
				if self.regs[i][knam] == kval:
					return i
		return False

	def get_reg_by_value(self,knam,kval):
		if self.reg.has_key(knam):
			for i in range(0,len(self.regs)):
				if self.regs[i][knam] == kval:
					return self.regs[i]
		return False

	def get_reg_by_index(self,indx):
	  	if indx in range(0,len(self.regs)):
			return self.regs[indx]
		return False

	def get_create_reg(self,knam,kval):
		r = self.get_reg_by_value(knam,kval)
		if r == False:
			i = len(self.regs)
			self.regs.append(self.reg.copy())
			self.regs[i][knam] = kval
			return self.regs[i]
		else:
			return r

	def set_reg(self,r):
		i = self.get_index_by_value("Ipa",r["Ipa"])
		if i == False:
			return False
		self.regs[i] = r.copy()
		return True

# This is the event object, it builds a dictionary from incomming json strings 
# and provides access to the dictionary entries containing the data for each field.

class Event(object):

	def __init__(self):

		# These are the UDP packets containing json strings we are expecting
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

                # Add ons

                self.DAT = { "Dat":"s" }                # Date
                self.SQN = { "Sqn":"0" }                # Sequence number
                self.PAT = { "Pat":"s","Ntf":"0" }      # Pushover application token

		# Now build the main dictionary with one entry for each json string we will process

                self.recd = {   "HTU":self.HTU, "BMP":self.BMP, "VIB":self.VIB, "MAG":self.MAG,
                                "ACL":self.ACL, "LOC":self.LOC, "TIM":self.TIM, "STS":self.STS,
                                "EVT":self.EVT, "DAT":self.DAT, "SQN":self.SQN, "PAT":self.PAT,
                                "DTG":self.DTG, "CMD":self.CMD, "HLP":self.HLP, "TXT":self.TXT,
                                "MEV":self.MEV, "BER":self.BER, "HPU":self.HPU, "UID":self.UID }

		self.newpat = False
		self.newsqn = False

	# Convert the incomming json strings into entries in the dictionary 

	def parse(self, line):					# parse the incomming json strings from arduino
		nstr = line.replace('\n','')			# Throw away <crtn>, we dont want them
		try:
			dic = ast.literal_eval(nstr)		# Build a dictionary entry
			kys = dic.keys()			# Get key names, the first is the address
			if self.recd.has_key(kys[0]):		# Check we know about records with this key
				self.recd[kys[0]] = dic[kys[0]]	# and put it in the dictionary at that address
			
			if kys[0] == "PAT":
				self.newpat = True

			if kys[0] == "SQN":
				self.newsqn = True

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

	def get_sqn(self):
		return self.recd["SQN"]

	def get_pat(self):
		return self.recd["PAT"]

	def get_hpu(self):
		return self.recd["HPU"]

	def get_uid(self):
		return self.recd["UID"]

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
	use = "Usage: %prog [--port=4901 --odir=/tmp]"
	parser = OptionParser(usage=use, version="cosmic_pi_server version 1.0")
	parser.add_option("-p", "--port",  help="Server portnumber", dest="ipport", type="int", default="15443")
	parser.add_option("-d", "--debug", help="Debug Option", dest="debug", default=False, action="store_true")
	parser.add_option("-o", "--odir",  help="Path to log directory", dest="logdir", default="/tmp")
	parser.add_option("-n", "--nolog", help="Event Logging", dest="logflg", default=True, action="store_false")
	parser.add_option("-c", "--csv",   help="Comma seperated value logging", dest="csv", default=False, action="store_true")
	parser.add_option("-b", "--back",  help="Run in background", dest="back", default=False, action="store_true")

	options, args = parser.parse_args()

	ipport = options.ipport
	logdir = options.logdir
	debug  = options.debug
	logflg = options.logflg
	csv    = options.csv
	back   = options.back

	if back:
		Daemon()		

	print ""
	print "cosmic_pi server running, hit '>' for commands\n"

	print "options (Server Port number)	port:%d" % ipport
	print "options (Logging directory)	odir:%s" % logdir
	print "options (Event logging)		log: %s" % logflg
	print "options (Comma Seperated Values) csv: %s" % csv
	print "options (Background Flag)       back: %s" % back

	file_name = "/tmp/pi-server-lock"
	fp = open(file_name, 'w')
	try:
		fcntl.lockf(fp, fcntl.LOCK_EX | fcntl.LOCK_NB)
	except Exception, e:
		print "Lock file:%s is in use" % (file_name)
		print "Only one instance of the server can run at any one time"
		print "Please kill the other instance or remove the lock file"
		sys.exit(1)
	
	ts = time.strftime("%d-%b-%Y-%H-%M-%S",time.gmtime(time.time()))
	lgf = "%s/cosmicpi-logs/%s.srv" % (logdir,ts)
	dir = os.path.dirname(lgf)
	if not os.path.exists(dir):
		os.makedirs(dir)
	try:
		log = open(lgf, "w");
	except Exception, e:
		msg = "Exception: Cant open log file: %s" % (e)
		print "Fatal: %s" % msg
		sys.exit(1)

	if back == False:
		kbrd = KeyBoard()
		kbrd.echo_off()

	sio = Socket_io(ipport)

	evt = Event()

	nfs = Notifications()

	reg = Registrations()

	cmp = Compare()

	newsqn = False
	badhard = False
	display = True
	line = ""

	try:
		while(True):

			recv = sio.recv_event_pkt()
			if len(recv[0]):

				if display:
					print "FromIP:%s" % (str(recv[1]))

				nstr = recv[0].split('*')
				for i in range(0,len(nstr)):
					nstr[i] = nstr[i].replace('\n','')
					#print "Parse:%s" % nstr[i]
					evt.parse(nstr[i])

				if nstr[0].find("EVT") != -1:
					newsqn = True
					evd = evt.get_evt()
					tim = evt.get_tim()
					dat = evt.get_dat()

					cmp.cmp(reg,evd["Evt"])
					r = reg.get_create_reg("Ipa",str(recv[1]))
					if evd["Evt"] == 1:
						r["Et1"] = evd["Etm"]

					if evd["Evt"] == 2:
						r["Et2"] = evd["Etm"]

					if evd["Evt"] == 3:
						r["Et3"] = evd["Etm"]

					if evd["Evt"] == 4:
						r["Et4"] = evd["Etm"]

					if evd["Evt"] == 5:
						r["Et5"] = evd["Etm"]

					#reg.set_reg(r)
					
					if display:
						print
						print "Cosmic Event..: Evt:%s Frq:%s Tks:%s Etm:%s" % (evd["Evt"],evd["Frq"],evd["Tks"],evd["Etm"])
						print "Adc[[Ch0][Ch1]: Adc:%s" % (str(evd["Adc"]))
						print "Time..........: Upt:%s Sec:%s" % (tim["Upt"],tim["Sec"])
						print "Date..........: Dat:%s" % (dat["Dat"])

				elif nstr[0].find("VIB") != -1:
					newsqn = True
					mag = evt.get_mag()
					vib = evt.get_vib()
					tim = evt.get_tim()
					acl = evt.get_acl()
					sqn = evt.get_sqn()
					if display:
						print
						print "Vibration.....: Vax:%s Vcn:%s Sqn:%d" % (vib["Vax"],vib["Vcn"],sqn["Sqn"])
						print "Time..........: Sec:%s" % tim["Sec"]
						print "Accelarometer.: Acx:%s Acy:%s Acz:%s" % (acl["Acx"],acl["Acy"],acl["Acz"])
						print "Magnatometer..: Mgx:%s Mgy:%s Mgz:%s" % (mag["Mgx"],mag["Mgy"],mag["Mgz"])

				elif nstr[0].find("HTU") != -1:
					newsqn = True
					tim = evt.get_tim()
					bmp = evt.get_bmp()
					htu = evt.get_htu()
					loc = evt.get_loc()
					if display:
						print
						print "Barometer.....: Tmb:%s Prs:%s Alb:%s" % (bmp["Tmb"],bmp["Prs"],bmp["Alb"])
						print "Humidity......: Tmh:%s Hum:%s Alt:%s" % (htu["Tmh"],htu["Hum"],loc["Alt"])
						print "Time..........: Sec:%s\n" % tim["Sec"]

				elif nstr[0].find("HPU") != -1:
					hpu = evt.get_hpu();
					if display:
						print
						print "HT Power set..: Ato:%s Hpu:%s Th0:%s Th1:%s Thr:%s Abr:%s" % (hpu["Ato"],hpu["Hpu"],hpu["Th0"],hpu["Th1"],hpu["Thr"],hpu["Abr"])


				elif nstr[0].find("PAT") != -1:
					pat = evt.get_pat()
					print
					print "Notification..: Pat:%s Ntf:%s" % (pat["Pat"],pat["Ntf"])
					if pat["Ntf"] == True:
						msg = "Your are now registered to recieve pi server notifications"
					else:
						msg = "You will no longer recieve pi server notifications"

					nfs.send_ntf(pat["Pat"],msg)

					r = reg.get_create_reg("Ipa",str(recv[1])) 
					r["Pat"] = pat["Pat"] 
					r["Ntf"] = pat["Ntf"]
					#reg.set_reg(r)

				elif nstr[0].find("STS") != -1:
					sts = evt.get_sts()
					r = reg.get_create_reg("Ipa",str(recv[1]))
					r["Htu"] = sts["Htu"]
					r["Bmp"] = sts["Bmp"]
					r["Acl"] = sts["Acl"]
					r["Mag"] = sts["Mag"]
					r["Gps"] = sts["Gps"]
					#reg.set_reg(r)

					msg = ""
					if int(r["Htu"]) == 0:
						msg = msg + "Htu down: "
					if int(r["Bmp"]) == 0:
						msg = msg + "Bmp down: "
					if int(r["Acl"]) == 0:
						msg = msg + "Acl down: "
					if int(r["Mag"]) == 0:
						msg = msg + "Mag down: "
					if int(r["Gps"]) == 0:
						msg = msg + "Gps down: "

					if len(msg) > 0:
						if badhard == False:
							badhard = True
							if r["Ntf"]: 
								nfs.send_ntf(pat["Pat"],msg)
							print "Hardware error:%s %s" % (str(recv[1]),msg)
					else:
						if badhard == True:
							badhard = False
							msg = "Hardware OK again"
							if r["Ntf"]: 
								nfs.send_ntf(pat["Pat"],msg)
							print "%s:%s" % (msg,str(recv[1]))
				if newsqn:
					newsqn = False
					sqn = evt.get_sqn()
					r = reg.get_create_reg("Ipa",str(recv[1]))
					j = int(r["Sqn"])
					i = int(sqn["Sqn"])
					if i != j+1 and j != 0:
						msg = "Sequence error: %s %d-%d" % (str(recv[1]),i,j)
						print msg
						if r["Ntf"]:
							nfs.send_ntf(pat["Pat"],msg)

					r["Sqn"] = i
					#reg.set_reg(r)					

				if logflg:
					if csv:
						 if nstr[0].find("EVT") != -1:
							z1 = "Evt:%s,Frq:%s,Tks:%s,Etm:%s," % (evd["Evt"],evd["Frq"],evd["Tks"],evd["Etm"])

							z2 = "Adc:%s\n" % (str(evd["Adc"]))
							z2 = z2.replace(' ','')

							z3 = str(recv[1])
							z3 = z3.replace('\'','')
							z3 = z3.replace(' ','')
							z3 = z3.replace(')','')
							z3 = z3.replace('(','')
							z3 = "Cli:%s\n" % z3

							line = "%s%s" % (z1,z3)
					else:
						line = "%s - %s\n" % (str(recv[0]),str(recv[1]))

					log.write(line)

			if (back == False) and kbrd.test_input():
				kbrd.echo_on()
				print "\n"
				cmd = raw_input(">")

				if cmd.find("h") != -1:
					print "Commands: h=help, r=registrations, s=status q=quit, x=stop display"

				if cmd.find("x") != -1:
					if display:
						display = False
						print "Event display off"
					else:
						display = True
						print "Event display on"


				if cmd.find("q") != -1:
					break

				if cmd.find("s") != -1:
					print "Server Status"
					print "Log file......:%s" % (lgf)
					print "Registrations.:%d" % (reg.get_len())

				if cmd.find("r") != -1:
					k = reg.get_len()
					if k>0:
						print "Client registrations and status"
						for i in range(0,k):
							r = reg.get_reg_by_index(i)
							print "Idx:%d Ipa:%s Pat:%s Sqn:%d Ntf:%d" % (i,r["Ipa"],r["Pat"],r["Sqn"],r["Ntf"])
							print "Et1:%s Et2:%s Et3:%s" % (r["Et1"],r["Et2"],r["Et3"])

				if back == False:
					kbrd.echo_off()

			ts = time.strftime("%d/%b/%Y %H:%M:%S",time.gmtime(time.time()))
			s = "cosmic_pi_server:[%s]\r" % (ts)
			sys.stdout.write(s)
			sys.stdout.flush()

	except Exception, e:
		msg = "Exception: main: %s" % (e)
		print "Fatal: %s" % msg
		traceback.print_exc(file=sys.stdout)
		
	finally:
		if back == False:
			kbrd.echo_on()
		print "Quitting ..."
		log.close()
		sio.close()
		time.sleep(1)
		sys.exit(0)

if __name__ == '__main__':

    main()

