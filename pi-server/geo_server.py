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
			print msg

		return False	# I love Python

	def close(self):
		self.sik.close()		

def main():
	use = "Usage: %prog [--port=4901 --dirnam=/tmp]"
	parser = OptionParser(usage=use, version="cosmic_pi_server version 1.0")
	
	parser.add_option("-p", "--port",  help="Server portnumber", dest="ipport", type="int", default="4901")
	parser.add_option("-d", "--debug", help="Debug Option", dest="debug", default=False, action="store_true")
	parser.add_option("-o", "--odir",  help="Path to log directory", dest="logdir", default="/tmp")
	parser.add_option("-l", "--log",   help="Event Logging", dest="logflg", default=False, action="store_true")

	options, args = parser.parse_args()

	ipport = options.ipport
	logdir = options.logdir
	debug  = options.debug
	logflg = options.logflg

	print "\n\ncosmic_pi server running, hit '>' for commands\n\n"

	if debug:
		print "\n"
		print "options (Server Port number)	port:%d" % ipport
		print "options (Logging directory)	odir:%s" % logdir
		print "options (Event logging)          log: %s" % logflg
	

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

	try:
		while(True):

			recv = sio.recv_event_pkt();
			if recv:
				print recv

			if kbrd.test_input():
				kbrd.echo_on()
				print "\n"
				cmd = raw_input(">")
        
				if cmd.find("q") != -1:
       					break
 
				kbrd.echo_off()
				 
	except Exception, e:
		msg = "Exception: main: %s" % (e)
		print "Fatal: %s" % msg

	finally:
		kbrd.echo_on()
		print "Quitting ..."
		log.close()
		sik.close()
		time.sleep(1)
		sys.exit(0)
		
if __name__ == '__main__':

    main()
