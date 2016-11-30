#!      /usr/bin/python
#       coding: utf8

"""
Convert log file into csv
Julian Lewis 2/Aug/2016
"""

from optparse import OptionParser
import sys

def main():
	use = "Usage: %prog --i <input file> --o <output file>"
	parser = OptionParser(usage=use, version="Log file converter")

	parser.add_option("-i", "--input",   help="Path to input  file", dest="ifile", default="./eg.log")
	parser.add_option("-o", "--output",  help="Path to output file", dest="ofile", default="./eg.csv")

	options, args = parser.parse_args()

	ifile = options.ifile
	ofile = options.ofile
	
	print "Options (Input  file)	ifile:%s" % ifile
	print "Options (Output file)	ofile:%s" % ofile

	try:
		ifn = open(ifile, "r")
	except Exception, e:
		msg = "Exception: Cant open input flie: %s" % (e)
		print "Fatal: %s" % msg
		sys.exit(1)

	try:
		ofn = open(ofile, "w")
	except Exception, e:
		msg = "Exception: Cant open output flie: %s" % (e)
		print "Fatal: %s" % msg
		sys.exit(1)

	il = 0
	ol = 0

	lines = ifn.readlines()
	for line in lines:
		il = il + 1
		if (len(line)):
			line = line.replace('\n','')
			if line.find("Evt:") != -1:
				line = line.replace('{','')
				line = line.replace('}','')
				evt = line
			elif line.find("Adc:") != -1:
				adc = line
			elif line.find("192.168") != -1:
				nstr = line.split(',')
				nstr[1] = nstr[1].replace(' ','')
				nstr[1] = nstr[1].replace(')','')
				prt = nstr[1]
				
				ol = ol + 1
				oln = "%s,Prt:%s\n" % (evt,prt)	
				ofn.write(oln)
	
	print "Finished: %d lines read: %d lines written" % (il,ol)

if __name__ == '__main__':

    main()
 
