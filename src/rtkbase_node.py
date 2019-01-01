#!/usr/bin/env python
import serial
import datetime as dt


class DecodeBinaryStream:

    msgbuf = []
    lenghtbuf = [] 
    curstate = "header"
    lastbytein  = b'\x20'

    def do_checksum(self, bufferin):
        chksum = 0
        for ii in bufferin:
            chksum = chksum ^ ord(ii)
        return chksum


    def process_byte(self, bytein):
       
        if self.curstate == 'checksum':
            if ord(bytein) != self.do_checksum(self.msgbuf[0:self.msg_length]):
                print "no checksum match"

            self.curstate = 'header'

        if self.curstate == 'body' :
            self.msgbuf.append(bytein) 

            if len(self.msgbuf) == self.msg_length:
                self.curstate = 'checksum'
                if ord(self.msgbuf[0]) == ord(b'\xDF'):
                    print "msg type: " + hex(ord(self.msgbuf[0]))
                    print "nav state: " + hex(ord(self.msgbuf[2]))
                    print "IOD: "+hex(ord(self.msgbuf[1]))
		    print dt.datetime.now().time()

        if self.curstate == 'length':
            self.lengthbuf.append(bytein)
            
            if len(self.lengthbuf) == 2:
                self.msg_length = 256*ord(self.lengthbuf[0])+ord(self.lengthbuf[1]	)
                #print "msg length: " + str(self.msg_length )
                self.curstate = 'body'

        if bytein == b'\xA1' and self.lastbytein == b'\xA0':
            self.curstate = 'length'
            self.lengthbuf = []
            self.msgbuf = []

        self.lastbytein = bytein 


radioout = serial.Serial('/dev/ttyAMA0',19200)
gpsin = serial.Serial('/dev/ttyUSB0',57600,timeout=1000.0)

txcount =0
dec = DecodeBinaryStream()

with serial.Serial('/dev/ttyUSB0',57600,timeout=3.0) as serin:
    while True :
        bytein = gpsin.read(1)
        #print "%s %s"%(dt.datetime.now(),format(ord(bytein),'02x'))
        #serout.write(bytein)
        radioout.write(bytein)

        dec.process_byte(bytein)

        txcount = (txcount + 1)
        if txcount %255 == 0:
		pass
                #print txcount



