#!/usr/bin/env python
import serial
import rospy
import datetime as dt
from std_msgs.msg import Int32


def publish_gpsnav_state(state):
   pub = rospy.Publisher("rtkbase_state", Int32, queue_size=10)
   state_out = Int32(state)
   pub.publish(state_out)


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

        message_complete = False
        nav_state = 0
       
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

                     #publish the status of the gps lock
                    nav_state = ord(self.msgbuf[2])
                    message_complete = True 

                   
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

        return message_complete, nav_state



def rtkbase():
    rospy.init_node("rtkbase")
    radioout = serial.Serial('/dev/ttyAMA0',19200)
    gpsin = serial.Serial('/dev/ttyUSB0',57600,timeout=1000.0)

    txcount =0
    dec = DecodeBinaryStream()

    with serial.Serial('/dev/ttyUSB0',57600,timeout=3.0) as serin:
        while not rospy.is_shutdown() :
            
            #read a byte from the gps module
            bytein = gpsin.read(1)

            #send a byte out to the radio
            radioout.write(bytein)

            #locally decode the stream and extract the navigation state
            [message_complete, nav_state] = dec.process_byte(bytein)
            if message_complete == True:
                publish_gpsnav_state(nav_state)


            txcount = (txcount + 1)
            if txcount %255 == 0:
            	pass



if __name__ == '__main__':
    rtkbase()

