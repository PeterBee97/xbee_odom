#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import Odometry
from basestation_msgs.msg import Radio
import serial
from optparse import OptionParser
import numpy as np
import time

DIV = 40
counter = 0
port = []
pub = []
dest_addr = b'\x00\x13\xA2\x00\x41\x93\xB5\x1A'
help = """
%prog [options] port my_adr

    port :    serial port of port of the xbee (/dev/ttyUSB0)
    my_adr:   MY address is the 16 bit address of this xbee in the 
              network. This must be a unique address in the network.
              This address is always 0 for the coordinator.  """
parser = OptionParser(usage=help)


parser.add_option('-P', '--pan_id', action="store", type="int", dest="pan_id", default=0x7FFF, help="Pan ID of the xbee network.  This ID must be the same for all XBees in your network.")
parser.add_option('-c', '--channel', action="store", type="string", dest="channel", default="0D", help="Frequency channel for the xbee network. The channel value must be the same for all XBees in your network.")
parser.add_option('-C', '--coordinator', action="store_true", dest="coordinator", default=False, help="Configures the XBee as Coordinator for the network.  Only make the XBee connected to the computer a coordiantor.")
def check(byt):
    byt = bytearray(byt)
    checksum = 0
    for c in byt:
        checksum = checksum + c
    checksum = 0xFF & checksum ^ 0xFF
    return checksum

def senddata(port, addr, data):
    start = b'\x7e'
    length = 0x0B + len(data)
    length = bytearray([length >> 8, length & 0xFF])
    type = b'\x00'
    id = b'\x01'
    options = b'\x00'
    beta = start+length+type+id+addr+options+data
    checksum = check(beta[3:])
    txbuf = str(beta+bytearray([checksum]))
    #for i,c in enumerate(txbuf):
    for c in txbuf:
        #print i,ord(c)
        #if (i > 3) and (c in [0x7E, 0x7D, 0x11, 0x13]):
            #port.write(0x7D)
            #time.sleep(0.06)
            #port.write(c ^ 0x20)
        #else:
        #    port.write(c)
        port.write(c) # no escape
        #time.sleep(0.06)
    time.sleep(0.06)
def send(port, cmd):
    for c in cmd+'\r':
        port.write(c)
        time.sleep(0.06)
		
def setAT(port, cmd):
    port.flushInput()
    send(port, 'AT'+cmd)
    rsp = port.readline()
    print rsp
    if 'OK' in rsp:
        return True
    else :
        return False

baud_lookup= { 1200   : 0, 
			   2400   : 1,
			   4800   : 2,
			   9600   : 3,
			   19200  : 4,
			   38400  : 5,
			   57600  : 6,
			   115200 : 7}
def beginAtMode(port):
	
	for i in range(0,3):
		port.write('+')
		time.sleep(0.05)
	time.sleep(1)
	if port.read(2) == 'OK':
		return True
	else :
		return False

def odom_callback(data):
    global counter
    counter = (counter + 1) % DIV
    #print "counter=%d"%counter
    if counter == 1:
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        theta = euler[2]
        print x,y,theta
        np.set_printoptions(formatter={'int':hex})
        txdata = np.array([x,y,theta]).tobytes()
        print txdata
        senddata(port,dest_addr,txdata)
        x,y,theta = np.frombuffer(txdata)
        print x,y,theta
        
def cmd_callback(cmd):
    global pub
    msgtype, data = np.frombuffer(data)
    radio = Radio()
    radio.header.stamp = rospy.Time.now()
    radio.header.frame_id = 'darpa_map'
    radio.message_type = msgtype
    radio.data = data
    print msgtype, data
    pub.publish(radio)

def xbee_robot():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global port, pub
    opts, args = parser.parse_args()
    baud = 57600
    if len(args) < 2:
        print "Not enough arguments!"
        exit()
    port_name = args[0]
    my_address = int(args[1])
    
    port = serial.Serial(port_name, baud, timeout=0.1)
    if beginAtMode(port):
        print "Connected to the XBee"
    else:
        print "Failed to connect to the XBee"
        exit()

    cmd = 'AP1,CE0,MY%d,BD%d,ID%d,RN1,RO5,WR'%(int(args[1]),baud_lookup[baud],opts.pan_id)
    
    if setAT(port, 'RE'): #reset the xbee
        print "XBee reset"
    else:
        print "Reset failed"
        exit()
    beginAtMode(port)
    time.sleep(1)
    print "Sending command : ", cmd

    if setAT(port, cmd):
        print "XBee sucessfully programed!"
    else:
        print "XBee programming failed.  Try again and then investigate using X-CTU"

    rospy.init_node('xbee_robot', anonymous=True)
    pub = rospy.Publisher('/radio', Radio, queue_size=10)
    rospy.Subscriber('/integrated_to_map', Odometry, odom_callback)

    while True:
        c = port.read()
        if c == '\x7e':
            length = bytearray(port.read(size=2))
            length = (length[0] << 8) + length[1]
            print 'len='+str(length)
            ftype = bytearray(port.read())
            addr = bytearray(port.read(size=8)) #no escape
            #print hex(addr[0])+hex(addr[1])+hex(addr[2])+hex(addr[3])+hex(addr[4])+hex(addr[5])+hex(addr[6])+hex(addr[7])
            if not addr == dest_addr:
                print addr
                continue
            print 'Received '+str(length)+' bytes'
            addr16 = bytearray(port.read(size=2))
            options = bytearray(port.read()) 
            data = bytearray(port.read(size=length-12)) #no escape
            alpha = ftype+addr+addr16+options+data
            checksum = ord(port.read())
            if check(alpha) == checksum:
                cmd_callback(data)
            else:
                print "checksum error"
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    xbee_robot()
