#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Pose2D, Point, Pose, Quaternion, Vector3, Twist
from nav_msgs.msg import Odometry
from mw_seimens_s7_plc_interface.msg import encoder_data
from tf.broadcaster import TransformBroadcaster
from tf import transformations
import struct
import math
import sys
import snap7
from snap7.util import *
import struct
from snap7.snap7types import *


class output(object):
	bool=1
	int=2
	real=3
	word=4
	dword=5

class PLC:
    # define constructor
    def __init__(self):
        self.plc = snap7.client.Client()
        self.ip_address = "0.0.0.0"
        self.connection_status = False
        self.debug = False

    def set_plc_address(self, address):
        self.ip_address = address

    def plc_connection_status(self):
        self.connection_status = self.plc.get_connected()
        return self.connection_status

    def connect_to_plc(self):
        self.plc = snap7.client.Client()
        self.plc.connect(self.ip_address, 0, 1)
        self.connection_status = self.plc.get_connected()
        if(self.plc.get_connected() == False):
            print("ERROR : Unable to connect to PLC")
        else:
            print("Successfully connected to PLC")

    def read_register(self, register, returnByte=False):
        area=0x83
        length=1
        type=0
        out=None
        bit=0
        start=0
        if(register[0].lower()=='m'):
            area=0x83
        if(register[0].lower()=='q'):
            area=0x82
        if(register[0].lower()=='i'):
            area=0x81
        
        if(register[1].lower()=='x'): #bit
            length=1
            out=output().bool
            start = int(register.split('.')[0][2:])
        if(register[1].lower()=='b'): #byte
            length=1
            out=output().int
            start = int(register[2:])
        if(register[1].lower()=='w'): #word
            length=2
            out=output().int
            start = int(register[2:])
        if(register[1].lower()=='d'):
            out=output().dword
            length=4
            if(register.split(".") != -1):
                start = int(register.split('.')[0][2:])
            else:
                start = int(register[2:])
        if('freal' in register.lower()): #double word (real numbers)
            length=4
            start=int(register.lower().replace('freal',''))
            out=output().real
		#print start,hex(area)
        if(output().bool==out):
            bit = int(register.split('.')[1])
        if(self.debug):
            print (register[0].lower(),bit)
        self.plc.read_area(area,0,start,length)
        mbyte=self.plc.read_area(area,0,start,length)
		#print str(mbyte),start,length
        if(returnByte):
            return mbyte
        elif(output().bool==out):
            return get_bool(mbyte,0,bit)
        elif(output().int==out):
            return get_int(mbyte,start)
        elif(output().real==out):
            return get_real(mbyte,0)
        elif(output().dword==out):
            return get_dword(mbyte,0)
        elif(output().word==out):
            return get_int(mbyte,start)

    def write_to_register(self, register, value):
        data=self.read_register(register,True) 
        area=0x83
        length=1
        type=0
        out=None
        bit=0
        start=0
        if(register[0].lower()=='m'):
            area=0x83
        if(register[0].lower()=='q'):
            area=0x82
        if(register[0].lower()=='i'):
            area=0x81
        if(register[1].lower()=='x'): #bit
            length=1
            out=output().bool
            start = int(register.split('.')[0][2:])
            bit = int(register.split('.')[1])
            set_bool(data,0,bit,int(value))
        if(register[1].lower()=='b'): #byte
            length=1
            out=output().int
            start = int(register[2:])
            set_int(data,0,value)
        if(register[1].lower()=='d'):
            out=output().dword
            length=4
            if(register.find(".") != -1):
                start = int(register.split('.')[0][2:])
                set_dword(data,0,value)
            else:
                start = int(register[2:])
                set_dword(data,0,value)
        if(register[1].lower()=='w'):
            out=output().word
            length=4
            if(register.find(".") != -1):
                start = int(register.split('.')[0][2:])
            else:
                start = int(register[2:])
            set_dword(data,0,value)
        if('freal' in register.lower()): #double word (real numbers)
            length=4
            start=int(register.lower().replace('freal',''))
            out=output().real
			#print data
            set_real(data,0,value)
        return self.plc.write_area(area,0,start,data)

    def ReadMemoryBlock(self, area, byte, bit, datatype):
        result = self.plc.read_area(area,0,byte,datatype)
        if datatype==S7WLBit:
            return get_bool(result,0,bit)
        elif datatype==S7WLByte or datatype==S7WLWord:
            return get_int(result,0)
        elif datatype==S7WLReal:
            return get_real(result,0)
        elif datatype==S7WLDWord:
            return get_dword(result,0)
        else:
            return None

    def WriteMemoryBlock(self, area, byte, bit, datatype, value):
        result = self.plc.read_area(area,0,byte,datatype)
        if datatype==S7WLBit:
            set_bool(result,0,bit,value)
        elif datatype==S7WLByte or datatype==S7WLWord:
            set_int(result,0,value)
        elif datatype==S7WLReal:
            set_real(result,0,value)
        elif datatype==S7WLDWord:
            set_dword(result,0,value)
        self.plc.write_area(area,0,byte,result)

    def plc_read(self, register):
        if(register.find(".") != -1):
            return self.read_register(register)
        else:
            if(register[0].lower()=='m'):
                areaM=0x83
            if(register[0].lower()=='q'):
                areaM=0x82
            if(register[0].lower()=='i'):
                areaM=0x81
        if(register[1].lower()=='d'):
            datatype = S7WLDWord
        if(register[1].lower()=='w'):
            datatype = S7WLWord
        addr_str = (register[2:])
        addr = int(addr_str) 
        return self.ReadMemoryBlock(areaM, addr, 0, datatype)

    def plc_write(self, register, value):
        if(register.find(".") != -1):
            self.write_to_register(register, value)
        else:
            if(register[0].lower()=='m'):
                areaM=0x83
            if(register[0].lower()=='q'):
                areaM=0x82
            if(register[0].lower()=='i'):
                areaM=0x81
            if(register[1].lower()=='d'):
                datatype = S7WLDWord
            if(register[1].lower()=='w'):
                datatype = S7WLWord
            addr_str = register[2:]
            addr = int(addr_str)
            self.WriteMemoryBlock(areaM, addr, 0, datatype, value)

class TeleopPLC:
    # Constructor to initialize all the member variables
    def __init__(self):
        # Defining PLC  actuator and sensor values
        self.m1_rpm = 0
        self.m2_rpm = 0
        self.encoder1_val = 0
        self.encoder2_val = 0
        self.wheel_radius = rospy.get_param("wheel_radius", 0.075)
        self.wheel_dist   = rospy.get_param("wheel_dist", 0.435)
        self.last_time = rospy.Time.now()
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0
        self.plc_motor1_rpm = 0
        self.plc_motor2_rpm = 0

        # Defining the registers to read PLC actuators and sensors
        self.s7_plc = PLC()
        self.plc_ip = rospy.get_param("plc_ip_addr", "192.168.0.123")
        self.s7_plc.set_plc_address(self.plc_ip)
        self.m1_addr = rospy.get_param("motor1_addr", "MD6.0")
        self.m2_addr = rospy.get_param("motor2_addr", "MD10")
        self.encoder1_addr = rospy.get_param("encoder1_addr", "MD14")
        self.encoder2_addr = rospy.get_param("encoder2_addr", "MD18")

        # Defining odometer publish frequency
        self.odom_pub_freq = rospy.get_param("odom_pub_freq", 10)
        self.odom_pub_duration = 1.0/(self.odom_pub_freq)

	# connect to plc
        try:
            self.s7_plc.connect_to_plc()
        except snap7.snap7exceptions.Snap7Exception:
            rospy.loginfo("Unable to connect to PLC")
            sys.exit()

        # Defining publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.timer    = rospy.Timer(rospy.Duration(self.odom_pub_duration), self.publish_odom_data)
        self.encoder_pub = rospy.Publisher("encoder_pub", encoder_data, queue_size=10)

        # Defining Subscribers
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
         
    # A callback function for reading in cmd_vel
    def cmd_vel_callback(self, velocity_data):

        # print the velocity being read
        rospy.loginfo("Reading linear velocity [x y w] = [%s %s %s]", velocity_data.linear.x, velocity_data.linear.y, velocity_data.angular.z)

    	# Get Motor1 and Motor2 velocty
        motor1, motor2 = self._velocity_to_rpm(velocity_data)
        self.plc_motor1_rpm = motor1
        self.plc_motor2_rpm = motor2

    # A callback function to publish Odometry Data
    def publish_odom_data(self, timer):
        odom_tf_broadcast = TransformBroadcaster()
    	
        if (self.s7_plc.plc_connection_status()==True):
            # Write to PLC motors
            self.s7_plc.plc_write(self.m1_addr, self.plc_motor1_rpm)
            self.s7_plc.plc_write(self.m2_addr, self.plc_motor2_rpm)
            # Read Encoder Data from PLC
            self.encoder1_val = self.s7_plc.plc_read(self.encoder1_addr)
            self.encoder2_val = self.s7_plc.plc_read(self.encoder2_addr)
        else:
            rospy.loginfo("Not Connected to PLC")

        if (self.encoder1_val > 125):
            self.encoder1_val = self.encoder1_val - 2**32
        if (self.encoder2_val > 125):
            self.encoder2_val = self.encoder2_val - 2**32
        
        rospy.loginfo("Motor1 RPM : %s", str(self.encoder1_val))
        rospy.loginfo("Motor2 RPM : %s", str(self.encoder2_val))
        encoder = encoder_data()
        encoder.stamp = rospy.Time.now()
        encoder.right = self.encoder1_val
        encoder.left  = self.encoder2_val
        self.encoder_pub.publish(encoder)

        # Call function to convert encoder values to linear and angular velocities
        v_x, v_y, w = self._encoder_to_odometry()

        # Set the current time and last recorded time
        current_time = rospy.Time.now()

        # compute odometry information 
        dt      = (current_time - self.last_time).to_sec()
        dx      = (v_x*math.cos(self.pose.theta) - v_y*math.sin(self.pose.theta))*dt
        dy      = (v_x*math.sin(self.pose.theta) + v_y*math.cos(self.pose.theta))*dt
        dtheta  = w * dt
        self.pose.x     = self.pose.x + dx
        self.pose.y     = self.pose.y + dy
        self.pose.theta = self.pose.theta + dtheta

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = transformations.quaternion_from_euler(0, 0, self.pose.theta)

        # publish the transformation on tf
        odom_tf_broadcast.sendTransform((self.pose.x, self.pose.y, 0), odom_quat, current_time, "base_link", "odom")

        # publish odometry message over ROS
        odom_data = Odometry()
        odom_data.header.stamp = current_time
        odom_data.header.frame_id = "odom"

        # set the position
        odom_data.pose.pose = Pose(Point(self.pose.x, self.pose.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom_data.child_frame_id = "base_link"
        odom_data.twist.twist = Twist(Vector3(v_x, v_y, 0), Vector3(0, 0, w))

        # publish the message
        self.odom_pub.publish(odom_data)
        
        self.last_time = current_time

    # A function to convert velocity to RPM
    def _velocity_to_rpm(self, velocity):
        lin_x = velocity.linear.x
        lin_y = velocity.linear.y
        v_lin = math.sqrt(lin_x*lin_x + lin_y*lin_y)
        w = velocity.angular.z

        w_r = 1/(2*self.wheel_radius)*(2*v_lin + self.wheel_dist*w)
        w_l = 1/(2*self.wheel_radius)*(2*v_lin - self.wheel_dist*w)

        m1_rpm = int(9.549297 * w_r)
        m2_rpm = int(9.549297 * w_l)

        if (w_r < 0):
	        m1_rpm = m1_rpm + 2**32
        else:
	        m1_rpm = m1_rpm

        if (w_l < 0):
	        m2_rpm = m2_rpm + 2**32
        else:
	        m2_rpm = m2_rpm

        return m1_rpm, m2_rpm

    # A function to convert encoder to odometry
    def _encoder_to_odometry(self):
        # Convert encoder data to linear and angular velocity
        w_r = self.encoder1_val/9.549297
        w_l = self.encoder2_val/9.549297

        # Calculate Linear and Angular velocity for the robot
        v_lin = (self.wheel_radius/2)*(w_r + w_l)
        w     = (self.wheel_radius/self.wheel_dist)*(w_r - w_l)

        # Calculate x and y component of linear velocity
        v_x = v_lin
        v_y = 0.0

        # Return the linear velocity components(v_x and v_y) and angular velocity(w)
        return v_x, v_y, w
    

if __name__ == '__main__':
    # Initializing Node
    rospy.init_node('teleop_plc')
    try:
        TeleopPLC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
