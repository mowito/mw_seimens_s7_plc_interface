import rospy
from plc_lib.S7_plc_lib import PLC
from geometry_msgs.msg import Pose2D, Point, Pose, Quaternion, Vector3, Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf import transformations
import struct
import math

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
        self.lat_time = rospy.Time.now()
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        # Defining the registers to read PLC actuators and sensors
        self.s7_plc = PLC()
        self.plc_ip = rospy.get_param("plc_ip_addr", "192.168.0.123")
        self.m1_addr = rospy.get_param("motor1_addr", "MD6.0")
        self.m2_addr = rospy.get_param("motor2_addr", "MD10")
        self.encoder1_addr = rospy.get_param("encoder1_addr", "MD14")
        self.encoder2_addr = rospy.get_param("encoder2_addr", "MD18")

        # Defining odometer publish frequency
        self.odom_pub_freq = rospy.get_param("odom_pub_freq", 10)
        self.odom_pub_duration = 1/(self.odom_pub_freq)

        # Defining publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.timer    = rospy.Timer(rospy.Duration(self.odom_pub_duration), self.publish_odom_data)

        # Defining Subscribers
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
         
    # A callback function for reading in cmd_vel
    def cmd_vel_callback(self, velocity_data):

        # print the velocity being read
        rospy.loginfo("Reading linear velocity [x y] = [%s %s]", velocity_data.linear.x, velocity_data.linear.y)

        # connect to plc
        #self.s7_plc.connect_to_plc()

        # proceed to remote control only if PLC connection status is true
        if (True):#s7_plc.plc_connection_status()==True):
            # Get radius, wheel_dist from parameters
            radius = rospy.get_param("radius", 0.075)
            wheel_dist = rospy.get_param("wheel_dist", 0.435)

            # Get Motor1 and Motor2 velocty
            motor1, motor2 = self._velocity_to_rpm(velocity_data)
        
            # Write to PLC motors
            #self.s7_plc.plc_write(self.m1_addr, motor1)
            #self.s7_plc.plc_write(self.m2_addr, motor2)
            rospy.loginfo("Motor1 RPM : %s", str(motor1))
            rospy.loginfo("Motor2 RPM : %s", str(motor2))

            # Read Encoder Data from PLC
            #self.encoder1_val = s7_plc.plc_read(self.encoder1_addr)
            #self.encoder2_val = s7_plc.plc_read(self.encoder2_addr)
            self.encoder1_val = motor1
            self.encoder2_val = motor2
        else:
            rospy.loginfo("Unable to connect to PLC... Remote control aborted")

    # A callback function to publish Odometry Data
    def publish_odom_data(self, timer):

        odom_tf_broadcast = TransformBroadcaster()

        # Call function to convert encoder values to linear and angular velocities
        v_x, v_y, w = self._encoder_to_odometry()

        # Set the current time and last recorded time
        current_time = rospy.Time.now()

        # compute odometry information 
        dt      = (current_time - self.lat_time).to_sec()
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
        
        self.lat_time = current_time

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

        if (m1_rpm < 0):
	        m1_rpm = m1_rpm + 2**8
        else:
	        m1_rpm = m1_rpm

        if (m2_rpm < 0):
	        m2_rpm = m2_rpm + 2**8
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