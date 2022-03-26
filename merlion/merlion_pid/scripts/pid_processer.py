#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

class PID_Input_Process_Output:
    def __init__(self):
        rospy.init_node('pid_processer')

        # In order of yaw, roll, pitch, depth
        # For coefficients, in order of Kp, Ki, Kd
        self.values = [0,0,0,0]

        # To change how to hardcode coefficients
        self.coefficients = [[1,1,1] for i in range(4)]
        self.setpoint = [0,0,0,0]

        # Get data from processed imu data
        rospy.Subscriber('yaw', Float64, queue_size=10, self.yaw_callback)
        rospy.Subscriber('roll', Float64, queue_size=10, self.roll_callback)
        rospy.Subscriber('pitch', Float64, queue_size=10, self.pitch_callback)
        rospy.Subscriber('depth', Float64, queue_size=10, self.depth_callback)

        # Get desired output (setpoint)
        rospy.Subscriber('/merlion_pid/yaw/setpoint', Float64, queue_size=10, self.yaw_setpoint_callback)
        rospy.Subscriber('/merlion_pid/roll/setpoint', Float64, queue_size=10, self.roll_setpoint_callback)
        rospy.Subscriber('/merlion_pid/pitch/setpoint', Float64, queue_size=10, self.pitch_setpoint_callback)
        rospy.Subscriber('/merlion_pid/depth/setpoint', Float64, queue_size=10, self.depth_setpoint_callback)

        # Create publishers for each control effort
        yaw_pub = rospy.Publisher('yaw/control_effort', Float64, queue_size=10)
        roll_pub = rospy.Publisher('roll/control_effort', Float64, queue_size=10)
        pitch_pub = rospy.Publisher('pitch/control_effort', Float64, queue_size=10)
        depth_pub = rospy.Publisher('depth/control_effort', Float64, queue_size=10)

        self.publish_control_efforts()


    def yaw_callback(self,msg):
        self.values[0] = msg

    def roll_callback(self,msg):
        self.values[1] = msg

    def pitch_callback(self,msg):
        self.values[2] = msg

    def depth_callback(self,msg):
        self.values[3] = msg

    def yaw_setpoint_callback(self,msg):
        self.setpoint[0] = msg

    def roll_setpoint_callback(self,msg):
        self.setpoint[1] = msg

    def pitch_setpoint_callback(self,msg):
        self.setpoint[2] = msg

    def depth_setpoint_callback(self,msg):
        self.setpoint[3] = msg

    def PID_calculation(self):
        self.control_efforts = [0,0,0,0]
        counter = 0
        for value,coefficient,setpoint in zip(self.values,self.coefficients,self,setpoint):
            # Basic proportion calculation
            self.control_efforts[counter] = coefficient*(setpoint-value)
            counter += 1

    def publish_control_efforts(self):
        self.yaw_pub.publish(self.control_efforts[0])
        self.roll_pub.publish(self.control_efforts[1])
        self.pitch_pub.publish(self.control_efforts[2])
        self.depth_pub.publish(self.control_efforts[3])

        


if __name__ == '__main__':
    PID_Input_Process_Output()
    rospy.spin()