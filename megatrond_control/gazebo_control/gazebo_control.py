#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('megatrond_control')
from std_msgs.msg import Float64
#added
from geometry_msgs.msg import Twist

flw_cmd = rospy.Publisher('/vel_ctrl_flw/command', Float64, queue_size=10)
frw_cmd = rospy.Publisher('/vel_ctrl_frw/command', Float64, queue_size=10)
rrw_cmd = rospy.Publisher('/vel_ctrl_rrw/command', Float64, queue_size=10)
rlw_cmd = rospy.Publisher('/vel_ctrl_rlw/command', Float64, queue_size=10)

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    
    #Constants:
    w = 0.285
    h = 0.635
    l = 0.700
    r = 0.203/2

    #Wheel velocity processing:
    flw_vel = 1/r*(msg.linear.x + msg.linear.y + msg.angular.z*(-w-h))
    frw_vel = 1/r*(-msg.linear.x + msg.linear.y + msg.angular.z*(w+h))
    rrw_vel = 1/r*(msg.linear.x + msg.linear.y + msg.angular.z*(w+h))
    rlw_vel = 1/r*(-msg.linear.x + msg.linear.y + msg.angular.z*(-w-h))

    flw_cmd.publish(flw_vel)
    frw_cmd.publish(frw_vel)
    rrw_cmd.publish(rrw_vel)
    rlw_cmd.publish(rlw_vel)

def gazebo_control():
    rospy.init_node('gazebo_control', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback) 
    rospy.spin()

if __name__ == '__main__':
    try:
        gazebo_control()
    except rospy.ROSInterruptException:
        pass
