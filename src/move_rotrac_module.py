#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

'''Module providing different functions to control the Rotrac's movements'''

class MoveRotrac:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_a_f = rospy.Publisher('/rotrac_e2/front_axle_position_controller/command', Float64, queue_size=1)
        self.pub_a_r = rospy.Publisher('/rotrac_e2/rear_axle_position_controller/command', Float64, queue_size=1)
        self.sub_front_axle_state = rospy.Subscriber('/rotrac_e2/front_axle_position_controller/state', 
                                                                JointControllerState, self.callback_front_axle_state)
        self.sub_rear_axle_state = rospy.Subscriber('/rotrac_e2/rear_axle_position_controller/state', 
                                                                JointControllerState, self.callback_rear_axe_state)
        self.cmd_data = Twist()
        self.axle_data = Float64()
        self.front_axle_sate = JointControllerState()
        self.rear_axle_state = JointControllerState()
        self.rate = rospy.Rate(1)
        rospy.sleep(1)                                     # Wait until connection is established

    def callback_front_axle_state(self,msg):                # Update state of front axle
        self.front_axle_sate = msg
        if self.front_axle_sate.process_value < 0.01:       # Check the axle's position
            self.front_axle_lowered = True
        else:
            self.front_axle_lowered = False
    
    def callback_rear_axe_state(self,msg):                  # Update state of rear axle
        self.rear_axle_state = msg
        if self.rear_axle_state.process_value < 0.01:       # Check the axle's position
            self.rear_axle_lowered = True
        else:
            self.rear_axle_lowered = False
    
    def axle_up (self):
        self.axle_data.data = 0.1
        self.pub_a_f.publish(self.axle_data)
        self.pub_a_r.publish(self.axle_data)
        # rospy.loginfo('Raising axles...')
        rospy.sleep(2)

    def axle_down (self):
        self.axle_data.data = 0.0
        self.pub_a_f.publish(self.axle_data)
        self.pub_a_r.publish(self.axle_data)
        # rospy.loginfo('Lowering axles...')
        rospy.sleep(2)

    def stop(self):          
        self.cmd_data.linear.x = 0
        self.cmd_data.angular.z = 0
        self.pub.publish(self.cmd_data)
        # rospy.loginfo('Stoping Rotrac ...')
        rospy.sleep(1)

    def forward(self,velocity):      
        self.cmd_data.linear.x = velocity
        self.cmd_data.angular.z = 0
        self.pub.publish(self.cmd_data)
        #rospy.loginfo(f'Velocity: {velocity}m/s.Moving Rotrac forward...')
    
    def backward(self,velocity):      
        self.cmd_data.linear.x = -1*velocity
        self.cmd_data.angular.z = 0
        self.pub.publish(self.cmd_data)
        #rospy.loginfo(f'Velocity: -{velocity}m/s. Moving Rotrac backward ...')
    
    def left(self,velocity):        
        self.cmd_data.angular.z = velocity
        self.pub.publish(self.cmd_data)
        #rospy.loginfo(f'Velocity: {velocity}rad/s. Rotating Rotrac left ...')
        
    def right(self,velocity):        
        self.cmd_data.angular.z = -1*velocity
        self.pub.publish(self.cmd_data)
        #rospy.loginfo(f'Velocity: -{velocity}rad/s. Rotating Rotrac right ...')
    
    def custom_cmd(self,linear_x,angular_z):        
        self.cmd_data.linear.x = linear_x
        self.cmd_data.angular.z = angular_z
        self.pub.publish(self.cmd_data)
        #rospy.loginfo(f'Linear velocity: {linear_x}m/s, Angular velocity: {angular_z}rad/s.\nMoving Rotrac custom ...')


if __name__ == "__main__":
    rospy.init_node('move_rotrac')
    MR = MoveRotrac()                                           # Veryfi proper Rotrac's movement by running script as main fcn
    MR.right(0.314, 3)
    rospy.loginfo('Rotrac movement done...')
