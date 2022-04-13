#!/usr/bin/env python3

from pickle import FALSE
import rospy
import actionlib
from move_rotrac_module import MoveRotrac
from scan_groove_module import ScanGroove
from rerail.msg import RerailRotracAction, RerailRotracFeedback, RerailRotracResult

class RerailRotrac:

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer('/rerail_rotrac', RerailRotracAction, self.rerail_rotrac, False)
        self.action_server.start()
        self.move_rotrac = MoveRotrac()
        self.scan_groove = ScanGroove()
        self.feedback = RerailRotracFeedback()
        self.result = RerailRotracResult()
        self.success = True
        self.ctrl_c = False

        # ----- Select constants depending on simulated LiDAR -----
        # self.target_groove_a = 62       # LMS111
        # self.target_groove_b = 357      # LMS111        
        # self.hz_no = 25                 # LMS111  
        self.target_groove_a = 56       # TIM571
        self.target_groove_b = 279      # TIM571
        self.hz_no = 15                 # TIM571 
        # ----- Select constact depending on simulated LiDAR -----
        self.rate = rospy.Rate(self.hz_no)
  
    def two_grooves_fcn(self):
        '''Case of detection two grooves'''
        if (abs(self.groove_a-self.target_groove_a) <= 3) and (abs(self.groove_b-self.target_groove_b) <= 3):       # Rotrac over the rails set paralell 
            self.align_and_block()
        elif (self.groove_a-self.target_groove_a) < 0 and (self.groove_b-self.target_groove_b) < 0:                 # Rotrac over the rails set too much to the left - rotate right
            self.move_rotrac.custom_cmd(0.4,abs(self.target_groove_a-self.groove_a)/-100)
            self.feedback.linear_vel = 0.4
            self.feedback.angular_vel = abs(self.target_groove_a-self.groove_a)/-100
        elif (self.groove_a-self.target_groove_a) > 0 and (self.groove_b-self.target_groove_b) > 0:                 # Rotrac over the rails set too much to the right - rotate left
            self.move_rotrac.custom_cmd(0.4,abs(self.groove_a-self.target_groove_a)/100)
            self.feedback.linear_vel = 0.4
            self.feedback.angular_vel = abs(self.target_groove_a-self.groove_a)/100

    def one_groove_fcn(self):
        '''Case of detection one groove'''
        if self.groove_a < 210:                                                  # Rotate right
            self.move_rotrac.custom_cmd(0.2,-0.8)
            self.feedback.linear_vel = 0.2
            self.feedback.angular_vel = -0.8

        elif self.groove_a >= 210:                                               # Rotate left
            self.move_rotrac.custom_cmd(0.2,0.8)
            self.feedback.linear_vel = 0.2
            self.feedback.angular_vel = -0.8
    
    def zero_groove_fcn(self):
        '''Case of detection zero groove'''
        self.move_rotrac.backward(0.2)
        self.feedback.linear_vel = -0.2
        self.feedback.angular_vel = 0
        self.action_server.publish_feedback(self.feedback)                    
        rospy.sleep(2)

    def initial_alignment(self):
        for i in range(4*self.hz_no):                                                      # First, initial adjustment
            self.scan_groove.analize_readings()
            self.groove_a, self.groove_b = self.scan_groove.get_groove_index()
            self.move_rotrac.custom_cmd(0.6,(self.groove_a-self.target_groove_a)/60)
            self.feedback.linear_vel = 0.6
            self.feedback.angular_vel = (self.groove_a-self.target_groove_a)/60
            self.action_server.publish_feedback(self.feedback)
            self.rate.sleep()

    def final_alignment(self):
        for i in range(4*self.hz_no):                                                      # Second, final adjustment
            self.scan_groove.analize_readings()
            self.groove_a, self.groove_b = self.scan_groove.get_groove_index()
            self.move_rotrac.custom_cmd(0.3,(self.groove_a-self.target_groove_a)/120)
            self.feedback.linear_vel = 0.3
            self.feedback.angular_vel = (self.groove_a-self.target_groove_a)/120
            self.action_server.publish_feedback(self.feedback)
            self.rate.sleep()

    def align_and_block(self):
        '''Align the position of Rotrac and enable the blockade
           Verify if axles are properly lowered, in case repeat the alignment'''
        
        rospy.loginfo('POSITION ALIGNMENT ...')
        self.initial_alignment()                                                # Initial alignment
        
        # Until axles are not lowered properly, perform final alignment
        while self.move_rotrac.rear_axle_lowered == False or self.move_rotrac.front_axle_lowered == False:
            self.move_rotrac.axle_up()
            self.final_alignment()                                              # Final alignment
            self.move_rotrac.stop()                                             # Stop and enable the blockade
            self.move_rotrac.axle_down()                                                                        
        
        rospy.loginfo('ROTRAC IS SET CORRECTLY')
        self.ctrl_c = True
        self.success = True


    def rerail_rotrac(self,goal):
        '''This fcn analizys the readings and depending on the number of detected grooves perform specific movements'''
        
        self.ctrl_c = False                                                         # Reset this value ach time, the action is called
        self.move_rotrac.axle_up()
        
        while not self.ctrl_c:
            if self.action_server.is_preempt_requested():                           # Check if action is not preempted
                rospy.loginfo('GOAL CANCELLED')
                self.action_server.set_preempted()
                self.ctrl_c = True
                self.success = False

            self.scan_groove.analize_readings()                                     # Analize LiDAR readings
            self.groove_a, self.groove_b = self.scan_groove.get_groove_index()      # Extract index values pointing the rails grooves
            
            # Check how many grooves were detected and decide what to do 
            if self.groove_a != 0 and self.groove_b != 0:                           # Two grooves detected
                self.two_grooves_fcn()
            elif self.groove_a != 0 and self.groove_b == 0:                         # One groove detected
                self.one_groove_fcn()
            elif self.groove_a == 0 and self.groove_b == 0:                         # Zero groove detected
                self.zero_groove_fcn()
            
            self.action_server.publish_feedback(self.feedback)                      # Publish feedback
            self.rate.sleep()                                                       # Set loop frequency

        if self.success:
            self.result.result = True
            rospy.loginfo('ACTION FINISHED')                                        # Finish action and return result
            self.action_server.set_succeeded(self.result)
        

if __name__ == "__main__":
    rospy.init_node('rerail_rotrac')
    RerailRotrac()
    rospy.loginfo('ACTION SERVER READY')
    rospy.spin()