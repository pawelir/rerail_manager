#!/usr/bin/env python3

from ctypes import alignment
from enum import Enum
from rerail_manager.msg import RerailRotracAction, RerailRotracFeedback, RerailRotracResult
from rotrac_behaviours_node import RotracBehaviours
from std_msgs.msg import Int32MultiArray
import actionlib
import rospy

class ProcessState(Enum):
    DETECTION = "DETECTION"
    ALIGNMENT = "ALIGNMENT"

class GroovesNumber(Enum):
    ZERO = 0
    ONE = 1
    TWO = 2

class RerailManager:
    _feedback = RerailRotracFeedback()
    _result = RerailRotracResult()
    
    def __init__(self):
        self._load_params()
        self.rerail_action = actionlib.SimpleActionServer('/rerail_rotrac',
                                                          RerailRotracAction,
                                                          self._rerail_rotrac,
                                                          auto_start=False)
        self.sub_grooves_indexes = rospy.Subscriber("/groove_scanner/grooves_indexes",
                                                    Int32MultiArray,
                                                    self._grooves_indexes_cb)
        self.rotrac_behaviour = RotracBehaviours()
        self.rate = rospy.Rate(self.frequency)
        self.rerail_action.start()
        while self.sub_grooves_indexes.get_num_connections() < 1:
            rospy.logwarn("Waiting for topic /groove_scanner/grooves_indexes")
            rospy.sleep(0.2)

    def _load_params(self) -> None:
        self.first_target_ray = rospy.get_param("rerail_manager/first_target_ray")
        self.second_target_ray = rospy.get_param("rerail_manager/second_target_ray")
        self.frequency = rospy.get_param("rerail_manager/frequency")
        self.central_ray = rospy.get_param("rerail_manager/central_ray")
        self.maximum_deviation = rospy.get_param("rerail_manager/maximum_deviation")

    def _grooves_indexes_cb(self, msg: Int32MultiArray) -> None:
        self.first_groove_index = msg.data[0]
        self.second_groove_index = msg.data[1]

    def _rerail_rotrac(self, goal: RerailRotracAction):
        
        self._result.success = False
        self._feedback.grooves_number = 0
        self._feedback.process_state = ProcessState.DETECTION.value

        while not self._result.success:
            if self.rerail_action.is_preempt_requested():
                rospy.logwarn('Rerailing cancelled!')
                self.rerail_action.set_preempted()
            
            self._feedback.grooves_number = self.count_grooves_detection()
            
            if self._feedback.grooves_number == 0:
                self.rotrac_behaviour.recovery()
            elif self._feedback.grooves_number == 1:
                self.search_for_second_groove()
            elif self._feedback.grooves_number == 2:
                if self.ready_for_alignment():
                    self.align_and_block()
                else: 
                    self.correct_position()
            
            self.rate.sleep()
            self.rerail_action.publish_feedback(self._feedback)

        rospy.loginfo('Rerail action finished properly!')
        self.rerail_action.set_succeeded(self._result.success)

    def count_grooves_detection(self) -> GroovesNumber:
        if self.first_groove_index == 0 and self.second_groove_index == 0:
            return GroovesNumber.ZERO.value
        elif self.first_groove_index != 0 and self.second_groove_index == 0:
            return GroovesNumber.ONE.value
        else:
            return GroovesNumber.TWO.value
    
    def search_for_second_groove(self) -> None:
        if self.first_groove_index < self.central_ray:
            self.rotrac_behaviour.rotate_right()
        else:
            self.rotrac_behaviour.rotate_left()
    
    def ready_for_alignment(self) -> bool:
        if ((abs(self.first_groove_index - self.first_target_ray) <= self.maximum_deviation) and
                (abs(self.second_groove_index - self.second_target_ray) <= self.maximum_deviation)):
            return True

    def align_and_block(self):
        rospy.logdebug('Starting alignment process...')
        self._feedback.process_state = ProcessState.ALIGNMENT.value
        
        self.rough_alignment()
        self.final_alignment()
        axle_lowered = self.rotrac_behaviour.lower_axles()

        while not axle_lowered:
            self.rotrac_behaviour.raise_axles()
            self.final_alignment()
            self.rotrac_behaviour.stop()
            axle_lowered = self.rotrac_behaviour.lower_axles()                                                                       
        self.rotrac_behaviour.s
        self._result.success = True
        
        rospy.logdebug('Finished alignment process!')

    def align_to_rails(self, linear_velocity: float, angular_velocity_factor: float, duration: float) -> None:
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.rotrac_behaviour.custom_cmd_vel(linear_velocity,
                    (self.first_groove_index - self.first_target_ray)/angular_velocity_factor)
            self.rate.sleep
    
    def rough_alignment(self):
        self.align_to_rails(linear_velocity=0.6, angular_velocity_factor=60, duration=4)

    def final_alignment(self):
        self.align_to_rails(linear_velocity=0.3, angular_velocity_factor=120, duration=4)
    
    def correct_position(self):
        if (self.first_groove_index < self.first_target_ray and
                self.second_groove_index < self.second_target_ray):
            self.rotrac_behaviour.rotate_right_gentle(self.first_target_ray - self.first_groove_index)
        elif ((self.first_groove_index > self.first_target_ray) > 0 and
                (self.second_groove_index > self.second_target_ray)):
            self.rotrac_behaviour.rotate_left_gentle(self.first_target_ray - self.first_groove_index)


if __name__=="__main__":
    rospy.init_node('rerail_manager')
    try:
        rerail_manager = RerailManager()
        rospy.loginfo("[rerail_manager] Node initialized!")
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
