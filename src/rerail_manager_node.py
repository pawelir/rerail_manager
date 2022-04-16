#!/usr/bin/env python3

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
        self._rate = rospy.Rate(self.frequency)
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
        self.first_ray_gap = self.first_groove_index - self.first_target_ray
        self.second_ray_gap = self.second_groove_index - self.second_target_ray

    def _rerail_rotrac(self, goal: RerailRotracAction):
        rospy.logdebug('Rerail action started!')
        self._result.success = False

        while not self._result.success:
            if self.rerail_action.is_preempt_requested():
                rospy.logwarn('Rerailing cancelled!')
                self.rerail_action.set_preempted()
            self._feedback.process_state = ProcessState.DETECTION.value            
            self._feedback.grooves_number = self.count_grooves_detection()

            if self._feedback.grooves_number == GroovesNumber.ZERO.value:
                self.rotrac_behaviour.recovery()
            elif self._feedback.grooves_number == GroovesNumber.ONE.value:
                self.rotrac_behaviour.search_for_second_groove(detection_index=self.first_groove_index,
                                                               central_ray=self.central_ray)
            elif self._feedback.grooves_number == GroovesNumber.TWO.value:
                if self.ready_for_alignment():
                    self._feedback.process_state = ProcessState.ALIGNMENT.value
                    self.rerail_action.publish_feedback(self._feedback)
                    self._result.success = self.align_and_block()
                else: 
                    self.rotrac_behaviour.correct_position(self.first_ray_gap,
                                                           self.second_ray_gap)
            
            self.rerail_action.publish_feedback(self._feedback)
            self._rate.sleep()

        self.rerail_action.set_succeeded(self._result)
        rospy.logdebug('Rerail action finished properly!')

    def count_grooves_detection(self) -> GroovesNumber:
        if not self.first_groove_index and not self.second_groove_index:
            return GroovesNumber.ZERO.value
        elif self.first_groove_index and not self.second_groove_index:
            return GroovesNumber.ONE.value
        else:
            return GroovesNumber.TWO.value
    
    def ready_for_alignment(self) -> bool:
        if ((abs(self.first_ray_gap) <= self.maximum_deviation) and
                (abs(self.second_ray_gap) <= self.maximum_deviation)):
            return True
        else: 
            return False

    def align_and_block(self) -> bool:
        self.rough_alignment()
        self.final_alignment()
        self.rotrac_behaviour.stop()
        while not self.rotrac_behaviour.lower_axles():
            self.rotrac_behaviour.raise_axles()
            self.final_alignment()
            self.rotrac_behaviour.stop()
        return True
    
    def rough_alignment(self) -> None:
        self.alignment(linear_velocity=0.6, angular_velocity_factor=60, duration=4)

    def final_alignment(self) -> None:
        self.alignment(linear_velocity=0.3, angular_velocity_factor=120, duration=4)
    
    def alignment(self, linear_velocity: float, angular_velocity_factor: float, duration: float) -> None:
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.rotrac_behaviour.custom_cmd_vel(linear_velocity, self.first_ray_gap / angular_velocity_factor)
            self._rate.sleep

if __name__=="__main__":
    rospy.init_node('rerail_manager')
    try:
        rerail_manager = RerailManager()
        rospy.loginfo("[rerail_manager] Node initialized!")
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
