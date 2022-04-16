#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest
import rospy

class RotracBehaviours:
    _default_angular_velocity = 0.2
    _default_angular_velocity = 0.8

    def __init__(self):
        rospy.wait_for_service("axle_bar_manager/raise_axle_bars")
        rospy.wait_for_service("axle_bar_manager/lower_axle_bars")
        self.srv_client_raise_axle = rospy.ServiceProxy("axle_bar_manager/raise_axle_bars", Trigger)
        self.srv_client_lower_axle = rospy.ServiceProxy("axle_bar_manager/lower_axle_bars", Trigger)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._cmd_vel_msg = Twist()
        self._rate = rospy.Rate(rospy.get_param("rerail_manager/frequency"))

    def raise_axles (self) -> bool:
        try:
            request = TriggerRequest()
            response = self.srv_client_raise_axle(request)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("[Service exception] raise_axles service failed: " + str(e))
            return False

    def lower_axles (self) -> bool:
        try:
            request = TriggerRequest()
            response = self.srv_client_lower_axle(request)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("[Service exception] lower_axles service failed: " + str(e))
            return False

    def search_for_second_groove(self, detection_index: int, central_ray: int) -> None:
        if detection_index < central_ray:
            self.rotate_right()
        else:
            self.rotate_left
    
    def correct_position(self, first_ray_gap: int, second_ray_gap: int) -> None:
        if first_ray_gap < 0 and second_ray_gap < 0:
            self.rotate_to_decrease_error(first_ray_gap)
        elif first_ray_gap> 0 and second_ray_gap > 0:
            self.rotate_to_decrease_error(first_ray_gap)
    
    def stop(self) -> None:          
        self._cmd_vel_msg.linear.x = 0
        self._cmd_vel_msg.angular.z = 0
        self.pub_cmd_vel.publish(self._cmd_vel_msg)
        rospy.sleep(1)
  
    def rotate_left(self) -> None:        
        self._cmd_vel_msg.linear.x = 0.2
        self._cmd_vel_msg.angular.z = 0.8
        self.pub_cmd_vel.publish(self._cmd_vel_msg)

    def rotate_right(self) -> None:        
        self._cmd_vel_msg.linear.x = 0.2
        self._cmd_vel_msg.angular.z = -0.8
        self.pub_cmd_vel.publish(self._cmd_vel_msg)

    def rotate_to_decrease_error(self, rays_gap: int) -> None:
        self._cmd_vel_msg.linear.x = 0.4
        self._cmd_vel_msg.angular.z = rays_gap/100
        self.pub_cmd_vel.publish(self._cmd_vel_msg)

    def custom_cmd_vel(self, linear_velocity: float, angular_velocity:float) -> None:        
        self._cmd_vel_msg.linear.x = linear_velocity
        self._cmd_vel_msg.angular.z = angular_velocity
        self.pub_cmd_vel.publish(self._cmd_vel_msg)

    def recovery(self) -> None:      
        self._cmd_vel_msg.linear.x = -0.2
        self._cmd_vel_msg.angular.z = 0
        self.pub_cmd_vel.publish(self._cmd_vel_msg)
        rospy.sleep(2)

if __name__=="__main__":
    rospy.init_node('rotrac_behaviours')
    try:
        rotrac_behaviours = RotracBehaviours()
        rospy.loginfo("[rotrac_behaviours] Node initialized!")
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
