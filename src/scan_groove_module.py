#!/usr/bin/env python3

from numpy.core.numeric import Inf
from numpy.lib.function_base import append
import rospy
from sensor_msgs.msg import LaserScan
import statistics
import numpy as np
import matplotlib.pyplot as plt

'''Module providing functions to analize readings from LiDAR and to extract specific index value which represents the rail groove'''

class ScanGroove:
    def __init__(self):
        self.sub = rospy.Subscriber('/rotrac_e2/laser/scan', LaserScan, self.callback_fcn)
        self.laser_data = LaserScan()
       
        # ------- PARAMETERS ADJUSTABLE FOR EACH LiDAR TYPE -------
        # self.threshold = 0.035              # Threshold for deviation -> LMS111
        # self.range_a = 150                  # Low limit of range for LiDAR readings set -> LMS111
        # self.range_b = 570                  # High limit of range for LiDAR readings set -> LMS111
        self.threshold = 0.04               # Threshold for deviation -> TIM571
        self.range_a = 105                  # Low limit of range for LiDAR readings set -> TIM571
        self.range_b = 440                  # High limit of range for LiDAR readings set -> TIM571
        # ------- PARAMETERS ADJUSTABLE FOR EACH LiDAR TYPE -------

        rospy.sleep(1)                      # Wait until connection is established

    def callback_fcn(self,msg):             # Read the current LiDAR readings
        self.laser_data = msg

    def analize_readings(self):
        '''Check whether difference between reading and the approximating function is at leat 3,5cm
           When it occures, index of groove is appended to the list'''
        
        self.grooves_index_list = []    
        self.ranges = []                             # Clear the indexes list

        for i in self.laser_data.ranges[self.range_a:self.range_b]:   # Reject the side areas and verify if all readings are valid (Inf veryfication)
            if i != Inf:
                self.ranges.append(i)
            else:
                continue
        
        # Approximating function
        x = np.array(range(len(self.ranges)))       # Calculate the vector x
        y = np.array(self.ranges)                   # Calculate the vector y 
        
        z = np.polyfit(x,y,10)                       # Calculate the coefficients of the function approximating the scanned points
        p = np.poly1d(z)                             # Calculate the function from the polynominal coefficients
                      
        for j in x:                                     # Check the difference for all readings and append to the list detected indexes
            if abs(p(j)-y[j]) >= self.threshold:
                self.grooves_index_list.append(j)
        
        # # -------- TEMPOPRARILY ONLY ------- DISPLAY RESULTS -------
        # rospy.loginfo(self.grooves_index_list)      
        # plt.plot(x,y,'.',x,p(x),'-',x,p(x)+0.035,'-',x,p(x)-0.035,'-')         # Plot the readings as points and the approximating function as a line 
        # plt.show()
        # # -------- TEMPOPRARILY ONLY ------- DISPLAY RESULTS -------

    def get_groove_index(self):
        '''This function analize the grooves indexes list and points the mean index number of the grooves
           One groove = one index number'''

        groove_a_list = []                                                         # Clear variables
        groove_b_list = []
        index_pointer = 0

        if len(self.grooves_index_list) > 1:                                                    # At least two elements in the list
            for i in range(len(self.grooves_index_list)):                                       # Search for the indexes of the first groove
                if i < len(self.grooves_index_list)-1:
                    if abs(self.grooves_index_list[i]-self.grooves_index_list[i+1]) < 20:
                        groove_a_list.append(self.grooves_index_list[i])
                    elif abs(self.grooves_index_list[i]-self.grooves_index_list[i+1]) >= 20:                             
                        groove_a_list.append(self.grooves_index_list[i])
                        index_pointer = i                                                       # End point for groove_a_list
                        break
                else:                                                                           # All list's indexes represent first groove
                    if abs(self.grooves_index_list[i]-self.grooves_index_list[i-1]) < 20:
                        groove_a_list.append(self.grooves_index_list[i])                        # Append the last index
                        index_pointer = i                                                       # End point for groove_a_list
           
            if len(self.grooves_index_list)-index_pointer > 1:                                  # Check if second groove is detected
                for j in range(len(self.grooves_index_list)-1,index_pointer,-1):                # Search for the indexes of the second groove
                    if abs(self.grooves_index_list[j]-self.grooves_index_list[j-1]) < 20:
                        groove_b_list.append(self.grooves_index_list[j])
                    elif abs(self.grooves_index_list[j]-self.grooves_index_list[j-1]) >= 20:                             
                        groove_b_list.append(self.grooves_index_list[j])
                        break
        
        elif len(self.grooves_index_list) == 1:                                                 # One element in the list
            groove_a_list.append(self.grooves_index_list[0])
              
        # -------- TEMPOPRARILY ONLY ------- DISPLAY RESULTS -------
        # rospy.loginfo(f'Full list of indexes detected as grooves: {self.grooves_index_list}')
        # print(f'List of indexes of the first groove: {groove_a_list}')
        # print(f'List of indexes of the second groove: {groove_b_list}')
        # -------- TEMPOPRARILY ONLY ------- DISPLAY RESULTS -------
        
        if len(groove_a_list) >= 1:                                                   # Check if grooves are detected, if so...
            self.groove_a = statistics.mean(groove_a_list)                            # Calculate one index which will point on the groove
        else:
            self.groove_a = 0
        if len(groove_b_list) >= 1: 
            self.groove_b = statistics.mean(groove_b_list)
        else:
            self.groove_b = 0
                    
        rospy.loginfo(f'Groove 1 -> index:{self.groove_a}; Groove 2 -> index:{self.groove_b}')

        return self.groove_a, self.groove_b                                             # Return the indexes
        
if __name__ == "__main__":
    rospy.init_node('scan_groove')
    SG = ScanGroove()
    
    while not rospy.is_shutdown():                                                       # Veryfi the proper groove recognition by running as main fcn
        SG.analize_readings()                                                            # Uncomment results sections to have the results showed 
        SG.get_groove_index()                                                            # in the terminal
        rospy.sleep(2)

