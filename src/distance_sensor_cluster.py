#! /usr/bin/env python

from gpiozero import DistanceSensor
import rospy
from std_msgs.msg import Float32

class DistanceSensorInfo: 
    def __init__(self, name, ECHO, TRIGGER):
        self.name = name
        self.ECHO = ECHO
        self.TRIGGER = TRIGGER

class DistanceSensorCluster: 
    def __init__(self, dist_sensor_info_dict):
        self.dist_sensor_name_to_echo = {}
        self.distance_sensor_pub = {}
        self.dist_sensor_info_dict = dist_sensor_info_dict 

        self.prev_dist = {sensor_name: 0 for sensor_name in self.dist_sensor_info_dict.keys()}
        self.weight = 0.75
        
        for dist_sensor_name, dist_sensor_info in dist_sensor_info_dict.items():
            self.dist_sensor_name_to_echo[dist_sensor_name] = DistanceSensor(
                max_distance = 4, echo = dist_sensor_info.ECHO, trigger = dist_sensor_info.TRIGGER)
            self.distance_sensor_pub[dist_sensor_name] = rospy.Publisher('ds_' + dist_sensor_name, Float32, queue_size=1)
            # print(self.distance_sensor_pub[dist_sensor_name])
    
    def publish(self):
        for dist_sensor_name, _ in self.dist_sensor_info_dict.items():
            new_dist = (1-self.weight)*round(self.dist_sensor_name_to_echo[dist_sensor_name].distance * 100, 4) + self.prev_dist[dist_sensor_name]
            self.prev_dist[dist_sensor_name] = new_dist
            
            self.distance_sensor_pub[dist_sensor_name].publish(round(self.dist_sensor_name_to_echo[dist_sensor_name].distance * 100, 4))
        rospy.sleep(0.025)
    
if __name__ == "__main__":
    rospy.init_node('distance_sensor_cluster')
    
    front_left_dist_sensor = DistanceSensorInfo(name = 'front_left', ECHO = 11, TRIGGER = 24)
    front_right_dist_sensor = DistanceSensorInfo(name = 'front_right', ECHO = 14, TRIGGER = 12)
    left_dist_sensor = DistanceSensorInfo(name = 'left', ECHO = 23, TRIGGER = 18)
    right_dist_sensor = DistanceSensorInfo(name = 'right', ECHO = 15, TRIGGER = 20)

    distance_sensor_obj_dict = {'left': left_dist_sensor, 'front_left': front_left_dist_sensor, 'front_right': front_right_dist_sensor, 'right': right_dist_sensor}

    dist_sensor_cls = DistanceSensorCluster(distance_sensor_obj_dict)

    
    while not rospy.is_shutdown(): 
        try:
            dist_sensor_cls.publish()
        except rospy.ROSInterruptException:
            break