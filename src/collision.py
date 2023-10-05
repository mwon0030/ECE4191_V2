#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, Float32MultiArray

class Collision():
  def __init__(self):
    # Initialise variables
    self.left_sensor_dist = 200 # in cm
    self.front_left_sensor_dist = 200
    self.front_right_sensor_dist = 200
    self.right_sensor_dist = 200
    self.dist_sensor_readings = {'left': self.left_sensor_dist, 'front-left': self.front_left_sensor_dist, 'front-right': self.front_right_sensor_dist, 'right': self.right_sensor_dist}
    
    self.obstacle_dist_threshold = 15
    self.arena_size = [120, 120]
    self.length = 25
    self.width = 21
    
    self.x = 0
    self.y = 0
    self.th = 0
    
    # Publisher for obstacle detection
    self.obstacle_detect_pub = rospy.Publisher('obstacle_detect', Bool, queue_size=1)
    
    # Subscribers for sensors
    self.ds_front_left_sub = rospy.Subscriber('/ds_front_left', Float32, self.ds_front_left_cb)
    self.ds_front_right_sub = rospy.Subscriber('/ds_front_right', Float32, self.ds_front_right_cb)
    self.ds_left_sub = rospy.Subscriber('/ds_left', Float32, self.ds_left_cb)
    self.ds_right_sub = rospy.Subscriber('/ds_right', Float32, self.ds_right_cb)
    self.th = rospy.Subscriber('/state', Float32MultiArray, self.state_cb) 

    
    
  def ds_front_left_cb(self, data):
    self.front_left_sensor_dist = round(data.data, 4)
    
  def ds_front_right_cb(self, data):
    self.front_right_sensor_dist = round(data.data, 4)
    
  def ds_left_cb(self, data):
    self.left_sensor_dist = round(data.data, 4)
    
  def ds_right_cb(self, data):
    self.right_sensor_dist = round(data.data, 4)
    
  def state_cb(self, data):
    self.x = data.data[0]
    self.y = data.data[1]
    self.th = data.data[2]
    
    
  def obstacle_detect(self):
    number_of_sensors_detecting_obstacle = len([sensor for sensor, dist in self.dist_sensor_readings.items() if dist < self.obstacle_dist_threshold])

    is_near_wall = True if self.x < 20 or self.x > 100 or self.y < 20 or self.y > 100 else False 

    if is_near_wall and number_of_sensors_detecting_obstacle > 1: 
      obstacle_detected = True
    elif not is_near_wall and number_of_sensors_detecting_obstacle: 
      obstacle_detected = True
    else: 
      obstacle_detected = False
    
    self.obstacle_detect_pub.publish(obstacle_detected)


if __name__ == "__main__":
  rospy.init_node('objection_detection')
  obstacle_detection = Collision()
  
  while not rospy.is_shutdown():
    try:
      obstacle_detection.obstacle_detect()
    except rospy.ROSInterruptException:
      break