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
    
    self.obstacle_dist_threshold = 15
    self.wall_dist_threshold = 7.5

    self.arena_size = [120, 120]
    self.length = 25
    self.width = 21
    
    self.x = 0
    self.y = 0
    self.th = 0
    
    # Publisher for obstacle detection
    self.obstacle_detect_pub = rospy.Publisher('obstacle_detect', Bool, queue_size=1)
    # Publisher for wall detection 
    self.wall_detect_pub = rospy.Publisher('wall_detect', Bool, queue_size=1)

    self.wall_detected = False
    self.obstacle_detected = False

    
    # Subscribers for sensors
    self.ds_front_left_sub = rospy.Subscriber('/ds_front_left', Float32, self.ds_front_left_cb)
    self.ds_front_right_sub = rospy.Subscriber('/ds_front_right', Float32, self.ds_front_right_cb)
    self.ds_left_sub = rospy.Subscriber('/ds_left', Float32, self.ds_left_cb)
    self.ds_right_sub = rospy.Subscriber('/ds_right', Float32, self.ds_right_cb)
    self.state_sub = rospy.Subscriber('/state', Float32MultiArray, self.state_cb) 

    
    
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
    dist_sensor_readings = { 'front-left': self.front_left_sensor_dist, 'front-right': self.front_right_sensor_dist}
    number_of_sensors_detecting_obstacle = len([sensor for sensor, dist in dist_sensor_readings.items() if dist < self.obstacle_dist_threshold])

    is_near_wall = True if self.x < 20 or self.x > 100 or self.y < 20 or self.y > 100 else False 

  
    if not(is_near_wall) and number_of_sensors_detecting_obstacle > 0: 
      # print(f'Number of sensors detecting obstacles {number_of_sensors_detecting_obstacle}')
      self.obstacle_detected = False
    else: 
      self.obstacle_detected = False
      # print(f'Number of sensors detecting obstacles {number_of_sensors_detecting_obstacle}')
      

    dist_sensor_readings = {'left': self.left_sensor_dist, 'front-left': self.front_left_sensor_dist, 'front-right': self.front_right_sensor_dist, 'right': self.right_sensor_dist}
    number_of_sensors_detecting_wall = len([sensor for sensor, dist in dist_sensor_readings.items() if dist < self.wall_dist_threshold])

    if is_near_wall and number_of_sensors_detecting_wall > 0: 
      self.wall_detected = False
    else: 
      self.wall_detected = False

    
    self.obstacle_detect_pub.publish(self.obstacle_detected)
    self.wall_detect_pub.publish(self.wall_detected)
    rospy.sleep(0.1)


      
if __name__ == "__main__":
  rospy.init_node('objection_detection')
  obstacle_detection = Collision()
  
  while not rospy.is_shutdown():
    try:
      obstacle_detection.obstacle_detect()
    except rospy.ROSInterruptException:
      break