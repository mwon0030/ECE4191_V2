#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, Float32MultiArray
import time
import numpy as np
import json 

class Localisation():
  def __init__(self):
    self.length = 25 # in cm
    self.width = 21
    self.max_arena_size = [120, 120] # arena dimensions based on home arena 
    self.wheel_rad = 3.21
    self.wheel_circum = 2 * np.pi * self.wheel_rad
    self.wheel_width = 30 # the distance between the left and right wheels
    self.calibration_factor = 1
    
    self.front_left_dist = 200
    self.front_right_dist = 200
    self.left_dist = 200
    self.right_dist = 200
    self.left_motor_speed = 0
    self.right_motor_speed = 0
    self.x = 90
    self.y = 20
    self.th = np.pi/2
    self.turning = False
    self.send_msg = Float32MultiArray()

    self.ref_left_motor_speed = 0

    self.ref_right_motor_speed = 0

    
    self.state_pub = rospy.Publisher('state', Float32MultiArray, queue_size=1)
    
    self.ds_front_left_sub = rospy.Subscriber('/ds_front_left', Float32, self.ds_front_left_cb)
    self.ds_front_right_sub = rospy.Subscriber('/ds_front_right', Float32, self.ds_front_right_cb)
    self.ds_left_sub = rospy.Subscriber('/ds_left', Float32, self.ds_left_cb)
    self.ds_right_sub = rospy.Subscriber('/ds_right', Float32, self.ds_right_cb)
    self.left_motor_sub = rospy.Subscriber('/left_motor', Float32, self.left_motor_cb)
    self.right_motor_sub = rospy.Subscriber('/right_motor', Float32, self.right_motor_cb)
    self.turning_sub = rospy.Subscriber('/turning', Bool, self.turning_cb)
    # self.obstacle_detection_sub = rospy.Subscriber('/')
    self.set_motor_speed_sub = rospy.Subscriber('/set_' + 'left_motor' + '_speed', Float32, self.set_left_motor_speed_cb)
    self.set_motor_speed_sub = rospy.Subscriber('/set_' + 'right_motor' + '_speed', Float32, self.set_right_motor_speed_cb)

    
    self.prev_time = time.time()

    # loading left motor duty_cycle to motor speed mapping
    file_name = "left_motor_duty_cycle_to_speeds.json"
    with open(file_name, "r") as json_file:
        self.left_motor_duty_cycle_to_motor_speed = json.load(json_file)

    # loading right motor duty cycle to motor speed mapping
    file_name = "right_motor_duty_cycle_to_speeds.json"
    with open(file_name, "r") as json_file:
        self.right_motor_duty_cycle_to_motor_speed = json.load(json_file)

    
  def ds_front_left_cb(self, data):
    self.front_left_dist = round(data.data, 4)
    
  def ds_front_right_cb(self, data):
    self.front_right_dist = round(data.data, 4)
    
  def ds_left_cb(self, data):
    self.left_dist = round(data.data, 4)
    
  def ds_right_cb(self, data):
    self.right_dist = round(data.data, 4)
    
  def left_motor_cb(self, data):
    self.left_motor_speed = data.data
  
  def right_motor_cb(self, data):
    self.right_motor_speed = data.data
    
  def turning_cb(self, data):
    self.turning = data.data

  def set_left_motor_speed_cb(self, data):
      self.ref_left_motor_speed = data.data

  def set_right_motor_speed_cb(self, data):
      self.ref_right_motor_speed = data.data
  
  def init_localise(self):
    for _ in range(10):
      self.x = ((self.left_dist + self.width/2) + (self.max_arena_size[0] - self.right_dist - self.width/2))/2
      # self.x = self.max_arena_size[0] - self.right_dist - self.width/2
      self.y = ((self.max_arena_size[1] - self.front_left_dist - self.length/2) + (self.max_arena_size[1] - self.front_right_dist - self.length/2))/2
      # print('x: ', self.x, '   y: ', self.y)
      rospy.sleep(0.025)
    self.send_msg.data = [self.x, self.y, self.th]
    self.state_pub.publish(self.send_msg)
    print("Robot Localised")
    self.prev_time = time.time()
  
  def localise_motor(self): # Localisation relying only on motors
    self.time = time.time() - self.prev_time

    left_motor_speed = self.left_motor_duty_cycle_to_motor_speed[self.ref_left_motor_speed]
    right_motor_speed = self.right_motor_duty_cycle_to_motor_speed[self.ref_right_motor_speed]

    self.th = self.th + (-(left_motor_speed * self.wheel_circum * self.time - right_motor_speed * self.wheel_circum * self.time))/self.wheel_width
    self.x = self.x + ((left_motor_speed * self.wheel_circum * self.time + right_motor_speed * self.wheel_circum * self.time)/2) * self.calibration_factor * np.cos(self.th)
    self.y = self.y + ((left_motor_speed * self.wheel_circum * self.time + right_motor_speed * self.wheel_circum * self.time)/2) *  self.calibration_factor * np.sin(self.th)
    self.prev_time = time.time()
    print('x: ', self.x, '   y: ', self.y, '     th: ', self.th, '     time: ', self.time)
    # print("5")
    # print('left speed: ', self.left_motor_speed, '    right speed: ', self.right_motor_speed, '      time: ', self.time)
    # self.th = self.clamp_angle(self.th)
    self.send_msg.data = [self.x, self.y, self.th]
    self.state_pub.publish(self.send_msg)
    rospy.sleep(0.07)
  
  def localise_sensor(self): # Localisation relying on sensors for x,y and motors for theta
    self.time = time.time() - self.prev_time
    if (self.th >= -np.pi/6 and self.th <= np.pi/6): # When theta is 0
      # print('1')
      self.x = ((self.max_arena_size[0] - self.front_left_dist - self.length/2) + (self.max_arena_size[0] - self.front_right_dist - self.length/2))/2
      self.y = ((self.max_arena_size[1] - self.left_dist - self.width/2) + (self.right_dist + self.width/2))/2

    elif (self.th >= (5/6) * np.pi) or (self.th <= (-5/6) * np.pi): # When theta is pi or -pi
      # print("2")
      self.x = ((self.front_left_dist + self.length/2) + (self.front_right_dist + self.length/2))/2
      self.y = ((self.left_dist + self.width/2) + (self.max_arena_size[1] - self.right_dist - self.width/2))/2

    elif (self.th >= np.pi/3 and self.th <= (2/3) * np.pi): # When theta is pi/2
      # print("3")
      self.x = ((self.left_dist + self.width/2) + (self.max_arena_size[0] - self.right_dist - self.width/2))/2
      self.y = ((self.max_arena_size[1] - self.front_left_dist - self.length/2) + (self.max_arena_size[1] - self.front_right_dist - self.length/2))/2
      
    elif (self.th >= (-2/3) * np.pi and self.th <= (-1/3) * np.pi): # When theta is -pi/2
      # print("4")
      self.x = ((self.max_arena_size[0] - self.left_dist - self.width/2) + (self.right_dist + self.width/2))/2
      self.y = ((self.front_left_dist + self.length/2) + (self.front_right_dist + self.length/2))/2
    print('\n', 'front left: ', self.front_left_dist, '    front right: ', self.front_right_dist, '    left: ', self.left_dist, '    right: ', self.right_dist)
    print('x: ', self.x, '   y: ', self.y, '     th: ', self.th, '     time: ', self.time)
      
    # print('x: ', self.x, '   y: ', self.y)
    self.th = self.th + (-(self.left_motor_speed * self.wheel_circum * self.time + self.right_motor_speed * self.wheel_circum * self.time))/self.wheel_width
    self.th = self.clamp_angle(self.th)
    self.prev_time = time.time()
    self.send_msg.data = [self.x, self.y, self.th]
    self.state_pub.publish(self.send_msg)
    rospy.sleep(0.05)
  
  def is_turning(self):
    turn = self.turning
    return turn
  
  def clamp_angle(self, rad_angle, min_value=-np.pi, max_value=np.pi):
    if min_value > 0:
      min_value *= -1
    angle = (rad_angle + max_value) % (2 * np.pi) + min_value
    return angle

  def localise(self):
    check_sensor_readings_legit = False
    if self.th % np.pi/2 <= 0.05: 
      print('Hi')
      if (self.left_dist + self.width + self.right_dist - self.max_arena_size[1]) < 5: 
        print('Hi2')
        if not self.obstacle_detect: # front sensors 
          check_sensor_readings_legit = True 
          
    if check_sensor_readings_legit: 
        print('using distance sensor coords')
        self.localise_sensor()
    elif self.turning: 
      self.localise_motor_turn()
    else: 
      self.localise_motor()

if __name__ == '__main__':
  rospy.init_node('localisation')
  rospy.sleep(1)
  localiser = Localisation()
  
  # try:
  #   localiser.init_localise()
  # except rospy.ROSInterruptException:
  #   quit()
  
  # while not rospy.is_shutdown():
  #   turning = localiser.is_turning()
  #   # print('\nTurning: ', turning)
  #   if turning:
  #     localiser.localise_motor_turn()
  #   elif not turning:
  #     localiser.localise_motor()
  #   # localiser.localise_motor()  

  while not rospy.is_shutdown():
    # localiser.localise()
    localiser.localise_motor() 