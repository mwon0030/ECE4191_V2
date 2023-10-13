#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, Float32MultiArray, String
import time

class System():
  def __init__(self, colour_to_goal_location_map, start_location, goal_locations):

    self.colour_to_goal_location_map = colour_to_goal_location_map
    self.goal_locations = goal_locations
    self.start_location = start_location
    self.front_left_sensor_dist = 200 
    self.front_right_sensor_dist = 200
    self.left_sensor_dist = 200
    self.right_sensor_dist = 200
    self.dist = {'left': self.left_sensor_dist, 'front-left': self.front_left_sensor_dist, 'front-right': self.front_right_sensor_dist, 'right': self.right_sensor_dist}
    self.x = 0
    self.y = 0
    self.th = 0
    self.is_turning = False
    self.waypoint_reached = False
    self.obstacle_detected = False
    self.wall_detected = False

    self.robot_length = 25 # in cm
    self.robot_width = 21
    

    self.dist_threshold = 0.5
    self.angle_threshold = np.pi/180

    self.max_arena_size = 120
    
    self.ds_front_left_sub = rospy.Subscriber('/ds_front_left', Float32, self.ds_front_left_cb)
    self.ds_front_right_sub = rospy.Subscriber('/ds_front_right', Float32, self.ds_front_right_cb)
    self.ds_left_sub = rospy.Subscriber('/ds_left', Float32, self.ds_left_cb)
    self.ds_right_sub = rospy.Subscriber('/ds_right', Float32, self.ds_right_cb)

    self.left_motor = rospy.Subscriber('/left_motor', Float32, self.left_motor_cb)
    self.right_motor = rospy.Subscriber('/right_motor', Float32, self.right_motor_cb)
    self.th = rospy.Subscriber('/state', Float32MultiArray, self.state_cb)
    self.obstacle_detect_sub = rospy.Subscriber('/obstacle_detect', Bool, self.obstacle_detect_cb)
    self.wall_detect_sub = rospy.Subscriber('/wall_detect', Bool, self.wall_detect_cb)

    
    self.turning_pub = rospy.Publisher('turning', Bool, queue_size=1)
    self.set_left_motor_speed_pub = rospy.Publisher('set_left_motor_speed', Float32, queue_size=1)
    self.set_right_motor_speed_pub = rospy.Publisher('set_right_motor_speed', Float32, queue_size=1)

    self.state_pub = rospy.Publisher('state', Float32MultiArray, queue_size=1)


    # self.colour_sensor_trigger_pub = rospy.Publisher('colour_sensor_trigger', Bool, queue_size=1)
    # self.package_colour_detected_sub = rospy.Subscriber('/package_colour', String, self.package_colour_detected_cb)
 
    self.default_motor_speed  = 0.8

  def ds_front_left_cb(self, data):
    self.front_left_sensor_dist = data.data
    
  def ds_front_right_cb(self, data):
    self.front_right_sensor_dist = data.data
    
  def ds_left_cb(self, data):
    self.left_sensor_dist = data.data
    
  def ds_right_cb(self, data):  
    self.right_sensor_dist = data.data
    
  def left_motor_cb(self, data):
    self.left_motor_speed = data.data 
    
  def right_motor_cb(self, data):
    self.right_motor_speed = data.data
    
  def state_cb(self, data):
    self.x = data.data[0]
    self.y = data.data[1]
    self.th = data.data[2]
    
  def obstacle_detect_cb(self, data):
    self.obstacle_detected = data.data

  def wall_detect_cb(self, data):
    self.wall_detected = data.data

  def package_colour_detected_cb(self, data):
    self.package_colour = data.data

  def drive_to_waypoint(self, goal):

    goal_angle = self.angle_to_turn(goal) + self.th # Relative goal angle + global current angle = global goal angle
    print("goal angle: ", goal_angle)
    self.turn(goal_angle)
    
    distance_to_drive = self.distance_from_goal(goal)
    prev_distance_to_drive = distance_to_drive 
    while distance_to_drive >= self.dist_threshold:
      self.drive(self.default_motor_speed, self.default_motor_speed)
      
      # # obstacle avoidance
      # if self.obstacle_detected and distance_to_drive > 20:
      #   self.obstacle_avoidance()

      # if self.wall_detected and distance_to_drive > 20: 
      #   self.wall_avoidance()

      distance_to_drive = self.distance_from_goal(goal)

      if distance_to_drive > prev_distance_to_drive: 
        break

      prev_distance_to_drive = distance_to_drive
      print("dist error: ", distance_to_drive) 

    print("Drive complete")
    
    self.stop()
      

  def drive(self, left_motor_speed, right_motor_speed):
      self.set_left_motor_speed_pub.publish(left_motor_speed)
      self.set_right_motor_speed_pub.publish(right_motor_speed)

  def stop(self):
    while round(self.left_motor_speed, 4) > 0.0 or round(self.right_motor_speed, 4) > 0.0:
      self.drive(0.0,0.0)
    print("stopped")
      
  def turn(self, goal_angle, stop = True, motor_turn_speed_control_signal = 0.4):
    self.is_turning = True
    self.turning_pub.publish(self.is_turning)
    angle_error = abs(goal_angle - self.th)

    while angle_error > self.angle_threshold:
      if goal_angle > self.th:
        self.drive(-motor_turn_speed_control_signal, motor_turn_speed_control_signal)
        
      elif goal_angle < self.th:
        self.drive(motor_turn_speed_control_signal, -motor_turn_speed_control_signal)

      angle_error = abs(goal_angle - self.th)
      self.turning_pub.publish(self.is_turning)
    
    print("Turn Complete")
    
    if stop:
      self.stop()
    
    self.is_turning = False
    self.turning_pub.publish(self.is_turning)

  def distance_from_goal(self, goal_location):
    x_diff = abs(float(goal_location[0]) - self.x)
    y_diff = abs(float(goal_location[1]) - self.y)
    distance_to_goal = np.hypot(x_diff, y_diff)
    return distance_to_goal
  
  # gives relative goal angle
  def angle_to_turn(self, goal_location): 
    x_diff = float(goal_location[0]) - self.x
    y_diff = float(goal_location[1]) - self.y
    angle = self.clamp_angle(np.arctan2(y_diff, x_diff) - self.th)
    return angle
  
  def clamp_angle(self, rad_angle, min_value=-np.pi, max_value=np.pi):
    if min_value > 0:
      min_value *= -1
    angle = (rad_angle + max_value) % (2 * np.pi) + min_value
    return angle
  
  def determine_goal_location(self): 
    self.colour_sensor_trigger_pub(True) 
    colour_detected = False

    # waiting until new colour is detected 
    while not colour_detected: 
      print('Waiting for colour to be detected....')
      if self.package_colour is not 'None': 
        print('Colour detected!')
        colour_detected = True 

    return self.colour_to_goal_location_map[self.package_colour]

  def deliver_package(self): 

    while self.front_left_sensor_dist > 7.5 or self.front_right_sensor_dist  > 7.5: 
            self.drive(self.default_motor_speed, self.default_motor_speed)


    self.stop()

    # do servo thing 

    # drive backwards 
    while self.front_left_sensor_dist < 10 or self.front_right_sensor_dist < 10: 
      self.drive(-self.default_motor_speed, -self.default_motor_speed)
    
    self.stop()

  def align_dist_sensor(self): 
    prev_diff_distance = abs(self.front_left_sensor_dist - self.front_right_sensor_dist)

    while prev_diff_distance > 0.1: 
      current_diff_distance = abs(self.front_left_sensor_dist - self.front_right_sensor_dist)
      print(f'current_diff_distance: {current_diff_distance}')
      print(f'prev_diff_distance: {prev_diff_distance}')

      # current_diff_distance = abs(self.front_left_sensor_dist - self.front_right_sensor_dist)

      print('aligning distance sensors')

      if self.front_left_sensor_dist > self.front_right_sensor_dist: 
        # turn right 
        angle_increments_to_turn = -np.pi/90
      else: 
        # turn left 
        angle_increments_to_turn = np.pi/90
    
      self.turn(angle_increments_to_turn + self.th, stop = False, motor_turn_speed_control_signal = 0.1)

      
      
      if current_diff_distance > prev_diff_distance + 0.4 and current_diff_distance < 5: 
        print('breaking')
        break

      prev_diff_distance = current_diff_distance
      
      # rospy.sleep(0.025)
    
    self.stop()
    
  def localise_dist_sensor(self): 
    # need to align directly to wall 
    self.align_dist_sensor()

    y = self.max_arena_size - (self.front_right_sensor_dist + self.front_right_sensor_dist)/2 - self.robot_length/2

    print(f'y: {y}' )
    # turn to other wall 
    if self.x > self.max_arena_size/2: 
      self.turn(0)
    else: 
      self.turn(np.pi)


    self.stop()

    self.align_dist_sensor()

    x = self.max_arena_size - (self.front_right_sensor_dist + self.front_right_sensor_dist)/2 - self.robot_length/2

    print(f'x: {x}' )
    self.turn(3*np.pi/2)

    th = -np.pi/2

    send_msg = Float32MultiArray()
    send_msg.data = [x, y, th]
    self.state_pub.publish(send_msg)

  
  def path_planning(self):
    for goal_location in goal_locations:
      # goal_location = self.determine_goal_location()
      # goal_location = [90,80]

      # Drive to goal location
      self.drive_to_waypoint(goal_location)

      print("Goal reached!") 

      # Commence delivery
      self.deliver_package()
      
      print("Delivery complete")
      
      # Relocalise
      # Using distance sensors, turn till straight
      self.localise_dist_sensor()
      # If at delivery location A, turn 90 degrees to the left
      # Else if at delivery location C, turn 90 degrees to the right
      # Else if at delivery location B, turn 90 degrees in the direction the other robot is not in
      
      print("Relocalised")
      
      self.stop()
      
      print("Driving home")

      # self.drive_to_waypoint(self.start_location)
      
      print("Back in loading zone")

  def obstacle_avoidance(self): 
    while self.obstacle_detected: 
      print('Robot detected')
      self.stop()

    print('robot avoided')

  def wall_avoidance(self): 
    self.stop()
    while self.wall_detected: 
      print('Wall detected')

      if self.x > self.max_arena_size/2: # turn left slowly to avoid obstalce
          angle_increment_to_turn = np.pi/90

      else: # turn right slowl to avoid obstacle
          angle_increment_to_turn = -np.pi/90

      self.turn(angle_increment_to_turn + self.th, stop = False, motor_turn_speed_control_signal = 0.1)

    print('wall avoided')



if __name__ == "__main__":
  rospy.init_node('system')
  colour_to_goal_location_map = {'red': [90, 80], 'green': [25, 80]}
  goal_locations = [[30, 90]]
  start_location = [30,20]
  robot = System(colour_to_goal_location_map, start_location, goal_locations)
  rospy.sleep(1)
  
  # robot.path_planning()
  # robot.drive_to_waypoint([30,63])
  # robot.turn(0)
  # robot.turn(-np.pi/2)
  # robot.turn(0)
  # robot.turn(np.pi/2)
  robot.align_dist_sensor()
  
  # while not rospy.is_shutdown():
  #   robot.drive_straight()