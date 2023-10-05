#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, Float32MultiArray, String

class System():
  def __init__(self, colour_to_goal_location_map, start_location):

    self.colour_to_goal_location_map = colour_to_goal_location_map
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
    

    self.dist_threshold = 3
    self.angle_threshold = np.pi/50

    self.max_arena_size = 120
    
    self.ds_front_left_sub = rospy.Subscriber('/ds_front_left', Float32, self.ds_front_left_cb)
    self.ds_front_right_sub = rospy.Subscriber('/ds_front_right', Float32, self.ds_front_right_cb)
    self.ds_left_sub = rospy.Subscriber('/ds_left', Float32, self.ds_left_cb)
    self.ds_right_sub = rospy.Subscriber('/ds_right', Float32, self.ds_right_cb)

    self.left_motor = rospy.Subscriber('/left_motor', Float32, self.left_motor_cb)
    self.right_motor = rospy.Subscriber('/right_motor', Float32, self.right_motor_cb)
    self.th = rospy.Subscriber('/state', Float32MultiArray, self.state_cb)
    self.obstacle_detect_sub = rospy.Subscriber('/obstacle_detect', Bool, self.obstacle_detect_cb)
    
    self.turning_pub = rospy.Publisher('turning', Bool, queue_size=1)
    self.set_left_motor_speed_pub = rospy.Publisher('set_left_motor_speed', Float32, queue_size=1)
    self.set_right_motor_speed_pub = rospy.Publisher('set_right_motor_speed', Float32, queue_size=1)

    # self.colour_sensor_trigger_pub = rospy.Publisher('colour_sensor_trigger', Bool, queue_size=1)
    # self.package_colour_detected_sub = rospy.Subscriber('/package_colour', String, self.package_colour_detected_cb)
 


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

  def package_colour_detected_cb(self, data):
    self.package_colour = data.data

  def drive_to_waypoint(self, goal):

    goal_angle = self.angle_to_turn(goal) + self.th # Relative goal angle + global current angle = global goal angle
    print("goal angle: ", goal_angle)
    self.turn(goal_angle)
    
    distance_to_drive = self.distance_from_goal(goal)
    
    while distance_to_drive >= self.dist_threshold:
      self.drive(0.4, -0.4)
      
      # obstacle avoidance
      if self.obstacle_detected and distance_to_drive > 20:
        print('obstacle detected')
        self.obstacle_avoidance(goal)

      distance_to_drive = self.distance_from_goal(goal)
      print("dist error: ", distance_to_drive) 

    while self.left_motor_speed != 0.0 and self.right_motor_speed != 0.0:
      self.drive(0,0)

  def drive_straight(self, motor_signal = 0.4):
      self.set_left_motor_speed_pub.publish(motor_signal)
      self.set_right_motor_speed_pub.publish(motor_signal)

  def drive(self, left_motor_speed, right_motor_speed):
      self.set_left_motor_speed_pub.publish(left_motor_speed)
      self.set_right_motor_speed_pub.publish(right_motor_speed)
    
  def turn(self, goal_angle):
    self.is_turning = True
    self.turning_pub.publish(self.is_turning)
    angle_error = abs(goal_angle - self.th)

    while angle_error > self.angle_threshold:
      motor_turn_speed_control_signal = 0.20

      if goal_angle > self.th:
        self.drive(-motor_turn_speed_control_signal, -motor_turn_speed_control_signal)
        
      elif goal_angle < self.th:
        self.drive(motor_turn_speed_control_signal, motor_turn_speed_control_signal)

      angle_error = abs(goal_angle - self.th)
      self.turning_pub.publish(self.is_turning)
    
    while self.left_motor_speed != 0.0 and self.right_motor_speed != 0.0:
      self.drive(0,0)
    
    print("Turn Complete")
    rospy.sleep(0.5)
        
    # self.set_left_motor_speed_pub.publish(0)
    # self.set_right_motor_speed_pub.publish(0)
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
      if self.package_colour: 
        colour_detected = True 

    return self.colour_to_goal_location_map[self.package_colour]

  
  def path_planning(self):
    
    # goal_location = self.determine_goal_location()
    goal_location = [30,90]

    self.drive_to_waypoint(goal_location)
    
    # Stop when waypoint is reached
    self.set_left_motor_speed_pub.publish(0)
    self.set_right_motor_speed_pub.publish(0) 

    print("Goal reached!") 

    ## commence delivery
    rospy.sleep(10)

    print("Driving home")

    self.drive_to_waypoint(self.start_location)

  
  def obstacle_avoidance(self, goal):
    # obstacle detection threshold 
    dist_threshold = 15

    # how far do we want to stay in obstacle avoidance mode
    dist_to_travel_threshold = 25
    dist_to_travel = 0

    # at what coordinates is obstacle initially detected
    initial_x = self.x
    initial_y = self.y

    while dist_to_travel < dist_to_travel_threshold: 
      # find out which sensors are seeing obstacle
      obstacle_sensors = {sensor:dist for sensor, dist in self.dist.items() if dist < dist_threshold}
      print(obstacle_sensors)
      if obstacle_sensors: 
        # determine which sensor group is seeing the obstacle
        left_score = len('left' in list(obstacle_sensors.keys()))
        right_score = len('right' in list(obstacle_sensors.keys())) 

        # updated initial obstacle detected coordinates 
        initial_x = self.x
        initial_y = self.y

        # turn in the direction which avoids obstacle (still want to be travelling forward to avoid)
        if left_score > right_score:
          print('turning right to avoid obstacle') 
          left_motor_speed = 0.2
          right_motor_speed = 0.4*left_score

        elif left_score < right_score: 
          print('turning left to avoid obstacle')
          left_motor_speed = 0.4*right_score
          right_motor_speed = 0.2

        # when left score and right score are equal 
        else: 
          left_motor_speed = 0.5
          right_motor_speed = -0.5
          if self.x > self.max_arena_size/2: 
            self.turn(np.pi)
          else: 
            self.turn(0)

        self.drive(left_motor_speed, -right_motor_speed)
      else:
        self.drive_straight()

      dist_to_travel = self.distance_from_goal([initial_x, initial_y])

    print('obstacle avoided. Turning to goal')

    # turn to goal once obstacle is cleared 
    goal_angle = self.angle_to_turn(goal) + self.th # Relative goal angle + global current angle = global goal angle
    print("goal angle: ", goal_angle)
    self.turn(goal_angle)

    rospy.sleep(5)


if __name__ == "__main__":
  rospy.init_node('system')
  colour_to_goal_location_map = {'red': [90, 80], 'green': [25, 80]}
  start_location = [20,30]
  robot = System(colour_to_goal_location_map, start_location)
  rospy.sleep(1)
  
  # robot.path_planning()
  while not rospy.is_shutdown():
    robot.drive_straight()