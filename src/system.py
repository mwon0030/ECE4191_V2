#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, Float32MultiArray, String
import time 
from gpiozero import Servo
import RPi.GPIO as GPIO


class ColourSensor: 
    NUM_CYCLES = 10
    colours = ['red', 'green', 'blue']
    
    def __init__(self, pin_S2, pin_S3, pin_OUT) -> None:
        self.pin_S2 = pin_S2
        self.pin_S3 = pin_S3
        self.pin_OUT = pin_OUT

        self.readCount = 0
        self.maxRead = 100
        self.diffThreshold = 30 #difference in average frequency values  

        self.start_detecting = False

        # self.colour_sensor_trigger_sub = rospy.Subscriber('/colour_sensor_trigger', Bool, self.colour_sensor_trigger_cb)

        # self.package_colour_pub = rospy.Publisher('package_colour', String, queue_size=1)
    
    def colour_sensor_trigger_cb(self, data): 
        self.start_detecting = data.data
        
    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_OUT,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_S2,GPIO.OUT)
        GPIO.setup(self.pin_S3,GPIO.OUT)
        print("\n")

    def find_frequency(self):
        start = time.time()
        #see how long it takes to read NUM_CYCLES
        for impulse_count in range(ColourSensor.NUM_CYCLES):
            GPIO.wait_for_edge(self.pin_OUT, GPIO.FALLING)
        duration = time.time() - start
        frequency = ColourSensor.NUM_CYCLES/duration
        return frequency
    
    def detect_loop(self):
        # set pins to read red
        GPIO.output(self.pin_S2,GPIO.LOW)
        GPIO.output(self.pin_S3,GPIO.LOW)

        red  = self.find_frequency()   
    
        #set pins to read blue
        GPIO.output(self.pin_S2,GPIO.LOW)
        GPIO.output(self.pin_S3,GPIO.HIGH)


        blue = self.find_frequency() - 3000
        
        #set pins to read green
        GPIO.output(self.pin_S2,GPIO.HIGH)
        GPIO.output(self.pin_S3,GPIO.HIGH)

        green = self.find_frequency()
        

        # print("red: ", red, "blue: ", blue, "green: ", green)
        
        colour_frequencies = [red, blue, green]
        package_colour_idx = colour_frequencies.index(max(colour_frequencies))
        colour_frequencies_keys = {"red": red, "blue": blue, "green": green}
        package_colour = list(colour_frequencies_keys)[package_colour_idx]
        
        # self.package_colour_pub.publish(package_colour)
        
        time.sleep(0.1)
        
        return package_colour

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
    self.wall_detected = False

    # self.servo = Servo(19)
    # self.servo.value = 0
    
    self.robot_length = 25 # in cm
    self.robot_width = 21
    
    self.right_motor_offset = 0.0

    self.dist_threshold = 1
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

    self.goal_location = []

    # self.colour_sensor_trigger_pub = rospy.Publisher('colour_sensor_trigger', Bool, queue_size=1)
    # self.package_colour_detected_sub = rospy.Subscriber('/package_colour', String, self.package_colour_detected_cb)
 
    self.default_motor_speed  = 0.6

    self.goal_location_publisher = rospy.Publisher('goal_location', Float32MultiArray, queue_size = 1)
    
    self.package_colour = "Initialised"
    
    # self.servo = Servo(19)
    
    pin_S2 = 6 
    pin_S3 = 13
    pin_OUT = 5
    self.colour_sensor = ColourSensor(pin_S2, pin_S3, pin_OUT)
    self.colour_sensor.setup()

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
    motor_drive_speed_control_signal = self.default_motor_speed
    while distance_to_drive >= self.dist_threshold:
      self.drive(motor_drive_speed_control_signal, motor_drive_speed_control_signal + self.right_motor_offset)
      
      # # obstacle avoidance
      # if self.obstacle_detected and distance_to_drive > 20:
      #   self.obstacle_avoidance()

      # if self.wall_detected and distance_to_drive > 20: 
      #   self.wall_avoidance()

      distance_to_drive = self.distance_from_goal(goal)
      print("distance to drive: ", distance_to_drive)
      # print("prev distance: ", prev_distance_to_drive)
      
      if distance_to_drive < 5 * self.dist_threshold:
        motor_drive_speed_control_signal = 0.1
      
      if distance_to_drive > prev_distance_to_drive + 0.2: 
        print('drive overshot, stopping driving')
        break

      prev_distance_to_drive = distance_to_drive
      # print("dist error: ", distance_to_drive) 
      
      if self.wall_detected:
        self.wall_avoidance()
      
      if self.obstacle_detected:
        self.obstacle_avoidance()
    
    
    
    print("Drive complete")
    
    self.stop()
      

  def drive(self, left_motor_speed, right_motor_speed):
      self.set_left_motor_speed_pub.publish(left_motor_speed)
      self.set_right_motor_speed_pub.publish(right_motor_speed)
      rospy.sleep(0.1)

  def stop(self):
    while abs(round(self.left_motor_speed, 4)) > 0.0 or abs(round(self.right_motor_speed, 4)) > 0.0:
      self.drive(0.0,0.0)
    print("stopped")
      
  def turn(self, goal_angle, stop = True, motor_turn_speed_control_signal = 0.4):
    self.is_turning = True
    self.turning_pub.publish(self.is_turning)
    angle_error = abs(goal_angle - self.th)

    while angle_error > self.angle_threshold:
      if goal_angle > self.th:
        self.drive(-motor_turn_speed_control_signal, motor_turn_speed_control_signal + self.right_motor_offset)
        
      elif goal_angle < self.th:
        self.drive(motor_turn_speed_control_signal, -motor_turn_speed_control_signal - self.right_motor_offset)
        
      if angle_error < 12 * self.angle_threshold:
        motor_turn_speed_control_signal = 0.1

      angle_error = abs(goal_angle - self.th)
      self.turning_pub.publish(self.is_turning)
    
    print("Turn Complete")
    
    if stop:
      self.stop()
      print("Stopped Turning")
    
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
  
  # def determine_goal_location(self): 
  #   self.colour_sensor_trigger_pub(True) 
  #   colour_detected = False

  #   # waiting until new colour is detected 
  #   while not colour_detected: 
  #     print('Waiting for colour to be detected....')
  #     if self.package_colour != 'None': 
  #       print('Colour detected!')
  #       colour_detected = True 

  #   return self.colour_to_goal_location_map[self.package_colour]

  def detect_package(self):
    print('Waiting 1s before detecting package colour')
    rospy.sleep(2)
    print('Detecting package colour.....')
    colours = ['red', 'blue', 'green']
    # for _ in range(10):

      # rospy.sleep(0.05)
    # totalCount = 0
    # while totalCount < 5:
    redCount = 0
    blueCount = 0
    greenCount = 0
    
    for _ in range(20):
      package_colour = self.colour_sensor.detect_loop()
      if package_colour == "red":
        redCount += 1
      elif package_colour == "blue":
        blueCount += 1
      elif package_colour == "green":
        greenCount += 1
      rospy.sleep(0.05)
    colourCount = [redCount, blueCount, greenCount]
    # print("colour: ", colourCount.index(max(colourCount)))
    # totalCount += 1
    
    print(f'Package colour detected is {colours[colourCount.index(max(colourCount))]}')
    
    return colours[colourCount.index(max(colourCount))]
      
      
  def deliver_package(self): 
    
    print('Moving to delivery location')
    while self.front_left_sensor_dist > 7.5 or self.front_right_sensor_dist > 7.5: 
            self.drive(self.default_motor_speed, self.default_motor_speed + self.right_motor_offset)


    self.stop()
    
    self.align_dist_sensor_2()
    
    print('Delivering package')
    
    # for _ in range(3):
    #   self.servo.max()
    #   rospy.sleep(0.76)

    #   self.servo.min()
    #   rospy.sleep(0.88)
    
    rospy.sleep(1)

    print('Delivering package complete')
    
    print('Driving backwards from delivery location')
    while self.front_left_sensor_dist < 10 or self.front_right_sensor_dist < 10: 
            self.drive(-self.default_motor_speed,-self.default_motor_speed - self.right_motor_offset)
            
    self.stop()
    
    
  def align_dist_sensor(self): 
    prev_diff_distance_arr = []
    for _ in range(5):
      prev_diff_distance_arr.append(abs(self.front_left_sensor_dist - self.front_right_sensor_dist))
    prev_diff_distance = sum(prev_diff_distance_arr)/len(prev_diff_distance_arr)
    print(prev_diff_distance)
    if prev_diff_distance > 0.2:
      while prev_diff_distance > 0.01: 
        current_diff_distance = abs(self.front_left_sensor_dist - self.front_right_sensor_dist)
        print(f'current_diff_distance: {current_diff_distance}')
        print(f'prev_diff_distance: {prev_diff_distance}')

        # current_diff_distance = abs(self.front_left_sensor_dist - self.front_right_sensor_dist)

        print('aligning distance sensors')

        if self.front_left_sensor_dist > self.front_right_sensor_dist: 
          # turn right 
          angle_increments_to_turn = -np.pi/179
        else: 
          # turn left 
          angle_increments_to_turn = np.pi/179
      
        self.turn(angle_increments_to_turn + self.th, stop = True, motor_turn_speed_control_signal = 0.1)

        
        
        if current_diff_distance > prev_diff_distance + 0.4 and current_diff_distance < 5: 
          print('breaking')
          break

        prev_diff_distance = current_diff_distance
      
        rospy.sleep(0.06)
    
    self.stop()
    
  def align_dist_sensor_2(self): 
    self.stop()
    
    dist_diff = self.front_left_sensor_dist - self.front_right_sensor_dist
    prev_dist_diff = dist_diff
    # start alignment when sensors are 0.75 cm apart
    if abs(dist_diff) > 0.75: 
      print('Alinging robot agaisnt wall')
      while abs(dist_diff) > 0.3: 
        if self.front_left_sensor_dist > self.front_right_sensor_dist: 
            # turn right 
            angle_increments_to_turn = -np.pi/179
        else: 
            # turn left 
            angle_increments_to_turn = np.pi/179
            
        self.turn(angle_increments_to_turn + self.th, stop = False, motor_turn_speed_control_signal = 0.2)
        
        if dist_diff > prev_dist_diff + 0.3: 
          break
        
        prev_dist_diff = dist_diff
        dist_diff = self.front_left_sensor_dist - self.front_right_sensor_dist
        
        rospy.sleep(0.04)
        
      self.stop()
      
    
  def localise_dist_sensor(self): 
    # need to align directly to wall 
    # self.align_dist_sensor()

    y = self.max_arena_size - (self.front_right_sensor_dist + self.front_right_sensor_dist)/2 - self.robot_length/2

    print(f'y: {y}' )
    
    # turn to other wall 
    # if self.x > self.max_arena_size/2: 
    #   self.turn(0)
    # else: 
    #   self.turn(np.pi)


    # self.stop()

    # self.align_dist_sensor()

    # x = self.max_arena_size - (self.front_right_sensor_dist + self.front_right_sensor_dist)/2 - self.robot_length/2

    # print(f'x: {x}' )
    # self.turn(3*np.pi/2)

    # th = -np.pi/2

    # send_msg = Float32MultiArray()
    # send_msg.data = [x, y, th]
    # self.state_pub.publish(send_msg)
    
    # drive backwards 
    while self.front_left_sensor_dist < 10 or self.front_right_sensor_dist < 10: 
      self.drive(-self.default_motor_speed, -self.default_motor_speed - self.right_motor_offset)
    
    self.stop()

  
  def path_planning(self):
    while True:
      package_colour = self.detect_package()
      goal_location = self.colour_to_goal_location_map[package_colour]
      print(f'Goal location is {goal_location}')
      
      # Drive to goal location
      self.goal_location = goal_location
      self.drive_to_waypoint(self.goal_location)

      print("Goal reached!") 

      # Commence delivery
      self.deliver_package()
      
    
      
      # Relocalise
      # Using distance sensors, turn till straight
      # self.localise_dist_sensor()
      # If at delivery location A, turn 90 degrees to the left
      # Else if at delivery location C, turn 90 degrees to the right
      # Else if at delivery location B, turn 90 degrees in the direction the other robot is not in
      
      print("Relocalised")
      
      self.stop()
      
      print("Driving home")

      self.goal_location =self.start_location
      self.drive_to_waypoint(self.goal_location) 
        
      print("Back in loading zone")

  def obstacle_avoidance(self): 
    while self.obstacle_detected: 
      print('Robot detected')
      self.stop()

    print('robot avoided')
    
    self.drive_to_waypoint(self.goal_location)

  def wall_avoidance(self): 
    self.stop()
    while self.wall_detected: 
      print('Wall detected')

      if self.x > self.max_arena_size/2: # turn left slowly to avoid obstalce
          angle_increment_to_turn = np.pi/179

      else: # turn right slowl to avoid obstacle
          angle_increment_to_turn = -np.pi/179

      self.turn(angle_increment_to_turn + self.th, stop = False, motor_turn_speed_control_signal = 0.1)

    print('wall avoided')
  
    



if __name__ == "__main__":
  rospy.init_node('system')
  colour_to_goal_location_map = {'red': [30, 90], 'green': [30, 80], 'blue': [30,70]}
  # goal_locations = [[30, 70], [30, 70], [30, 70], [30, 70], [30, 70], [30, 70], [30, 70], [30, 70], [30, 70], [30, 70]]
  start_location = [30,30]
  robot = System(colour_to_goal_location_map, start_location)
  rospy.sleep(1)
  
  robot.path_planning()
  # robot.deliver_package()
  
  # robot.detect_package()
  # robot.drive_to_waypoint([105,70])
  # robot.drive_to_waypoint([30,90])
  # robot.drive(0.5, 0.5)
  # robot.turn(0)
  # robot.turn(0)
  # robot.turn(0)
  # robot.turn(3*np.pi/2)
  # robot.align_dist_sensor()
  
  # while not rospy.is_shutdown():
  #   robot.drive_straight()