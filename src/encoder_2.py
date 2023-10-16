#! /usr/bin/env python

from gpiozero import RotaryEncoder
from time import time
import rospy
from std_msgs.msg import Float32

class Encoder: 
    pulses_per_wheel_rotation = 75*12 # 75 turns of encoder is 1 rotation of wheel, 12 pulses per encoder rotation

    def __init__(self, pin_encoder_A, pin_encoder_B, motor_name): 
        self.encoder = RotaryEncoder(a = pin_encoder_A, b = pin_encoder_B, max_steps = 0) 
        self.prev_time = time()
        self.prev_number_of_steps = 0
        self.encoder_pub = rospy.Publisher(motor_name, Float32, queue_size=1)

        self.prev_speed = 0
        self.weight = 0
        self.name = motor_name
        
    ## NOTE: need to add delay between consecutive readings
    def publish_motor_speed(self):
        current_number_of_steps = self.encoder.steps
        current_time = time()
        print(self.name, " change: ", current_number_of_steps-self.prev_number_of_steps)
        
        steps_per_second = (current_number_of_steps-self.prev_number_of_steps)/(current_time - self.prev_time)
        motor_speed = steps_per_second/Encoder.pulses_per_wheel_rotation

        self.prev_number_of_steps = current_number_of_steps
        self.prev_time = current_time
        self.encoder_pub.publish(motor_speed)
        
        
if __name__ == "__main__":
    rospy.init_node('encoder_2')
    
    print('Initialising right motor encoder')
    pin_right_motor_encoder_A = 26
    pin_right_motor_encoder_B = 25
    right_motor_name = 'right_motor'
    right_motor_encoder = Encoder(pin_right_motor_encoder_A, pin_right_motor_encoder_B, right_motor_name)
    
    while not rospy.is_shutdown():
        try:
            right_motor_encoder.publish_motor_speed()
            rospy.sleep(0.07)

        except rospy.ROSInterruptException:
            break