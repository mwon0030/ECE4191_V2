from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
from time import sleep, time
import rospy
from std_msgs.msg import Float32, Bool
import json
import pandas as pd

import sys
print(sys.path)

class Encoder: 
    pulses_per_wheel_rotation = 75*12 # 75 turns of encoder is 1 rotation of wheel, 12 pulses per encoder rotation

    def __init__(self, pin_encoder_A, pin_encoder_B): 
        self.encoder = RotaryEncoder(a = pin_encoder_A, b = pin_encoder_B, max_steps = 0) 
        self.prev_time = time()
        self.prev_number_of_steps = 0
        self.weight = 0.8
        self.prev_speed = 0

    ## NOTE: need to add delay between consecutive readings
    def get_motor_speed(self): 
        current_number_of_steps = self.encoder.steps
        current_time = time()
        steps_per_second = (current_number_of_steps-self.prev_number_of_steps)/(current_time - self.prev_time)
        motor_speed = steps_per_second/Encoder.pulses_per_wheel_rotation

        self.prev_number_of_steps = current_number_of_steps
        self.prev_time = current_time
        
        new_speed = (1- self.weight)*motor_speed + self.weight*self.prev_speed

        self.prev_speed = new_speed
        
        sleep(0.07)
        
        return new_speed

class MotorControl: 
    def __init__(self, pin_PWM1, pin_PWM2, pin_EN, pin_encoder_A, pin_encoder_B): 
        self.motor_PWM1 = PWMOutputDevice(pin = pin_PWM1, initial_value = 0.0, frequency = 1000)
        self.motor_PWM2 = PWMOutputDevice(pin = pin_PWM2, initial_value = 0, frequency = 1000)
        self.motor_EN = DigitalOutputDevice(pin = pin_EN)

        self.encoder = Encoder(pin_encoder_A, pin_encoder_B)
        
    def enable_motor(self): 
        self.motor_EN.on()
        
    def disable_motor(self): 
        self.motor_EN.off()

    def set_motor_speed(self, ref_motor_speed):
        # max_iter = 1

        # for idx in range(max_iter):
        current_motor_speed = self.encoder.get_motor_speed()

        if ref_motor_speed > 0: 
            self.motor_PWM1.value = ref_motor_speed
            # self.motor_PWM1.value = self.pi_controller.update(ref_motor_speed, current_motor_speed, time())
            self.motor_PWM2.value = 0
        elif ref_motor_speed < 0: 
            self.motor_PWM1.value = 0
            self.motor_PWM2.value = -ref_motor_speed
            # self.motor_PWM2.value = self.pi_controller.update(-ref_motor_speed, -current_motor_speed, time())
        else: 
            self.motor_PWM1.value = 0
            self.motor_PWM2.value = 0

        print(f'Current Motor speed: {current_motor_speed} rev/s, Set Motor speed: {ref_motor_speed} rev/s')
        # sleep(0.05) 

    def get_current_motor_speed(self): 
        return self.encoder.get_motor_speed()
    

if __name__ == "__main__":
    pin_left_motor_PWM1 = 27
    pin_left_motor_PWM2 = 17
    pin_left_motor_EN = 22
    pin_left_motor_encoder_A = 9
    pin_left_motor_encoder_B = 10
    
    left_motor_control = MotorControl(pin_left_motor_PWM1, pin_left_motor_PWM2, pin_left_motor_EN, pin_left_motor_encoder_A, pin_left_motor_encoder_B)
    left_motor_control.enable_motor()

    pin_right_motor_PWM1 = 8
    pin_right_motor_PWM2 = 16
    pin_right_motor_EN = 7
    pin_right_motor_encoder_A = 26
    pin_right_motor_encoder_B = 25
    
    right_motor_control = MotorControl(pin_right_motor_PWM1, pin_right_motor_PWM2, pin_right_motor_EN, pin_right_motor_encoder_A, pin_right_motor_encoder_B)
    right_motor_control.enable_motor()


    # duty_cycles_to_test = [0.4, -0.4, 0.6, -0.6, 0.8, -0.8]
    duty_cycles_to_test = [0.15, -0.15]
    test_time = 10
    start_recording_time = 0
    
    left_motor_duty_cycle_to_avg_motor_speed = {}
    right_motor_duty_cycle_to_avg_motor_speed = {} 
    duty_cycle_to_avg_motor_speed = {}

    left_motor_duty_cycle_to_motor_speed = {}
    right_motor_duty_cycle_to_motor_speed = {} 
    
    

    for duty_cycle in duty_cycles_to_test:
        start_time = time()
        test_time_elapsed = 0

        left_motor_speeds = []
        right_motor_speeds = [] 

        left_motor_duty_cycle_to_motor_speed[duty_cycle] = []
        right_motor_duty_cycle_to_motor_speed[duty_cycle] = []



        while test_time_elapsed < test_time: 
            left_motor_control.set_motor_speed(duty_cycle)
            right_motor_control.set_motor_speed(duty_cycle)

            # letting motors reach steady state
            if test_time_elapsed > start_recording_time:
                left_motor_speeds.append(left_motor_control.get_current_motor_speed())
                right_motor_speeds.append(right_motor_control.get_current_motor_speed())
                
                left_motor_duty_cycle_to_motor_speed[duty_cycle].append(left_motor_speeds[-1])

                right_motor_duty_cycle_to_motor_speed[duty_cycle].append(right_motor_speeds[-1])

            test_time_elapsed = time() - start_time

        

        left_motor_duty_cycle_to_avg_motor_speed[duty_cycle] = sum(left_motor_speeds)/len(left_motor_speeds)
        right_motor_duty_cycle_to_avg_motor_speed[duty_cycle] = sum(right_motor_speeds)/len(right_motor_speeds)
        
        duty_cycle_to_avg_motor_speed[duty_cycle] = (sum(left_motor_speeds)/len(left_motor_speeds) + sum(right_motor_speeds)/len(right_motor_speeds))/2




    ### left motor
    left_motor_speed_df = pd.DataFrame(left_motor_duty_cycle_to_motor_speed)
    excel_file_path = "left_motor_duty_cycle_to_motor_speed.csv"
    left_motor_speed_df.to_csv(excel_file_path, index=False)

    file_name = "left_motor_duty_cycle_to_speeds.json"

    with open(file_name, "w") as json_file:
        json.dump(duty_cycle_to_avg_motor_speed, json_file)
        

    ### right motor
    right_motor_speed_dt = pd.DataFrame(right_motor_duty_cycle_to_motor_speed)
    excel_file_path = "right_motor_duty_cycle_to_motor_speed.csv"
    right_motor_speed_dt.to_csv(excel_file_path, index=False)

    file_name = "right_motor_duty_cycle_to_speeds.json"

    with open(file_name, "w") as json_file:
        json.dump(duty_cycle_to_avg_motor_speed, json_file)




    







