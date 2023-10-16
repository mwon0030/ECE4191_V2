#! /usr/bin/env python

import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float32, Bool, Float32MultiArray, String
import rospy


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

        self.colour_sensor_trigger_sub = rospy.Subscriber('/colour_sensor_trigger', Bool, self.colour_sensor_trigger_cb)

        self.package_colour_pub = rospy.Publisher('package_colour', String, queue_size=1)
    
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
        
        self.package_colour_pub.publish(package_colour)
        
        time.sleep(0.05)
    
if __name__=='__main__':
    rospy.init_node('colour_detect')
    
    pin_S2 = 6 
    pin_S3 = 13
    pin_OUT = 5

    colour_sensor = ColourSensor(pin_S2, pin_S3, pin_OUT)

    colour_sensor.setup()

    while not rospy.is_shutdown():
        try:
            if colour_sensor.start_detecting:
                colour_sensor.detect_loop()
        except rospy.ROSInterruptException:
            break
