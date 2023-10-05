import RPi.GPIO as GPIO
import time
from src.collision import Collision
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
        self.maxRead = 10
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
        if self.start_detecting: 
            detect_colour = False
            redArr, greenArr, blueArr = [], [], []
            while not detect_colour:  

                # set pins to read red
                GPIO.output(self.pin_S2,GPIO.LOW)
                GPIO.output(self.pin_S3,GPIO.LOW)
                time.sleep(0.3) #don't know if we need delay

                red  = self.find_frequency()   
            
                #set pins to read blue
                GPIO.output(self.pin_S2,GPIO.LOW)
                GPIO.output(self.pin_S3,GPIO.HIGH)
                time.sleep(0.3)


                blue = self.find_frequency()
                
                #set pins to read green
                GPIO.output(self.pin_S2,GPIO.HIGH)
                GPIO.output(self.pin_S3,GPIO.HIGH)
                time.sleep(0.3)

                green = self.find_frequency()

                self.readCount += 1 #add one to count of frequencies 

                #append freq values to array
                redArr.append(red)
                blueArr.append(blue)
                greenArr.append(green)

                #update moving average of readings
                redAvg = sum(redArr)/len(redArr)
                blueAvg = sum(blueArr)/len(blueArr)
                greenAvg = sum(greenArr)/len(greenArr)
                
                #find differences between all values
                minDiff = min([abs(redAvg-blueAvg),abs(redAvg - greenAvg),abs(blueAvg-greenAvg)])

                if (self.readCount > self.maxRead and minDiff < self.diffThreshold):
                    detect_colour = True
            
            detected_colour = ColourSensor.colours.index([redAvg, blueAvg, greenAvg].index(min([redAvg, blueAvg, greenAvg])))
            self.package_colour_pub.publish(detect_colour)
            return detected_colour
        else: 
            self.package_colour_pub.publish(None)
            return None 
    
if __name__=='__main__':
    
    pin_S2 = 1 
    pin_S3 = 2
    pin_OUT = 25

    colour_sensor = ColourSensor(pin_S2, pin_S3, pin_OUT)

    colour_sensor.setup()

    while not rospy.is_shutdown():
        try:
            colour_sensor.detect_loop()
        except rospy.ROSInterruptException:
            break
