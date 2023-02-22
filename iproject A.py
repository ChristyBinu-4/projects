#Importing libraries
import RPi.GPIjO as GPIO
import time

#Definition of motor pins
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
PWM_A = 16
PWM_B = 13

#Definition of start key
key = 8

#Definition of line tracker pins
TrackIRLeftPin1 = 3
TrackIRLeftPin2 = 5
TrackIRRightPin1 = 4 
TrackIRRightPin2 = 18

#Definition of ultrasonic sensor pin
EchoPin = 0
TrigPin = 1

#Definition of obstacle avoidance sensor pins
AvoidSensorLeft = 12
AvoidSensorRight = 17

#Definition of servo motor pin
ServoPin = 23

#Definition of RGB module pin
LED_R = 22
LED_G = 27
LED_B = 24

#Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Variable for detecting number of times of right-angled corners
count = 0

def init():
    global pwm_left
    global pwm_right
    global pwm_servo
    global pwm_rled
    global pwm_gled
    global pwm_bled
    GPIO.setup(PWM_A, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PWM_B, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    
    GPIO.setup(AvoidSensorLeft, GPIO.IN)
    GPIO.setup(AvoidSensorRight, GPIO.IN)
    
    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    GPIO.setup(TrackIRLeftPin1, GPIO.IN)
    GPIO.setup(TrackIRLeftPin2, GPIO.IN)
    GPIO.setup(TrackIRRightPin1, GPIO.IN)
    GPIO.setup(TrackIRRightPin2, GPIO.IN)
    #Set the PWM pin and frequency is 2000Hz
    pwm_left = GPIO.PWM(PWM_A, 2000)
    pwm_right = GPIO.PWM(PWM_B, 2000)
    pwm_left.start(0)     # where 0 is the duty cycle
    pwm_right.start(0)   # where 0 is the duty cycle
    pwm_rled = GPIO.PWM(LED_R, 1000)
    pwm_gled = GPIO.PWM(LED_G, 1000)
    pwm_bled = GPIO.PWM(LED_B, 1000)
    pwm_rled.start(0)
    pwm_gled.start(0)
    pwm_bled.start(0)
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0) 
    
#Advance
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(leftspeed)
    pwm_right.ChangeDutyCycle(rightspeed)

#Reverse
def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(leftspeed)
    pwm_right.ChangeDutyCycle(rightspeed)
    
#Turn left 
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(leftspeed)
    pwm_right.ChangeDutyCycle(rightspeed)

#Turn right
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(leftspeed)
    pwm_right.ChangeDutyCycle(rightspeed)
    
#Turn left in place
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(leftspeed)
    pwm_right.ChangeDutyCycle(rightspeed)

#Turn right in place
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(leftspeed)
    pwm_right.ChangeDutyCycle(rightspeed)

#Brake
def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

#Button detection
def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
            while not GPIO.input(key):
                pass
            
#Servo angle
def servo_angle(pos):
    pwm_servo.ChangeDutyCycle(2.5 +10 * pos/180)
    time.sleep(0.005)


#Ultrasonic Sensor 
def distance_sensing(): 
        GPIO.output(TrigPin,GPIO.HIGH)            
        time.sleep(0.000015)
        GPIO.output(TrigPin,GPIO.LOW)                  
        while not GPIO.input(EchoPin):                    
            pass
        t1 = time.time()                                                
        while GPIO.input(EchoPin):                            
            pass
        t2 = time.time()                                                 
        cal_distance = int(((t2-t1) * 340/2) *100)    
        print(f"Distance is {cal_distance}.")
        print(f"Count = {count}")
        time.sleep(0.001)
        return cal_distance                                          

#RGB light control
def color_led_pwm(iRed, iGreen, iBlue):
    v_red = (100*iRed)/255
    v_green = (100*iGreen)/255
    v_blue = (100*iBlue)/255
    pwm_rled.ChangeDutyCycle(v_red)
    pwm_gled.ChangeDutyCycle(v_green)
    pwm_bled.ChangeDutyCycle(v_blue)
    time.sleep(0.02)

time.sleep(2)

try:
    init() 
    key_scan()
    while True:
        color_led_pwm(0,255,0) #Emit green LED – Idle mode
        servo_angle(90) #Ensure servo is facing front #User press start button to initiate movement
        color_led_pwm(0,0,255) #Emit blue LED – Operation mode
        distance = distance_sensing() #Constantly check distance ahead
        
        LeftSensorValue  = GPIO.input(AvoidSensorLeft);
        RightSensorValue = GPIO.input(AvoidSensorRight);
        while (count < 4):
            color_led_pwm(0,0,255) #Emit blue LED – Operation mode
            distance = distance_sensing() #Constantly check distance ahead
            if distance > 100:
                if LeftSensorValue == True and RightSensorValue == True:
                    TrackSensorLeftValue1  = GPIO.input(TrackIRLeftPin1)
                    TrackSensorLeftValue2  = GPIO.input(TrackIRLeftPin2)
                    TrackSensorRightValue1 = GPIO.input(TrackIRRightPin1)
                    TrackSensorRightValue2 = GPIO.input(TrackIRRightPin2)

        #Straight track, leftspeed = 35, rightspeed = 35
                    if TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False:
                        run(20,20)

        #Align left, leftspeed = 0, rightspeed = 35
                    elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == True:
                        left(0,20)

        #Align right, leftspeed = 35, rightspeed = 0
                    elif TrackSensorLeftValue2 == True and TrackSensorRightValue1 == False:
                        right(20,0)

        #Slide left, leftspeed = 35, rightspeed = 35
                    elif TrackSensorLeftValue1 == False:
                        spin_left(20,20)

        #Slide right, leftspeed = 35, rightspeed = 35
                    elif TrackSensorRightValue2 == False:
                        spin_right(20,20)

        #Turn left in place, with delay 50ms, leftspeed = 35, rightspeed = 35
                    elif TrackSensorLeftValue1 == False and (TrackSensorRightValue1 == False or TrackSensorRightValue2 == False):
                        spin_left(20,20)
                        time.sleep(0.05)

        #Turn right in place, with delay 50ms, leftspeed = 35, rightspeed = 35
                    elif (TrackSensorLeftValue1 == False or TrackSensorLeftValue2 == False) and TrackSensorRightValue2 == False:
                        spin_right(20,20)
                        time.sleep(0.05)
            
            
   

        
        #Obstacle on right, leftspeed = 20, rightspeed = 20     
                elif LeftSensorValue == True and RightSensorValue == False:
                    spin_left(20,20)   
                    time.sleep(0.002)
        #Obstacle on left, leftspeed = 20, rightspeed = 20     
                elif RightSensorValue == True and LeftSensorValue == False:
                    spin_right(20,20)  
                    time.sleep(0.002)
        #Obstacle ahead		
                elif RightSensorValue == False and LeftSensorValue == False:
                    brake() #Runs at normal speed
            elif (distance <= 100) and (distance >=30):
                color_led_pwm(255,0,0) #Flash red LED 
                time.sleep(0.25)
                color_led_pwm(0,0,0)
                time.sleep(0.25)
                if LeftSensorValue == True and RightSensorValue == True:
                    TrackSensorLeftValue1  = GPIO.input(TrackIRLeftPin1)
                    TrackSensorLeftValue2  = GPIO.input(TrackIRLeftPin2)
                    TrackSensorRightValue1 = GPIO.input(TrackIRRightPin1)
                    TrackSensorRightValue2 = GPIO.input(TrackIRRightPin2)

        #Straight track, leftspeed = 35, rightspeed = 35
                    if TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False:
                        run(10,10)

        #Align left, leftspeed = 0, rightspeed = 35
                    elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == True:
                        left(0,10)

        #Align right, leftspeed = 35, rightspeed = 0
                    elif TrackSensorLeftValue2 == True and TrackSensorRightValue1 == False:
                        right(10,0)

        #Slide left, leftspeed = 35, rightspeed = 35
                    elif TrackSensorLeftValue1 == False:
                        spin_left(10,10)

        #Slide right, leftspeed = 35, rightspeed = 35
                    elif TrackSensorRightValue2 == False:
                        spin_right(10,10)

        #Turn left in place, with delay 50ms, leftspeed = 35, rightspeed = 35
                    elif TrackSensorLeftValue1 == False and (TrackSensorRightValue1 == False or TrackSensorRightValue2 == False):
                        spin_left(10,10)
                        time.sleep(0.05)

        #Turn right in place, with delay 50ms, leftspeed = 35, rightspeed = 35
                    elif (TrackSensorLeftValue1 == False or TrackSensorLeftValue2 == False) and TrackSensorRightValue2 == False:
                        spin_right(10,10)
                        time.sleep(0.05)
            
            
   

        
        #Obstacle on right, leftspeed = 20, rightspeed = 20     
                elif LeftSensorValue == True and RightSensorValue == False:
                    spin_left(10,10)   
                    time.sleep(0.002)
        #Obstacle on left, leftspeed = 20, rightspeed = 20     
                elif RightSensorValue == True and LeftSensorValue == False:
                    spin_right(10,10)  
                    time.sleep(0.002)
        #Obstacle ahead		
                elif RightSensorValue == False and LeftSensorValue == False:
                    brake() #Reduced speed
            elif distance < 30: 
                color_led_pwm(255,0,0) #Emit red LED – Stop mode
                brake()
                servo_angle(180) #Face ultrasonic to the left
                time.sleep(0.5)
                distanceleft = distance_sensing() #Check instantaneous distance of left side
                if distanceleft < 30: #Right-angled wall detected
                    servo_angle(90) #Face servo back to the front
                    spin_right(12,12) #Turn 90 degree to the right
                    time.sleep(0.6)
                    count = count + 1 #Count number of 4th corners encountered
                else: #No right-angled corner found; robot assume a person is in front of robot
                    servo_angle(90)
                    time.sleep(0.5)
        
        count = 0

except KeyboardInterrupt:
    pass
pwm_left.stop()
pwm_right.stop()
pwm_rled.stop()
pwm_gled.stop()
pwm_bled.stop()
pwm_servo.stop()
GPIO.cleanup()

