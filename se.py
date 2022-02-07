import pygame
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

ENA = 36
IN1 = 38
IN2 = 40

STOP = 0
FORWARD = 1
BACKWARD = 2

CH1 = 0


OUTPUT = 1
INPUT = 0

H = 1
L = 0

def setPinConfig(EN,INA,INB):
    GPIO.setwarnings(False)
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setwarnings(False)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setwarnings(False)
    GPIO.setup(INB, GPIO.OUT)
    
    pwm = GPIO.PWM(EN,1000)
    pwm.start(0)
    return pwm


def setMotorControl(pwm, INA, INB, speed, state):
    
    pwm.ChangeDutyCycle(speed)
    
    
    if state == FORWARD:
        GPIO.output(INA, H)
        GPIO.output(INB, L)
        
    elif state == BACKWARD:
        GPIO.output(INA, L)
        GPIO.output(INB, H)
        
    elif state == STOP:
        GPIO.output(INA, L)
        GPIO.output(INB, L)
        
        
def setMotor(ch, speed, state):
    if ch == CH1:
        setMotorControl(pwmA, IN1, IN2, speed, state)
    

pwmA = setPinConfig(ENA, IN1, IN2)





servoPin = 12
SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3


GPIO.setup(servoPin, GPIO.OUT)

servo = GPIO.PWM(servoPin, 58)
servo.start(0)

def setServoPos(degree):
    """if degree>180:
        degree=180"""
    
    if degree > 180:
        degree = 180
    
        
    duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    
    servo.ChangeDutyCycle(duty)
    
def degree_converter_UtoServo(U):
   
    degree = -0.4167*U+95
    return degree    

pygame.init()
screen = pygame.display.set_mode((400,400))
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            
        key = pygame.key.get_pressed()
        if key[pygame.K_w]:
            setMotor(CH1, 100, FORWARD)
            print("go forward")
        if key[pygame.K_s]:
            setMotor(CH1, 100, BACKWARD)
            print("go backward")
        if key[pygame.K_a]:
            print("turn left")
            U = 90
            degree = degree_converter_UtoServo(U)
            setServoPos(degree)
        if key[pygame.K_d]:
            print("turn right")
            U = -90
            degree = degree_converter_UtoServo(U)
            setServoPos(degree)
        if key[pygame.K_w]==0 and key[pygame.K_s]==0:
            setMotor(CH1, 0, STOP)
            print("stop")
        if key[pygame.K_a]==0 and key[pygame.K_d]==0:
            U=0
            degree = degree_converter_UtoServo(U)
            setServoPos(degree)
            print("straight")
    
  
servo.stop()
setMotor(CH2, 0, STOP)
GPIO.cleanup()
