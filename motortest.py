import RPi.GPIO as io
import time

io.setmode(io.BCM)
io.setwarnings(False)

PWM_MAX = 100

######## BT7690 ##################
# LEFT MOTOR
L_L_EN = 21 # leftmotor_in1_pin
L_R_EN = 20 # leftmotor_in2_pin
L_L_PWM = 12 # leftmotorpwm_pin_l
L_R_PWM = 16 # leftmotorpwm_pin_r

# RIGHT MOTOR
R_L_EN = 26 # rightmotor_in1_pin
R_R_EN = 19 # rightmotor_in2_pin
R_L_PWM = 13 # rightmotorpwm_pin_l
R_R_PWM = 6 # rightmotorpwm_pin_r

leftmotor_in1_pin = L_L_EN
leftmotor_in2_pin = L_R_EN
rightmotor_in1_pin = R_L_EN
rightmotor_in2_pin = R_R_EN

io.setup(leftmotor_in1_pin, io.OUT)
io.setup(leftmotor_in2_pin, io.OUT)
io.setup(rightmotor_in1_pin, io.OUT)
io.setup(rightmotor_in2_pin, io.OUT)

io.output(leftmotor_in1_pin, True)
io.output(leftmotor_in2_pin, True)
io.output(rightmotor_in1_pin, True)
io.output(rightmotor_in2_pin, True)

leftmotorpwm_pin_l = L_L_PWM 
leftmotorpwm_pin_r = L_R_PWM
rightmotorpwm_pin_l = R_L_PWM
rightmotorpwm_pin_r = R_R_PWM

io.setup(leftmotorpwm_pin_l, io.OUT)
io.setup(leftmotorpwm_pin_r, io.OUT)
io.setup(rightmotorpwm_pin_l, io.OUT)
io.setup(rightmotorpwm_pin_r, io.OUT)

leftmotorpwm_l = io.PWM(leftmotorpwm_pin_l,100)
leftmotorpwm_r = io.PWM(leftmotorpwm_pin_r,100)
rightmotorpwm_l = io.PWM(rightmotorpwm_pin_l,100)
rightmotorpwm_r = io.PWM(rightmotorpwm_pin_r,100)

leftmotorpwm_l.start(0)
leftmotorpwm_r.start(0)
rightmotorpwm_l.start(0)
rightmotorpwm_r.start(0)

leftmotorpwm_l.ChangeDutyCycle(0)
leftmotorpwm_r.ChangeDutyCycle(0)
rightmotorpwm_l.ChangeDutyCycle(0)
rightmotorpwm_r.ChangeDutyCycle(0)
############################################

def setMotorLeft(power):
   int(power)
   if power < 0:
      # Rueckwaertsmodus fuer den linken Motor
      #setMotorMode("leftmotor", "reverse")
      pwm = -int(PWM_MAX * power)
      if pwm > PWM_MAX:
         pwm = PWM_MAX
      leftmotorpwm_l.ChangeDutyCycle(pwm)
      leftmotorpwm_r.ChangeDutyCycle(0)	  
   elif power > 0:
      # Vorwaertsmodus fuer den linken Motor
      #setMotorMode("leftmotor", "forward")
      pwm = int(PWM_MAX * power)
      if pwm > PWM_MAX:
         pwm = PWM_MAX
      leftmotorpwm_l.ChangeDutyCycle(0)
      leftmotorpwm_r.ChangeDutyCycle(pwm)
   else:
      # Stoppmodus fuer den linken Motor
      leftmotorpwm_l.ChangeDutyCycle(0)
      leftmotorpwm_r.ChangeDutyCycle(0)

def setMotorRight(power):
   int(power)
   if power < 0:
      # Rueckwaertsmodus fuer den rechten Motor
      #setMotorMode("rightmotor", "reverse")
      pwm = -int(PWM_MAX * power)
      if pwm > PWM_MAX:
         pwm = PWM_MAX
      rightmotorpwm_l.ChangeDutyCycle(pwm)
      rightmotorpwm_r.ChangeDutyCycle(0)
   elif power > 0:
      # Vorwaertsmodus fuer den rechten Motor
      #setMotorMode("rightmotor", "forward")
      pwm = int(PWM_MAX * power)
      if pwm > PWM_MAX:
         pwm = PWM_MAX
      rightmotorpwm_l.ChangeDutyCycle(0)
      rightmotorpwm_r.ChangeDutyCycle(pwm)
   else:
      # Stoppmodus fuer den rechten Motor
      rightmotorpwm_l.ChangeDutyCycle(0)
      rightmotorpwm_r.ChangeDutyCycle(0)

def forward():
   setMotorLeft(1)
   setMotorRight(1)
def backward():
   setMotorLeft(-1)
   setMotorRight(-1)
def left():
   setMotorLeft(-1)
   setMotorRight(1)
def right():
   setMotorLeft(1)
   setMotorRight(-1)

try:
    while True:
        forward()

except KeyboardInterrupt:
    rpwm.stop()
    lpwm.stop()
    io.cleanup()
