import RPi.GPIO as IO
import time

pwmPin = 12 #Jetson Nano PIN 32
dirPin = 6 #Jetson Nano PIN 31

#pwmPin = 19 #Jetson Nano PIN 35
#dirPin = 13 #Jetson Nano PIN 33


encPinA = 23 #Jetson Nano PIN 16
encPinB = 24 #Jetson Nano PIN 18

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin, IO.OUT)
IO.setup(dirPin, IO.OUT)

p = IO.PWM(12,100) #12pin , strength 20%
p.start(0)


encoderPos = 0

def encoderA(channel):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos += 1
    else:
        encoderPos -= 1
    #print('PinA: %d, encoder: %d' %(channel, encoderPos))


def encoderB(channel):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos -= 1
    else:
        encoderPos += 1
    #print('PinB: %d, encoder: %d' %(channel, encoderPos))

IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)   
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)

targetDeg = 360
ratio = 360/4096
Kp = 1000
Kd = 0
Ki = 0
dt = 0
dt_sleep = 0.01
tolerance = 0.01

start_time = time.time()
error_prev = 0
time_prev = 0

while True:
    motorDeg = encoderPos * ratio

    error = targetDeg - motorDeg
    de = error - error_prev
    dt = time.time() - time_prev
    control = Kp*error + Kd*de/dt + Ki*error*dt

    error_prev = error
    time_prev = time.time()

    IO.output(dirPin, control >= 0)
    p.ChangeDutyCycle(min(abs(control),100))

    print("P-term = %7.1f, D-term = %7.1f, I-term = %7.1f" %(Kp*error, Kd*de/dt, Ki*de*dt))
    print("time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f" %(time.time()-start_time, encoderPos, motorDeg, error, control))
    print("%f, %f" %(de, dt))

    if abs(error) <= tolerance:
        IO.output(dirPin, control >= 0)
        p.ChangeDutyCycle(0)
        break

time.sleep(dt_sleep)
