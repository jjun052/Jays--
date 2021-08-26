import RPi.GPIO as IO
import time

pwmLPin = 12 #Jetson Nano PIN 32
pwmRPin = 21
dirLPin = 6 #Jetson Nano PIN 31
dirRPin = 13
#pwmPin = 19 #Jetson Nano PIN 35
#dirPin = 13 #Jetson Nano PIN 33

encLPinA = 23 #Jetson Nano PIN 16
encLPinB = 24 #Jetson Nano PIN 18
encRPinA = 11
encRPinB = 12

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encLPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encLPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup([pwmLPin,pwmRPin,dirLPin,dirRPin], IO.OUT)


pL = IO.PWM(pwmLPin,100) #12pin , strength 20%
pL.start(0)

pR = IO.PWM(pwmLPin,100) #12pin , strength 20%
pR.start(0)

encoderLPos = 0

def encoderLA(channel):
    global encoderLPos
    if IO.input(encLPinA) == IO.input(encRPinB):
        encoderLPos += 1
    else:
        encoderLPos -= 1
    #print('PinA: %d, encoder: %d' %(channel, encoderPos))


def encoderLB(channel):
    global encoderLPos
    if IO.input(encLPinA) == IO.input(encRPinB):
        encoderLPos -= 1
    else:
        encoderLPos += 1
    #print('PinB: %d, encoder: %d' %(channel, encoderPos))

def encoderRA(channel):
    global encoderRPos
    if IO.input(encLPinA) == IO.input(encRPinB):
        encoderRPos += 1
    else:
        encoderRPos -= 1
    #print('PinA: %d, encoder: %d' %(channel, encoderPos))


def encoderRB(channel):
    global encoderRPos
    if IO.input(encLPinA) == IO.input(encRPinB):
        encoderRPos -= 1
    else:
        encoderRPos += 1
    #print('PinB: %d, encoder: %d' %(channel, encoderPos))

IO.add_event_detect(encLPinA, IO.BOTH, callback=encoderLA)
IO.add_event_detect(encLPinB, IO.BOTH, callback=encoderLB)

IO.add_event_detect(encRPinA, IO.BOTH, callback=encoderRA)
IO.add_event_detect(encRPinB, IO.BOTH, callback=encoderRB)

# targetLDeg = 360
targetLSpd = 10

# targetRDeg = 360
targetRSpd = 10

ratio = 360/4096/3
Kp = 1000
Kd = 0
Ki = 0
dt = 0
dt_sleep = 0.01
tolerance = 0.01

start_time = time.time()
errL_prev = 0
time_prev = 0

while True:
    motorLDeg = encoderLPos * ratio

    spdLrpm = encoderLPos * 60 * 1000/dt_sleep * 1/4096
    errL = targetLspd - spdLrpm
    derrL = errL -errL_prev
    dtL = time.time() - time_prev
    controlL = Kp*errL + Kd*derrL/dt + Ki*errL*dtL

    # errorL = targetLDeg - motorLDeg
    # de = errorL - error_prev
    # dt = time.time() - time_prev
    # controlL = Kp*error + Kd*de/dt + Ki*error*dt

    # errorL_prev = errorL
    # time_prev = time.time()

    IO.output(dirLPin, control >= 0)
    pL.ChangeDutyCycle(min(abs(controlL),100))

    print("P-term = %7.1f, D-term = %7.1f, I-term = %7.1f" %(Kp*errL, Kd*derrL/dt, Ki*errL*dtL))
    print("time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f" %(time.time()-start_time, encoderPos, motorDeg, error, control))
    print("%f, %f" %(derrL, dtL))

    if abs(errL) <= tolerance:
        IO.output(dirLPin, control >= 0)
        pL.ChangeDutyCycle(0)
        break

errR_prev = 0

while True:

    motorRDeg = encoderRPos * ratio

    spdRrpm = encoderRPos * 60 * 1000/dt_sleep * 1/4096
    errR = targetRspd - spdRrpm
    derrR = errR -errR_prev
    dtL = time.time() - time_prev
    controlR = Kp*errR + Kd*derrR/dt + Ki*errR*dtR

    # errorL = targetLDeg - motorLDeg
    # de = errorL - error_prev
    # dt = time.time() - time_prev
    # controlL = Kp*error + Kd*de/dt + Ki*error*dt

    # errorL_prev = errorL
    # time_prev = time.time()

    IO.output(dirRPin, control >= 0)
    pR.ChangeDutyCycle(min(abs(controlR),100))

    print("P-term = %7.1f, D-term = %7.1f, I-term = %7.1f" %(Kp*errR, Kd*derrR/dt, Ki*errR*dtR))
    print("time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f" %(time.time()-start_time, encoderRPos, motorRDeg, errR, controlR))
    print("%f, %f" %(derrL, dtL))

    if abs(errR) <= tolerance:
        IO.output(dirRPin, control >= 0)
        pR.ChangeDutyCycle(0)
        break

time.sleep(dt_sleep)
