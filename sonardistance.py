import RPi.GPIO as GP     #Jetson nano GPIO Change
import time

GP.setmode(GP.BCM)

TRIG = 23      #PIN 16 jetson nano
ECHO = 24      #PIN 18 jetson nano

print("Distance Measurement In Progress")

GP.setup(TRIG,GP.OUT)
GP.setup(ECHO,GP.IN)


def distance():

    GP.output(TRIG, True)
    time.sleep(0.00001)
    GP.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GP.input(ECHO) == 0:
        pulse_start = time.time()
        #print("ECHO is 0!!!")


    while GP.input(ECHO) == 1:
        pulse_end = time.time()
        #print("ECHO is 1!!!")
        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2)

        #print("Distance:", distance, "cm")

        #GP.cleanup()
        
        return distance

if __name__ == '__main__':
    try:
        while True:
            dist = distance()
            print("MEasured Distance = ", dist, "cm")
            time.sleep(1)
        
    except KeyboardInterrupt:
        print("Measurement stopped by user")
        GP.cleanup()
