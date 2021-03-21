## script that starts the race
## made by Jon Skerlj


from pixyBot import pixyBot
from pixyCam import pixyCam
from robotRace_noObstacles import robotRace
from PIDcontroller import PID_controller

# exercises
def main():
    ### IMPORTANT
    servoCorrection = 0 # put the servo correction for your robot here
    ###

    r = pixyBot(servoCorrection, PID_controller(0.08, 0, 0.1))
    #r = pixyBot(servoCorrection, PID_controller(0.1, 0, 0.3))
    p = pixyCam()
    rR = robotRace(r, p)

    rR.race(0.8)
    #rR.test2angles()
main()