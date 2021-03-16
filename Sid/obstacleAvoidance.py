## library for obstacle avoidance and lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2020

from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
import math

# obstacleAvoidance class: has a bunch of convinient functions for navigation
#
# input arguments
#   bot                         - (optional) pixyBot object
#   cam                         - (optional) pixyCam object
#
# attributes - feel free to add more!
#   bot                         - pixyBot object
#   cam                         - pixyCam object
#   IDs                         - signature ID number for various colour blocks
#   frameTimes                  - list of frameTimes for blockDistance and blockAngle
#   blockDistance                   - list of angular size of queried block over frameTimes in ([deg])
#   blockAngle                  - list of angular error of queried block over frameTimes in ([deg])
#
# methods
#   drive                       - differential drive function that takes bias for the right wheel speed
#   getBlockParams              - get and store the size and anglular error of a selected block
#   visTrack                    - move camera to point at selected block
#   stopAtStationaryObstacles   - move bot along a lane until it encounters an obstacle
#   avoidStationaryObstacles    - move bot along a lane and move to avoid stationary obstacles
class obstacleAvoidance(object):
    def __init__(self, bot=pixyBot(0), cam=pixyCam()):
        self.bot = bot

        self.bot = bot
        self.cam = cam

        self.centerLineID   = 1
        self.leftLineID     = 2
        self.rightLineID    = 3
        self.obstacleID     = 4

        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockDistance = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]

        self.obstacleDistance = 30
        self.obstacleAvoidDistance = 18
        self.angleOrientation = -20
        self.driveLevel = 0.3
        self.focalLength = 315
    # output            - none
    # drive             - desired general bot speed (-1~1)
    # bias              - ratio of drive speed that is used to turn right (-1~1)
    def drive(self, drive, bias): # Differential drive function
        if bias > 1:
            bias = 1
        if bias < -1:
            bias = -1

        maxDrive = 1 # set safety limit for the motors

        totalDrive = drive * maxDrive # the throttle of the car
        diffDrive = bias * totalDrive # set how much throttle goes to steering
        straightDrive = totalDrive - abs(diffDrive) # the rest for driving forward (or backward)

        lDrive = straightDrive + diffDrive
        rDrive = straightDrive - diffDrive
        self.bot.setMotorSpeeds(lDrive, rDrive)

    # output            - -1 if error
    # blockIdx          - index of block in block list that you want to save parameters for
    def getBlockParams(self, blockIdx):
        if (self.cam.newCount-1) < blockIdx or blockIdx < 0: # do nothing when block doesn't exist
            return -1
        else:
            pixelSize = self.cam.newBlocks[blockIdx].m_width
            angleSize = pixelSize/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular size of block
            distance = (self.focalLength*4.7)/pixelSize
            pixelError = self.cam.newBlocks[blockIdx].m_x -  self.cam.pixyCenterX
            angleError = pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular error of block relative to front

            # save params
            self.blockDistance.append(distance)
            self.blockAngle.append(angleError)
            self.frameTimes.append(time())

            # remove oldest params
            self.blockDistance.pop(0)
            self.blockAngle.pop(0)
            self.frameTimes.pop(0)

    # output            - tracking error in ([deg]) and new camera angle in ([deg]) (tuple), or -1 if error
    # blockIdx          - index of block in block list that you want to track with the camera
    def visTrack(self, blockIdx): # Get pixycam to rotate to track an object
        if blockIdx < 0: # do nothing when block doesn't exist
            self.bot.setServoPosition(0)
            return 0, 0
        else:
            pixelError =  self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX # error in pixels
            visAngularError = -(pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV) # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition

    # output            - none
    # speed             - general speed of bot
    def stopAtStationaryObstacles(self, speed):
        self.bot.setServoPosition(0) # set servo to centre
        #self.drive(0, 0) # set racer to stop
        servoPos = float('nan')
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            
            obstacleBlock = self.cam.isInView(self.obstacleID) # try find obstacle
            #print(centerLineBlock,leftLineBlock,rightLineBlock,obstacleBlock)
            stop = False
            objectClose = False
            
            if obstacleBlock >= 0: #if obstacle exists
                self.getBlockParams(obstacleBlock)
                print("angle: {:.2f} size: {:.2f}".format(self.blockAngle[-1],self.blockDistance[-1]))                    
                
                if self.blockDistance[-1] < self.obstacleDistance:#if obstacle is close enough 
                    
                    objectClose = True
                    servoError,servoPos = self.visTrack(obstacleBlock) #start tracking obstacle instead of line 

                    print("servo: ",servoPos,"servo error",servoError,"distance: ",self.blockDistance[-1])

                    if servoPos < self.angleOrientation -10 :   #maybe get rid of the stop as loop breaks anyways 
                        print("engaged left")
                        self.drive(0.1,1)
                    elif servoPos > self.angleOrientation + 5:
                        print("engaged right")
                        self.drive(0.1,-1)
                    else:
                        if abs(servoError) < 10:

                            print("stop = true")
                            stop = True
                            self.drive(0, 0)    
                            break
                    continue #go to next loop instead of rest of the script 
                else:
                    objectClose = False
            
            if not objectClose: #if object is not close enough, follow line
                stop = False
                print("stop = false")

                if centerLineBlock >= 0: # drive while we see a line            
                    _,servoPos = self.visTrack(centerLineBlock)
                    print(servoPos)
                    
                    if servoPos > 10:
                        self.drive(self.driveLevel,-0.2)
                    elif servoPos < -10:
                        self.drive(self.driveLevel,0.2)
                    else:
                        self.drive(self.driveLevel,0)

                elif leftLineBlock >= 0:

                    print("left")
                    self.drive(0.2,0.2)

                elif rightLineBlock >= 0:
                    
                    print("right")
                    self.drive(0.2,-0.2)

                else: # stop the racer and wait for new blocks 
                    self.drive(0, 0)
        print("ended")
        return

    # output            - none
    # speed             - general speed of bot
    def avoidStationaryObstacles(self, speed):
        scan = 0
        scanFreq = 50
        self.bot.setServoPosition(0) # set servo to centre
        #self.drive(0, 0) # set racer to stop
        servoPos = float('nan')

        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            #centerLineBlock = -1
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            
            obstacleBlock = -1 #self.cam.isInView(self.obstacleID) # try find obstacle
            #print(centerLineBlock,leftLineBlock,rightLineBlock,obstacleBlock)
            
            objectClose = False
            
            if obstacleBlock >= 0: #if obstacle exists
                self.getBlockParams(obstacleBlock)
                print("obstacle detected, angle: {:.2f} size: {:.2f}".format(self.blockAngle[-1],self.blockDistance[-1]))                    
                
                if self.blockDistance[-1] < self.obstacleAvoidDistance:#if obstacle is close enough 
                    
                    objectClose = True
                    servoError,servoPos = self.visTrack(obstacleBlock) #start tracking obstacle instead of line 

                    print("Obstacle close, servo: ",servoPos,"servo error",servoError,"distance: ",self.blockDistance[-1])

                    if servoPos > 0: #obstacle on the left must move righ t
                        self.drive(0.2,0.2)
                    
                    elif servoPos < 0:
                        self.drive(0.2,-0.2) 
                    continue #go to next loop instead of rest of the script 
                else:
                    objectClose = False
            
            if not objectClose: #if object is not close enough, follow line
                

                if centerLineBlock >= 0: # drive while we see a line            
                    _,servoPos = self.visTrack(centerLineBlock)
                    turnRate = (-0.5/50)*servoPos
                    print("turn rate",turnRate,"servo pos",servoPos)
                    
                    if servoPos > 5:
                        self.drive(self.driveLevel,turnRate)
                    elif servoPos < -5:
                        self.drive(self.driveLevel,turnRate)
                    else:
                        self.drive(self.driveLevel,0)

                else: #if centre line missing 
                    
                    if leftLineBlock >= 0:
                        _,servoPos = self.visTrack(leftLineBlock)
                        
                        turnRate = (-0.0125)*servoPos + 0.5
                        print("left detected",servoPos,turnRate)
                        self.drive(0.2,turnRate)

                    elif rightLineBlock >=0:
                        _,servoPos = self.visTrack(rightLineBlock)
                        turnRate = (-0.025)*servoPos - 0.5
                        print("right detected",servoPos,turnRate)
                        self.drive(0.2,turnRate)                        

                    else: #leftLineBlock < 0 and rightLineBlock <0: # stop the racer and wait for new blocks 
                        self.drive(0, 0)
                        
                        #scan environment
                        if scan > 0 and scan < scanFreq:
                            self.bot.setServoPosition(30)
                        elif scan > scanFreq and scan < 2*scanFreq:
                            self.bot.setServoPosition(-30)
                        elif scan > 2*scanFreq and scan < 3*scanFreq:
                            self.bot.setServoPosition(0)
                        #print(scan)
                        scan = (scan + 1 )% (3*scanFreq)
                    
        print("ended")
        self.drive(0,0)

        return
