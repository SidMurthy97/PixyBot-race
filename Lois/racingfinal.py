from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
import math

class racingfinal(object):
    def __init__(self, bot=pixyBot(0), cam=pixyCam()):
        self.bot = bot
        self.cam = cam
    
        self.centerLineID = 1
        self.leftLineID = 2
        self.rightLineID = 3
        self.obstacleID = 4
    
        nObservations = 20
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]
        self.blockHeight = [float('nan') for i in range(nObservations)]
        self.blockDistance = [float('nan') for i in range(nObservations)]
        self.blockRot = [float('nan') for i in range(nObservations)]
        
        self.obstacleDistance = 25
        self.obstacleError = 40
        
    def drive(self,drive,bias):
        if bias > 1:
            bias = 1
        if bias < -1:
            bias = -1
            
        maxDrive = 0.847
            
        totalDrive = drive*maxDrive
        diffDrive = bias*totalDrive
        straightDrive = totalDrive-abs(diffDrive)
            
        lDrive = 1.2*(straightDrive+diffDrive)
        rDrive = straightDrive-diffDrive
            
        self.bot.setMotorSpeeds(lDrive,rDrive)
            
    def getBlockParams(self,blockIdx):
        if (self.cam.newCount-1)<blockIdx or blockIdx < 0:
            return -1
        else:
            pixelSize = self.cam.newBlocks[blockIdx].m_width
            pixelHeight = self.cam.newBlocks[blockIdx].m_height
            angleSize = pixelSize/self.cam.pixyMaxX*self.cam.pixyX_FoV
            pixelError = self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX
            angleError = pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV #angle error between block and camera's field of view
            angleOfBlock = self.cam.newBlocks[blockIdx].m_angle
                
        self.blockSize.append(angleSize)
        self.blockAngle.append(angleError)
        self.frameTimes.append(time())
        self.blockHeight.append(pixelHeight)
        self.blockRot.append(angleOfBlock)
        
            
        self.blockSize.pop(0)
        self.blockAngle.pop(0)
        self.frameTimes.pop(0)
        self.blockHeight.pop(0)
        self.blockRot.pop(0)
            
    def visTrack(self,blockIdx):
        if blockIdx < 0:
            return -1
        else:
            pixelError = self.cam.newBlocks[blockIdx].m_x-self.cam.pixyCenterX
            visAngularError = -(pixelError/self.cam.pixyMaxX*self.campixyX_FoV)
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError)
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition
                
    def race(self,speed):
        self.bot.setServoPosition(0)
        self.drive(0,0) 
        steering = 0
        servopos = 0
        oldservopos = servopos
        drivespeed = speed
         
        scan =0
        scanFreq = 50
        
        while True:   
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
                
            if centerLineBlock >= 0:
             
                self.getBlockParams(centerLineBlock)
                angle = self.blockAngle[-1]
                camerarot = -(servopos/50)*25
                totalangle = angle+camerarot
                #steering = totalangle/self.cam.pixyX_FoV
                #speed = drivespeed
                #if self.blockSize < self.blockHeight:
                    #steering = totalangle/self.cam.pixyX_FoV
                    #speed = drivespeed
                #elif self.blockSize > self.blockHeight:
                    #steering = 2.4*totalangle/self.cam.pixyX_FoV
                    #speed = 0.1
                if self.blockRot[-1] > 3:
                    steering = totalangle/self.cam.pixyX_FoV
                    speed = 0.2
                elif self.blockRot[-1] < -3:
                    steering = totalangle/self.cam.pixyX_FoV
                    speed = 0.2
                elif abs(self.blockRot[-1]) < 3:
                    steering = totalangle/self.cam.pixyX_FoV
                    speed = drivespeed
                   
                    
            elif leftLineBlock >= 0:
                self.getBlockParams(leftLineBlock)
                angle = self.blockAngle[-1] + 16
                camerarot = -(servopos/50)*25
                totalangle = angle+camerarot
                #steering = totalangle/(self.cam.pixyX_FoV/1.5)
                #speed = drivespeed
                if self.blockRot[-1] > 3:
                    steering = totalangle/(self.cam.pixyX_FoV/1.5)
                    speed = 0.2
                elif self.blockRot[-1] < -3:
                    steering = totalangle/(self.cam.pixyX_FoV/1.5)
                    speed = 0.2
                elif abs(self.blockRot[-1]) < 3:
                    steering = totalangle/(self.cam.pixyX_FoV/1.5)
                    speed = drivespeed
                
               # if self.blockSize < self.blockHeight:
                   # steering = totalangle/(self.cam.pixyX_FoV/1.5)
                   # speed = drivespeed
               # elif self.blockSize > self.blockHeight:
                    #steering = totalangle/(self.cam.pixyX_FoV/1.5)
                   # speed = 0.1
                
                
            elif rightLineBlock >= 0:
                self.getBlockParams(leftLineBlock)
                angle = self.blockAngle[-1] - 16
                camerarot = -(servopos/50)*25
                totalangle = 2*angle+camerarot
                #steering = -totalangle/(self.cam.pixyX_FoV/1.5)
                #speed = drivespeed
                if self.blockRot[-1] > 3:
                    steering = -totalangle/(self.cam.pixyX_FoV/1.5)
                    speed = 0.2
                elif self.blockRot[-1] < -3:
                    steering = -totalangle/(self.cam.pixyX_FoV/1.5)
                    speed = 0.2
                elif abs(self.blockRot[-1]) < 3:
                    steering = -totalangle/(self.cam.pixyX_FoV/1.5)
                    speed = drivespeed
                
               # if self.blockSize < self.blockHeight:
                    #steering = totalangle/(self.cam.pixyX_FoV/1.5)
                    #speed = drivespeed
                #elif self.blockSize > self.blockHeight:
                    #steering = totalangle/(self.cam.pixyX_FoV/1.5)
                    #speed = 0.1
                                   
            else:
                speed = 0
                steering = 0 
                
                if scan > 0 and scan < scanFreq:
                    self.bot.setServoPosition(20)
                elif scan > scanFreq and scan < 2*scanFreq:
                    self.bot.setServoPosition(-20)
                elif scan > 2*scanFreq and scan < 3*scanFreq:
                    self.bot.setServoPosition(0)
                scan = (scan + 1)% (3*scanFreq)
                
                
                    
            self.drive(speed,steering)