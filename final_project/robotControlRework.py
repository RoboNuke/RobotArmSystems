#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import time
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
import threading

class RobotControl:
    def __init__(self, track_offset = (0, -2, 5), grasp_height = 2):
        self.action_complete = False
        self.grasp_z = grasp_height
        self.hold_height = 12
        self.offset = track_offset
        self.AK = ArmIK()
        self.binCoords = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        self.gripper_servo = 500
        self.goHome()
        self.prepick = False
        self.tracking = False


    def goHome(self):
        self.action_complete = False
        Board.setBusServoPulse(1, self.gripper_servo - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)
        self.action_complete = True

    def moveToLoc(self, x, y, z, pitch, moveTime=None):
        """ Takes pose in and returns if successful """
        result = self.AK.setPitchRangeMoving((x,y,z), pitch, -90, 0, moveTime)
        if result == False:
            return False
        time.sleep(result[2]/1000)
        return True
    
    def trackLoc(self, x, y):
        xx = x + self.offset[0]
        yy = y + self.offset[1]
        zz = self.offset[2]
        result = self.moveToLoc(xx, yy, zz, -90, 20) 
        self.tracking = False
        return result
    
    def beep(self, t):
        """ Beeps for t seconds """
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(t)
        Board.setBuzzer(0)

    def setColor(self, color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()


    def rotateGripper(self, angle):
        Board.setBusServoPulse(2, angle, 500)
        time.sleep(0.8)

    def closeAndGrip(self):
        Board.setBusServoPulse(1, self.gripper_servo, 500) 
        time.sleep(1)
        Board.setBusServoPulse(2, 500, 500) # hold gripper closed

    def openGripper(self):
        Board.setBusServoPulse(1, self.gripper_servo - 280, 500)

    def placeInBin(self, binName="red", height = 1):
        bx = self.binCoords[binName][0]
        by = self.binCoords[binName][1]
        bz = self.binCoords[binName][2]
        self.action_complete = False
        # move above bin
        self.moveToLoc(bx, by, self.hold_height, -90)
        # adjust pitch angle
        pitch_angle = getAngle(bx, by, -90)
        self.rotateGripper(pitch_angle)
        # align above spot
        self.moveToLoc(bx, by, bz * height + 3, -90)
        # move to spot
        self.moveToLoc(bx, by, bz * height, -90)
        # open gripper
        self.openGripper()
        # move above placed object
        self.moveToLoc(bx, by, self.hold_height, -90)
        self.goHome()
        self.prepick = False
        self.action_complete = True

    def moveToPrePick(self, x, y):
        self.action_complete = False
        xx = x + self.offset[0]
        yy = y + self.offset[1]
        zz = self.offset[2]
        self.prepick = self.moveToLoc(xx,yy,zz,-90)
        self.action_complete = True
        return self.prepick

    def pick(self, x,  y, block_angle):
        self.action_complete = False
        self.openGripper()
        # set pitch angle
        pitch_angle = getAngle(x, y, block_angle)
        self.rotateGripper(pitch_angle)
        # move to grasp location
        self.moveToLoc(x,y,self.grasp_z, -90)
        self.closeAndGrip()
        # move to post grasp location
        self.moveToLoc(x,y,self.hold_height,-90, 1000)
        self.action_complete = True
