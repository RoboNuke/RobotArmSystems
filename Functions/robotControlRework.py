#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import time
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from perceptionRework import Perception
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

start_pick_up = False
rotation_angle = 0.0
world_x, world_y = 0,0
world_X, world_Y = 0,0

def perceptionLogic():
    global percept, rc
    global start_pick_up
    global world_x, world_X, world_y, world_Y
    global rotation_angle

    percept = Perception(detect_color = 'red')
    percept.startCamera()
    keep_roi = False
    last_x, last_y = 0,0
    while True:
        img = percept.getImg()
        if img is not None:
            
            if not start_pick_up:
                img, new_world_x, new_world_y, rotation_angle = percept.run(img, not start_pick_up, keep_roi)
                keep_roi = False
                if new_world_x == None:
                    continue
                world_x = new_world_x
                world_y = new_world_y
                keep_roi = True
                distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #对比上次坐标来判断是否移动
                last_x, last_y = world_x, world_y
                rc.tracking = True
                #print(count, distance)
                if rc.action_complete:
                    if distance < 0.75:
                        center_list.extend((world_x, world_y))
                        count += 1
                        if start_count_t1:
                            start_count_t1 = False
                            t1 = time.time()
                            #print("Reset")
                        if time.time() - t1 > 0.5:
                            start_count_t1 = True
                            world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                            count = 0
                            center_list = []
                            start_pick_up = True
                            #print("Start pickup is True")
                    else:
                        t1 = time.time()
                        start_count_t1 = True
                        count = 0
                        center_list = []
                        #print("Else reset")
            else:
                img = percept.drawAlignCross(img)
            if not percept.displayImg(img):
                break
        img = None
    percept.closeCamera()

def controlLogic():
    global percept, rc
    global start_pick_up
    global world_x, world_X, world_y, world_Y
    global rotation_angle


    while True:
        #print("here rc")
        if not rc.prepick and start_pick_up:
            rc.setColor(percept.detect_color)
            rc.beep(0.1)
            rc.moveToPrePick(world_X, world_Y)
            start_pick_up = False
        elif rc.prepick:
            rc.setColor(percept.detect_color)
            if rc.tracking:
                rc.trackLoc(world_x, world_y)
            if start_pick_up:
                rc.pick(world_X, world_Y, rotation_angle)
                rc.placeInBin(percept.detect_color, 1)
                start_pick_up = False
                rc.setColor('None')
            

    

rc = RobotControl()
percept = Perception('red')

th = threading.Thread(target = controlLogic)
th.setDaemon(True)
th.start()
if __name__ == "__main__":

    perceptionLogic()



