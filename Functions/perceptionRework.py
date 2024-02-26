#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import color_range
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *


"""
    BELOW IS NEW PERCEPTION CODE
"""

class Perception:
    def __init__(self, detect_color=-1):
        self.size = (640, 480) # image size to crop to 
        self.detect_color = detect_color
        self.minBlobArea = 2500
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

    def reset(self):
        pass
        

    def startCamera(self):
        self.cam = Camera.Camera()
        self.cam.camera_open()

    def closeCamera(self):
        self.cam.camera_close()
        cv2.destroyAllWindows()

    def getImg(self):
        """ Get Image from Camera """
        return self.cam.frame

    def displayImg(self, img):
        """ Returns false if quit was pressed """
        cv2.imshow("Frame", img)
        key = cv2.waitKey(1)
        return not key == 27
    
    def drawAlignCross(self, img):
        # draw lines for alignment
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        return img

    def preprocessImg(self, img):
        """ Gaussian Blur, and Resize and color convert"""
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
        return frame_lab

    def colorPreprocess(self, img, detect_color='red'):
        """ Binary Filter + open/close """
        frame_mask = cv2.inRange(img, color_range[detect_color][0], color_range[detect_color][1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
        return closed

    def getColorBlob(self, img, detect_color='red'):
        """ Find Contour, return area, location
            requires preprocessed binary image
        """
        contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] 
        return self.getAreaMaxContour(contours) 

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # 历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max

    def getLargestColorBlob(self, img):
        """ Iterate over colors keeping largest area and location"""
        if not self.detect_color == -1:
            ppbImg = self.colorPreprocess(img, self.detect_color) # preprocess binary image
            maxBlob, maxArea =  self.getColorBlob(ppbImg, self.detect_color)
            return maxBlob, maxArea, self.detect_color
        
        maxArea = -1
        maxBlob = None
        color_detect = 'red'
        for i in color_range:
            ppbImg = self.colorPreprocess(img, i) # preprocess binary image
            areaBlob, areaMax = self.getColorBlob(ppbImg, i)
            if areaMax > maxArea:
                maxArea = areaMax
                maxBlob = areaBlob
                color_detect = i
        return maxBlob, maxArea, i

    def identifyContour(self, maxBlob):
        """ Return bounding box, center, center in robot coords"""
        rect = cv2.minAreaRect(maxBlob)
        box = np.int0(cv2.boxPoints(rect))

        roi = getROI(box) 

        img_x, img_y = getCenter(rect, roi, self.size, square_length) 
        world_x, world_y = convertCoordinate(img_x, img_y, self.size) 
        return box, world_x, world_y, rect[2]

    def highlightBlobOnImg(self, img, box, world_x, world_y, detect_color='red'):
        """ Adds lines around object and writes center """
        cv2.drawContours(img, [box], -1, self.range_rgb[detect_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[detect_color], 1)

    def estCenter(self):
        """ Keeps running avg of center locations """
        pass

    def run(self, img, find_blobs = True):
        img_copy = img.copy()
        img = self.drawAlignCross(img)

        if find_blobs:
            img_lab = self.preprocessImg(img_copy)
            maxBlob, maxArea, detect_color = self.getLargestColorBlob(img_lab)
            if maxArea > self.minBlobArea:
                bbox, world_x, world_y, angle = self.identifyContour(maxBlob)
                self.highlightBlobOnImg(img, bbox, world_x, world_y, detect_color )
                return img, world_x, world_y, angle
        return img, None, None, None




"""
    BELOW IS OLD ROBOT CONTROL CODE FOR DEMO
"""

AK = ArmIK()
__isRunning = False
first_move = False
start_pick_up = False
action_finish = True
detect_color = None # come back to 
world_X, world_Y = 0,0  # final block location
world_x, world_y = 0,0  # tracking target
last_x, last_y = 0,0
rotation_angle = 0
unreachable = False
track = False
servo1 = 500
get_roi = False
_stop = False


count = 0
center_list = []
detect_color = 'None'
__target_color = ()
start_count_t1 = False
t1 = 0

def reset():
    global count
    global track
    global _stop
    global get_roi
    global first_move
    global center_list
    global __isRunning
    global detect_color
    global action_finish
    global start_pick_up
    global __target_color
    global start_count_t1
    
    count = 0
    _stop = False
    track = False
    get_roi = False
    center_list = []
    first_move = True
    __target_color = ()
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True
# 初始位置
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")

# app停止玩法调用
def stop():
    global _stop 
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Stop")

# app退出玩法调用
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Exit")
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


def set_rgb(color):
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

# 机械臂移动线程
def move():
    global track
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color
    global action_finish
    global rotation_angle
    global world_X, world_Y
    global world_x, world_y
    global start_pick_up, first_move

    # 不同颜色木快放置坐标(x, y, z)
    coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
    }
    while True:
        if __isRunning:
            if first_move and start_pick_up: # 当首次检测到物体时               
                action_finish = False
                set_rgb(detect_color)
                setBuzzer(0.1)               
                result = AK.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90, 0) # 不填运行时间参数，自适应运行时间
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                time.sleep(result[2]/1000) # 返回参数的第三项为时间
                start_pick_up = False
                print("Start pickup == false: if first_move and start_pick_up")
                first_move = False
                action_finish = True
            elif not first_move and not unreachable: # 不是第一次检测到物体
                set_rgb(detect_color)
                if track: # 如果是跟踪阶段
                    if not __isRunning: # 停止以及退出标志位检测
                        continue
                    AK.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
                    time.sleep(0.02)                    
                    track = False
                if start_pick_up: #如果物体没有移动一段时间，开始夹取
                    action_finish = False
                    if not __isRunning: # 停止以及退出标志位检测
                        continue
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # 爪子张开
                    # 计算夹持器需要旋转的角度
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.8)
                    
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  # 降低高度
                    time.sleep(2)
                    
                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1, 500)  # 夹持器闭合
                    time.sleep(1)
                    
                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  # 机械臂抬起
                    time.sleep(1)
                    
                    if not __isRunning:
                        continue
                    # 对不同颜色方块进行分类放置
                    result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)   
                    time.sleep(result[2]/1000)
                    
                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)
                    
                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # 爪子张开，放下物体
                    time.sleep(0.8)
                    
                    if not __isRunning:
                        continue                    
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)

                    initMove()  # 回到初始位置
                    time.sleep(1.5)

                    detect_color = 'None'
                    first_move = True
                    get_roi = False
                    action_finish = True
                    start_pick_up = False
                    print("Start pickup == false after pickup sequence")
                    set_rgb(detect_color)
                else:
                    time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)

# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


if __name__ == '__main__':
    __target_color = 'red'
    detect_color = 'red'
    initMove()
    percept = Perception(detect_color = detect_color)
    percept.startCamera()
    start()
    while True:
        img = percept.getImg()
        if img is not None:
            
            if not start_pick_up:
                img, world_x, world_y, rotation_angle = percept.run(img, not start_pick_up)

                distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #对比上次坐标来判断是否移动
                last_x, last_y = world_x, world_y
                track = True
                if action_finish:
                    if distance < 0.3:
                        center_list.extend((world_x, world_y))
                        count += 1
                        if start_count_t1:
                            start_count_t1 = False
                            t1 = time.time()
                            print("Reset")
                        if time.time() - t1 > 1.5:
                            start_count_t1 = True
                            world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                            count = 0
                            center_list = []
                            start_pick_up = True
                            print("Start pickup is True")
                    else:
                        t1 = time.time()
                        start_count_t1 = True
                        count = 0
                        center_list = []
                        print("Else reset")
            else:
                img = percept.drawAlignCross(img)
            if not percept.displayImg(img):
                break
    percept.closeCamera()