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
from block_solver import SceneGraph

class SceneGraphPerception():
    def __init__(self):
        self.size = (640, 480) # image size to crop to 
        self.minBlobArea = 2500
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

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

    def getMaskROI(self, frame, roi, size):
        x_min, x_max, y_min, y_max = roi
        x_min -= 10
        x_max += 10
        y_min -= 10
        y_max += 10

        if x_min < 0:
            x_min = 0
        if x_max > size[0]:
            x_max = size[0]
        if y_min < 0:
            y_min = 0
        if y_max > size[1]:
            y_max = size[1]

        black_img = np.zeros([size[1], size[0]], dtype=np.uint8)
        black_img = cv2.cvtColor(black_img, cv2.COLOR_GRAY2RGB)
        black_img[y_min:y_max, x_min:x_max] = frame[y_min:y_max, x_min:x_max]
        
        return black_img
    
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

    def getColorBlob(self, img):
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

    def getLargestColorBlob(self, img, color='red'):
        """ Iterate over colors keeping largest area and location"""
        ppbImg = self.colorPreprocess(img, color) # preprocess binary image
        return self.getColorBlob(ppbImg) # returns contour, area

    def identifyContour(self, maxBlob):
        """ Return bounding box, center, center in robot coords"""
        rect = cv2.minAreaRect(maxBlob)
        box = np.int0(cv2.boxPoints(rect))

        self.roi = getROI(box) 

        img_x, img_y = getCenter(rect, self.roi, self.size, square_length) 
        world_x, world_y = convertCoordinate(img_x, img_y, self.size) 
        return box, world_x, world_y, rect[2]

    def highlightBlobOnImg(self, img, box, world_x, world_y, detect_color='red'):
        """ Adds lines around object and writes center """
        cv2.drawContours(img, [box], -1, self.range_rgb[detect_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[detect_color], 1)

    def getGraph(self, img):
        img_copy = img.copy()
        img = self.drawAlignCross(img)

        img_lab = self.preprocessImg(img_copy)
        sg = SceneGraph()
        idx = 0
        for color in ['red','blue','green']:
            maxBlob, maxArea = self.getLargestColorBlob(img_lab)
            if maxArea > self.minBlobArea:
                bbox, world_x, world_y, angle = self.identifyContour(maxBlob)
                print(bbox, world_x, world_y, angle)
                self.highlightBlobOnImg(img, bbox, world_x, world_y)
                sg.addBlock(color, [world_x,world_y,angle], idx)
                idx += 1
        return sg, img


if __name__=="__main__":
    SGP = SceneGraphPerception()
    SGP.startCamera()
    for i in range(10):
        raw_img = SGP.getImg()
        if raw_img is not None:
            scene_graph, post_img = SGP.getGraph(raw_img)
            for block in scene_graph.blocks:
                print(block)
            if not SGP.displayImg(post_img):
                break
    SGP.closeCamera()


