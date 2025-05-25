import RPi.GPIO as GPIO
import queue
import numpy as np
import math
import time
import threading
import cv2
from collections import deque

class PID:
    """PID Controller
    """

    def __init__(self, P=80, I=0, D=0, C=0,speed=0.4, duty=26):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Kc = C
        self.err_pre = 0
        self.err_last = 0
        self.u = 0
        self.integral = 0
        self.ideal_speed = speed
	
    def update(self, feedback_value):
        global xc
        global nowColor
        self.err_pre = self.ideal_speed - feedback_value
        self.integral += self.Ki * self.err_pre
        # 限制积分项范围
        self.integral = max(min(self.integral, 100), -100)
        self.u = self.Kp * self.err_pre + self.integral + self.Kd * (self.err_pre - self.err_last)
        self.u+=self.Kc*xc
        if self.u > 100:
            self.u = 100
        elif self.u < 0:
            self.u = 0
        self.err_last = self.err_pre  # 更新 err_last
        return self.u
    def set_ideal_speed(self, new_speed):
        """设置新的理想速度"""
        self.ideal_speed = new_speed
    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain
    def setKc(self, derivative_gain):
        self.Kc = derivative_gain

def pulse_callback(channel):
    """GPIO事件回调函数，用于记录脉冲时间并计算速度"""
    global lspeed, rspeed, last_ltime, last_rtime, lcounter, rcounter, ltotal_time, rtotal_time
    current_time = time.time()
    pulse_count = 5  # 设置需要的下降沿数量

    if channel == LS:
        if last_ltime is not None:
            period = current_time - last_ltime
            if period > 0:
                ltotal_time += period
                lcounter += 1
                if lcounter >= pulse_count:
                    lspeed = pulse_count / ltotal_time / 585.0 # 平均速度 = 脉冲数 / 总时间
                    lcounter = 0
                    ltotal_time = 0
        last_ltime = current_time

    elif channel == RS:
        if last_rtime is not None:
            period = current_time - last_rtime
            if period > 0:
                rtotal_time += period
                rcounter += 1
                if rcounter >= pulse_count:
                    rspeed = pulse_count / rtotal_time / 585.0  # 平均速度 = 脉冲数 / 总时间
                    rcounter = 0
                    rtotal_time = 0
        last_rtime = current_time

def stop():
    pwmright.ChangeDutyCycle(0)
    pwmleft.ChangeDutyCycle(0)

def shortforward(gotime):
    pwmright.ChangeDutyCycle(27)
    pwmleft.ChangeDutyCycle(25)
    time.sleep(gotime)
    stop()
    time.sleep(1)
def turnright(gotime):
    pwmright.ChangeDutyCycle(32)
    pwmleft.ChangeDutyCycle(0)
    time.sleep(gotime)
    stop()
    time.sleep(1)
    
def turnleft(gotime):
    pwmleft.ChangeDutyCycle(32)
    pwmright.ChangeDutyCycle(0)
    time.sleep(gotime)
    stop()
    time.sleep(1)
    
def pidforward():
    global lspeed
    global rspeed
    global image
    global lspeed
    global rspeed
    global lcounter
    global rcounter

    # 新增全局变量，用于记录脉冲时间
    last_ltime = None
    last_rtime = None
    ltotal_time = 0
    rtotal_time = 0
    GPIO.add_event_detect(6, GPIO.FALLING, callback=pulse_callback,bouncetime=200)
    GPIO.add_event_detect(12, GPIO.FALLING, callback=pulse_callback,bouncetime=200)

    i = 0
    x = []
    y1 = []
    y2 = []
    ridealspeed = 1.9
    lidealspeed = 2
    l_origin_duty = 0
    r_origin_duty = 0
    pwmright.start(l_origin_duty)
    pwmleft.start(r_origin_duty)
    L_control = PID(15, 0, 0.01, 0.03, lidealspeed, l_origin_duty)
    R_control = PID(13.7, 0, 0.01, 0, ridealspeed, r_origin_duty)
    
    while canforward(image,60000):
        pwmright.ChangeDutyCycle(L_control.update(lspeed))
        pwmleft.ChangeDutyCycle(R_control.update(rspeed))
        #print('left: %f  right: %f lduty: %f rduty: %f' % (lspeed, rspeed, L_control.u, R_control.u))
    
    stop()
    GPIO.remove_event_detect(6)
    GPIO.remove_event_detect(12)

def getthrough():#根据实际需要以及自己的巧思进行修改
    if nowTurnDirection==RIGHT:
        turnright(0.7)#0.8
        shortforward(1.3)#1.3
        turnleft(0.6)#0.7
        shortforward(1)#2
        #turnleft(0.5)add turnright ot left according to the next position
        #shortforward(1)
        #turnright(0.5)
        #time.sleep(0.5)
    elif nowTurnDirection==LEFT:
        turnleft(0.6)#0.8
        shortforward(1.2)#1
        turnright(0.7)#0.9
        shortforward(1.3)#1.8
        turnright(0.3)#this need to be adjusted
        #turnright(0.9)#0.9
        #shortforward(0.9)#0.9
        #turnleft(1)#1
        time.sleep(0.5)
    else:
        shortforward(2)
        
#图像处理模块
def canforward(image,threshold):

    # 转换为 HSV 色彩空间
    global nowColor
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 创建掩膜
    lower=lowerRange[nowColor]
    upper=upperRange[nowColor]
    mask = cv2.inRange(hsv, lower, upper)
    kernel=np.ones((3,3),np.uint8)
    mask=cv2.erode(mask,kernel,iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=5)
    # 计算非零像素的数量
    pixel_count = cv2.countNonZero(mask)
    print('count',pixel_count)
    if nowColor=="green":
        if pixel_count < 700:#700
            return 0
        return 1
    else:
        if pixel_count > threshold:
            return 0
        return 1        

def offset(image):
    global nowColor
    # 转换为 HSV 色彩空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 创建掩膜
    lower=lowerRange[nowColor]
    upper=upperRange[nowColor]
    mask = cv2.inRange(hsv, lower, upper)
    kernel=np.ones((3,3),np.uint8)
    mask=cv2.erode(mask,kernel,iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=5)
    #cv2.imshow("mask",mask)
    #cv2.waitKey(1)
    # 查找轮廓
    _,contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0
    
    # 计算图像中心的横坐标
    image_center_x = 320
    
    if nowColor in ["red", "yellow"]:
        # 对于红色和黄色，选择最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            # 计算偏移量
            offset_x = cX - image_center_x
            return offset_x
        return 0
    elif nowColor == "green":
        # 对于绿色，选择两个最大的轮廓
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
        centers = []
        for cnt in sorted_contours:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                # 计算轮廓的中心
                cX = int(M["m10"] / M["m00"])
                centers.append(cX)
        if len(centers) >= 2:
            # 计算两个中心的中点
            mid_x = (centers[0] + centers[1]) // 2
            # 计算偏移量
            offset_x = mid_x - image_center_x
            return offset_x
        return 0
    else:
        return 0        

def imgget():
    global image
    ret,image=cap.read()
    if not ret:
        print("image wrong")
        return 0
    
#函数主体

def getxc():
    global image
    global xc
    while 1:
        imgget()
        xc=offset(image)
        print(xc)

cap = cv2.VideoCapture(0)
pts = deque(maxlen=128)

EA, I2, I1, EB, I4, I3, LS, RS = (13, 19, 26, 16, 20, 21, 6, 12)
LEFT=1
RIGHT=2
MID=3
FREQUENCY = 50
GPIO.setmode(GPIO.BCM)
GPIO.setup([EA, I2, I1, EB, I4, I3], GPIO.OUT)
GPIO.setup([LS, RS],GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.output([EA, I2, EB, I3], GPIO.LOW)
GPIO.output([I1, I4], GPIO.HIGH)

pwmright = GPIO.PWM(EA, FREQUENCY)
pwmleft = GPIO.PWM(EB, FREQUENCY)#左右可能设置反，调整此处
pwmright.start(0)
pwmleft.start(0)

lspeed = 0
rspeed = 0
lcounter = 0
rcounter = 0
xc=0

# 新增全局变量，用于记录脉冲时间
last_ltime = None
last_rtime = None
ltotal_time = 0
rtotal_time = 0

lower_red = np.array([0, 80, 50])
upper_red = np.array([8, 255, 220])

lower_green = np.array([35, 61, 64])
upper_green = np.array([77, 200, 200])

lower_yellow = np.array([15, 70, 40])
upper_yellow = np.array([34, 200, 200])

lowerRange = {"red": lower_red, "green": lower_green, "yellow": lower_yellow}
upperRange = {"red": upper_red, "green": upper_green, "yellow": upper_yellow}


Color2Direction = {"red":RIGHT, "green":MID, "yellow":LEFT}
Colors = queue.Queue(4)

Colors.put("red")#根据时期情况填入颜色
Colors.put("green")
Colors.put("yellow")
Colors.put("red")


print("ready")
input()
nowColor = Colors.get()
nowTurnDirection = Color2Direction[nowColor]

thread1 = threading.Thread(target = getxc)
thread1.start()
time.sleep(3)
#行走逻辑
try:
    while 1:
        pidforward()
        print("remove")
        time.sleep(1)
        getthrough()
        if Colors.empty():
            pidforward()
        else:
            nowColor = Colors.get()
            nowTurnDirection = Color2Direction[nowColor]
except KeyboardInterrupt:
    pass

pwmright.stop()
pwmleft.stop()
cap.release()
GPIO.cleanup()      
