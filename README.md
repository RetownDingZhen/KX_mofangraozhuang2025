# 魔方绕桩2025
本代码通过视觉驱动，可以完成遇见特定颜色的方块左转、右转以及从中间穿行的任务
## 注意：
本代码与电子系统导论并无直接关联，代码稳定性较差，如非走投无路并不建议采用此代码，若使用过程中产生任何问题均与本人无关
## 函数解析
### 以下为控制部分：
- **stop（）**：用于将小车左右轮占空比调整为0，使小车运动停止

- **shortforward(gotime)**:直接设置小车的左右轮占空比达到使得小车短距离直行的目的，其中gotime为需要小车短距离直行的时间，注意本函数不使用PID算法，无法进行长距离执行，函数中的左右轮占空比需要调整以达到小车可以大致直行以及速度合适的效果

- **turnleft(gotime)/turnright(gotime)**:直接设置小车的左/右轮占空比达到使得小车以另一轮为轴旋转的目的，其中gotime为需要小车短转弯的时间

- **pidforward()**:本函数引入 Kp Ki Kd Kc 四个参数，Kp Ki Kd 为传统的PID参数，引入的Kc为参数xc(目标方块中心或需要穿行的两方块连线中点与摄像头320像素间的距离)的系数，引入这两个变量以达到通过视觉引导小车的目的，PID的详细算法在class PID的定义中，无需改动。本函数启动后在满足函数canforward（）的条件下会一直执行

- **getthrough()**:在不同的nowTurnDirection的设置下，产生不同的绕行效果，以右转为例，小车会做出右转、直行、左转、直行的动作，完成绕过方块的行为，如发现绕行时转弯幅度过大或过小可在此函数的定义部分修改

以下为图像处理部分：
- **imgget()**:此函数用作不断从摄像头获取图像
- **canforward(image,threshold)**:此函数接受imgget（）函数获取的图像，对其进行创建掩膜等一系列处理，最终计算非零像素的数量，与threshold对比以评估距方块的距离。注意当颜色为需要从中间穿行的颜色时时此函数评估方式与其他颜色不同，并不使用threshold，此时的评估标准需要在函数定义内部进行修改！！！本代码配套的摄像头为640*480像素，即总共307200个像素点，酌情输入threshold以判断距离
- **offset(image)**：函数对imgget()获取图像进行创建掩膜等一系列处理，并查找轮廓，根据颜色获取对应的中心点坐标，最终计算摄像头横向中点与颜色块中心点坐标的距离以实现通过改进的PID算法追踪所需颜色块或颜色块中点。
- **getxc()**:imgget函数与offset(image)函数的组合，以达到在新开线程中不断获取xc以提交PID算法的效果
## 代码主体解析
```python
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
```
此部分为参数初始化部分，可根据实际情况修改EA I1 I2等值

```python
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

Colors.put("red")
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
```
此部分为颜色的初始化以及调整遇到不同颜色需要做出的动作的部分，不同颜色的HSV参数可根据实际情况修改。在Color2Direction中修改小车面对不同颜色时做出的反应。在队列中预先按顺序填入颜色块处理的顺序，以代码中的顺序为例，代码会以红色、绿色、黄色、红色的顺序处理颜色。在显示ready后，随意输入以开始行走。

```python
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
```
此为行走逻辑部分，满足canforward()的条件时，小车会追踪当前正在处理的颜色块，在距离不满足后，PID停止，小车进入getthrough（）函数开始通过方块，通过后，如果有还有需要处理的方块，则继续进行追踪并绕行，若没有需要处理的方块，小车开始进入直行（建议在通过所需方块后再在队列中多加入一个颜色）
