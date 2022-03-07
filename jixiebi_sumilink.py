# author:xiaobai
# time:2020/11/2220:52
# file name:jixiebi_sumilinkpy
# tools:Pycharm
import math
import numpy as np
import matplotlib.pyplot as plt
from itertools import count
from mpl_toolkits.mplot3d import Axes3D
import time
import multiprocessing
import serial
from math import radians, sin, cos,atan2,acos
from Arm import Arm
from my_serial import Communication
import struct
import tkinter
import threading
import cv2

#################杆的初始化################
Link_list=[45,35,80,70]
oring_zuobiao=np.array([[Link_list[3]],[0],[0],[1]])
xiaobai=Arm(0,0,0 ,1,1,1,Link_list)
usart = Communication("COM63", 9600, 0.5)
############################
x_track = np.zeros((1, 3))
x_track_s = np.array([.0,.0,.0])
ax = plt.axes(projection = '3d')
ax.set_xlabel('y')
ax.set_ylabel('x')
ax.set_title('xiaobai Arm')
x_track=[[12,34,2]]
plt.grid(True)
ax.set_xlim(-200,200)
ax.set_ylim(-200,200)
# plt.ion()  #
theta=0
mode=3
clor=2
runflag=0
###################################
rand1 = np.array([[39 ,24 ,205,178 ,225 ,205],[75  ,63 ,236,178 ,187, 236],[41 , 21 ,205,177 ,229 ,205],
                  [41 , 21, 205,177 ,229 ,205]
                  ,[45,  29, 214,177, 220, 214]])

rand2 = np.array([[ 152, 224 ,226 ,102 ,95, 250],[ 150 ,220 ,226, 102 ,90 ,245],
                  [152, 220 ,226 ,102 ,106, 240],[152, 220 ,216 ,102 ,106, 242 ],[34,  33, 31, 198, 221  , 223],[34,  33, 31, 192, 221  , 213],[34,  33, 31, 178, 221  , 213]])
ptLeftTop = (460, 200)
ptRightBottom = (500, 240)
point_color = (0, 0, 255) # BGR
thickness = 1
lineType = 8
lable = np.array([[0],[0],[0],[0],[0],[1],[1],[1],[1],[2],[2],[2]])
data = np.vstack((rand1,rand2)) #合并到一起
data = np.array(data,dtype='float32')
svm = cv2.ml.SVM_create()
# ml 机器学习模块 SCM_create() 创建
svm.setType(cv2.ml.SVM_EPS_SVR)
svm.setType(cv2.ml.SVM_C_SVC) # svm type
criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 1000, 1e-3)
svm.setTermCriteria(criteria)
svm.setKernel(cv2.ml.SVM_LINEAR) # line #线性分类器
svm.setC(0.01)
result= svm.train(data,cv2.ml.ROW_SAMPLE,lable)
#################获取圆的坐标################
def gen_path(): #
    global x_track_s,x_track,theta
    theta += 5*np.pi/180
    x = 40*np.sin(theta)
    y = 40*np.cos(theta)+80
    z=100
    x_track_s =[x,y,z]
    x_track = np.append(x_track, [x_track_s],axis=0)
    x_track_s=np.trunc(x_track_s)
    return x_track
#################开始画图的进程################
def my_svm(img):
    # print(img.shape[0:2])
    rgb=[0,0,0]
    hsv=[0,0,0]
    for i in range(14+460,30+460,2):
        for j in range(14+200,30+200,2):
            for k in range(0,3):
                rgb[k]=rgb[k]+img[j,i][k]/64
    img2=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for i in range(10,30,2):
        for j in range(10,30,2):
            for k in range(3):
                hsv[k]=hsv[k]+img[j,i][k]/64
    pt_data = np.array(rgb+hsv)
    pt_data = np.vstack([rgb+hsv])
    # print(pt_data)
    pt_data = np.array(pt_data, dtype='float32')
    (par1, par2) = svm.predict(pt_data)
    # print((int(par2[0])))
    return (int(par2[0]))
    pass
#########
def send_date(a,b,c):
    begin=bytes([35, (int)(a*100/100), (int)(a*100%100),(int)((180-b)*100/100),(int)((180-b)*100%100), (int)(c*100/100), (int)(c*100%100), 42])
    usart.Send_data(begin)
def draw_picture(mode):
    if mode==1:
        x_track = gen_path()
        xiaobai.IK(x_track_s[0], x_track_s[1], x_track_s[2])
        xiaobai.FK((int)(xiaobai.angle1 - 90), (int)(xiaobai.angle2 - 90), (int)(xiaobai.angle3 - 90))
    else:
        # xiaobai.FK((int)(xiaobai.angle1 - 90), (int)(xiaobai.angle2 - 90), (int)(xiaobai.angle3 - 90))
        pass
    # send_date(xiaobai.angle1, xiaobai.angle2, xiaobai.angle3)
    # print(xiaobai.angle1,xiaobai.angle2,xiaobai.angle3)
    mouduan=np.round(np.dot(xiaobai.joint_hm[2], np.array([[70], [0], [0], [1]])),decimals=4)
    # print(mouduan[1])
    # [1]])),decimals=4)[0])
    # print(x_track_s)
    # plt.cla()
    xlist = []
    ylist = []
    zlist = []

    xlist = xlist + ([hm[0, 3] for hm in xiaobai.joint_hm])
    xlist.append(mouduan[0])
    ylist = ylist + ([hm[1, 3] for hm in xiaobai.joint_hm])
    ylist.append(mouduan[1])
    zlist = zlist + ([hm[2, 3] for hm in xiaobai.joint_hm])
    zlist.append(mouduan[2])
    ax.plot3D(xlist, ylist, zlist, 'blue')
    xiaobai.joint_hm.clear()
    xlist.clear()
    ylist.clear()
    zlist.clear()
    plt.pause(0.002)
    # end = time.clock()
    pass
def move_point_to_point(x1,y1,z1,x2,y2,z2):
    xiaobai.IK(x1,y1,z1)
    chazhi_list=[0,0,0]
    abs_chaizhi_list=[0,0,0]
    first_theta1=[0,0,0]
    second_theta2=[0,0,0]
    first_theta1[0]=xiaobai.angle1
    first_theta1[1] = xiaobai.angle2
    first_theta1[2] = xiaobai.angle3
    xiaobai.IK(x2,y2,z2)
    second_theta2[0] = xiaobai.angle1
    second_theta2[1] = xiaobai.angle2
    second_theta2[2] = xiaobai.angle3
    for i in range(3):
        chazhi_list[i]=second_theta2[i]-first_theta1[i]
        if chazhi_list[i]<0:
            abs_chaizhi_list[i]=(int)(-chazhi_list[i]+0.5)
        else:
            abs_chaizhi_list[i]=(int)(chazhi_list[i]+0.5)
    # print(abs_chaizhi_list,chazhi_list)=
    max_value=max(abs_chaizhi_list)
    weingt=[1,1,1]
    for i in range(3):
        weingt[i]=abs_chaizhi_list[i]/max_value
    for i in range(max_value):
        for i in range(3):
            if(abs_chaizhi_list[i]>=0):
                abs_chaizhi_list[i] = abs_chaizhi_list[i] -1*weingt[i]
                if(chazhi_list[i] < 0):
                    first_theta1[i]=first_theta1[i]-1*weingt[i]
                else:
                    first_theta1[i] = first_theta1[i] +1*weingt[i]
        xiaobai.FK((first_theta1[0]-90),(first_theta1[1]-90),(first_theta1[2])-90)
        draw_picture(0)
    pass
def run1(data):
    ip_camera_url = 'http://admin:admin@10.178.58.84:8081/video'  # rtsp数据流
    cap = cv2.VideoCapture(ip_camera_url)
    while cap.isOpened():
        ret, frame = cap.read()
        if mode==0:
            cv2.rectangle(frame, ptLeftTop, ptRightBottom, point_color, thickness, lineType)
            global clor
            clor = my_svm(frame)
            if clor==1:
                cv2.putText(frame, 'yellow', ptLeftTop, cv2.FONT_HERSHEY_TRIPLEX, 0.5,  (0, 0, 255), 1, 4)
            elif clor==0:
                cv2.putText(frame, 'juhuanhg', ptLeftTop, cv2.FONT_HERSHEY_TRIPLEX, 0.5,  (0, 0, 255), 1, 4)
            else:
                cv2.putText(frame, 'nothing', ptLeftTop, cv2.FONT_HERSHEY_TRIPLEX, 0.5,  (0, 0, 255), 1, 4)
        # print(frame.shape[0:2])
        cv2.imshow('xiaobai_camera', frame)
        if cv2.waitKey(1) == ord('q'):
            # 退出程序
            break
    pass
if __name__ == '__main__':
     # xiaobai.move_point_to_point(30,40,60,0,80,20)
    # re=multiprocessing.Process(target=xiaobai.move_point_to_point(40,50,60,20,80,50))
    # re.start()
    # draw_target = multiprocessing.Process(target=draw_picture)
    # draw_target.start()
    # move_point_to_point(0, 70, 160, 0, 140, 40)
    # plt.cla()
    # move_point_to_point(0, 140, 40, 56, 20, 160)
    # plt.cla()
    #move_point_to_point(56, 20, 160, 0, 100, 40)
    # plt.cla()
    thre = threading.Thread(target=run1, args=(500,))  # 创建一个线程
    thre.start()
    print("这是主进程")
    print("请输入你要选择的模式")
    while 1:
        mode = int(input("0代表颜色分类抓取 1代表轨迹规划 :"))
        if mode == 1:
            for t in count():
                # start = time.clock()
                if t == 360:
                    plt.close()
                    break
                if t % 72 == 0:
                    plt.cla()
                draw_picture(1)
        else:
            while runflag<2:
                if cv2.waitKey(1) == ord('w'):
                    # 退出程序
                    break
                if clor == 1:
                    move_point_to_point(0, 70, 160, 0, 140, 40)
                    plt.cla()
                    move_point_to_point(0, 140, 40, 56, 20, 160)
                    plt.cla()
                    move_point_to_point(56, 20, 160, 100, 20, 40)
                    plt.cla()
                    move_point_to_point(100, 20, 40, 0, 70, 160)
                    plt.cla()
                    runflag=runflag+1

                elif clor == 0:
                    move_point_to_point(0, 70, 160, 0, 140, 40)
                    plt.cla()
                    move_point_to_point(0, 140, 40, -56, 20, 160)
                    plt.cla()
                    move_point_to_point(-56, 20, 160, -100, 20, 40)
                    plt.cla()
                    move_point_to_point(-100, 20, 40, 0, 70, 160)
                    plt.cla()
                    runflag = runflag + 1
                print(clor)
                pass








