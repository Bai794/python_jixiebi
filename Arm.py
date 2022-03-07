# author:xiaobai
# time:2020/11/1621:37
# file name:Armpy
# tools:Pycharm
import math
import serial
import numpy as np
import matplotlib.pyplot as plt
from itertools import count
from mpl_toolkits.mplot3d import Axes3D
import serial
import time
from math import radians, sin, cos,atan2,acos
ax = plt.axes(projection = '3d')
ax.set_xlabel('y')
ax.set_ylabel('x')
ax.set_title('xiaobai Arm')
plt.grid(True)
ax.set_xlim(-200,200)
ax.set_ylim(-200,200)
plt.grid(True)
plt.ion()  #
class Arm:
    def __init__(self,angle1,angle2,angle3,w1,w2,w3,List_link):
        self.angle1=angle1
        self.angle2 = angle2
        self.angle3 = angle3
        self.speed1=w1
        self.speed2=w2
        self.speed3=w3
        self.L1=List_link[0]
        self.L2=List_link[1]
        self.L3=List_link[2]
        self.L4=List_link[3]
        self.joint_hm=[]#存储齐次变换矩阵
        self.dof=1
        self.joints_alpha = [0]
        self.joints_a = [self.L3]
        self.joints_d = [0]
        self.joints_theta = [-90]
        self.joints_angle = [0]
        self.moduan_zubiao_lis=[]
        self.ax = plt.axes(projection='3d')
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_title('xiaobai Arm')

        plt.grid(True)
        self.ax.set_xlim(-200, 200)
        self.ax.set_ylim(-200, 200)
        plt.ion()  #
        theta = 0
    def jici_matrix(self,theta1, theta2):
        theta1 = theta1 / 180 * np.pi
        theta2 = theta2 / 180 * np.pi
        first = np.array([[cos(theta1), -sin(theta1), 0, 0],
                          [sin(theta1), cos(theta1), 0, 0],
                          [0, 0, 1, self.L1],
                          [0, 0, 0, 1]])
        second = np.array([[0, 0, 1, 0],
                           [-sin(theta2), -cos(theta2), 0, 0],
                           [cos(theta2), -sin(theta2), 0, self.L2],
                           [0, 0, 0, 1]])
        return first, second
        pass
    def dh_matrix(alpha, a, d, theta):
        # 传入四个DH参数
        alpha = alpha / 180 * np.pi
        theta = theta / 180 * np.pi
        matrix = np.identity(4)
        matrix[0, 0] = cos(theta)
        matrix[0, 1] = -sin(theta)
        matrix[0, 2] = 0
        matrix[0, 3] = a
        matrix[1, 0] = sin(theta) * cos(alpha)
        matrix[1, 1] = cos(theta) * cos(alpha)
        matrix[1, 2] = -sin(alpha)
        matrix[1, 3] = -sin(alpha) * d
        matrix[2, 0] = sin(theta) * sin(alpha)
        matrix[2, 1] = cos(theta) * sin(alpha)
        matrix[2, 2] = cos(alpha)
        matrix[2, 3] = cos(alpha) * d
        matrix[3, 0] = 0
        matrix[3, 1] = 0
        matrix[3, 2] = 0
        matrix[3, 3] = 1
        return matrix
    def IK(self,x_value,y_value,z_value):
        norm=math.sqrt(x_value**2+y_value**2+(z_value-self.L1-self.L2)**2)
        z_value=z_value-self.L1-self.L2
        self.angle1=atan2(y_value,x_value)*180/np.pi
        self.angle3=acos((self.L3**2+self.L4**2-norm**2)/(2*self.L3*self.L4))*180.0/np.pi
        A=atan2(z_value,math.sqrt(x_value**2+y_value**2))*180/np.pi
        B=acos((norm*norm+self.L3**2-self.L4**2)/(2*self.L3*norm))*180/np.pi
        self.angle2=A+B
        pass
    def FK(self,angle1, angle2, angle3):
        first, second =Arm.jici_matrix(self,angle1,angle2)
        self.angle1=angle1+90
        self.angle2=angle2+90
        self.angle3=angle3+90
        self.joint_hm.append(np.round(first,decimals=4))
        self.joint_hm.append(np.round(second,decimals=4))
        for i in range(self.dof):
            self.joint_hm.append(Arm.dh_matrix(self.joints_alpha[i - 1], self.joints_a[i - 1], self.joints_d[i - 1],
                                               self.joints_theta[i - 1] + angle3))
        for i in range(self.dof + 1):
            self.joint_hm[i + 1] = np.round(np.dot(self.joint_hm[i], self.joint_hm[i + 1]),decimals=4)
        self.moduan_zubiao_list=np.round(np.dot(self.joint_hm[2], np.array([[self.L4], [0], [0], [1]])),decimals=4)
        pass
    def Jakebi(self,x,y,z):
        matrix = np.identity(3)
        matrix[0, 0] = self.L4*(cos(self.angle1)*sin(self.angle2)*sin(self.angle3)-cos(self.angle1)*cos(self.angle2)\
        *cos(self.angle3))+self.L3*cos(self.angle1)*sin(self.angle2)
        matrix[0, 1] = self.L4*(cos(y)*sin(x)*sin(z) + cos(z)*sin(x)*sin(y)) + self.L3*cos(y)*sin(x)
        matrix[0, 2] = self.L4*(cos(y)*sin(x)*sin(z) + cos(z)*sin(x)*sin(y))
        matrix[1, 0] = self.L3*sin(x)*sin(y) - self.L4*(cos(y)*cos(z)*sin(x) - sin(x)*sin(y)*sin(z))
        matrix[1, 1] = - self.L4*(cos(x)*cos(y)*sin(z) + cos(x)*cos(z)*sin(y)) - self.L3*cos(x)*cos(y)
        matrix[1, 2] = -self.L4*(cos(x)*cos(y)*sin(z) + cos(x)*cos(z)*sin(y))
        matrix[2, 0] = 0
        matrix[2, 1] =  self.L4*(cos(y)*cos(z) - sin(y)*sin(z)) - self.L3*sin(y)
        matrix[2, 2] = self.L4*(cos(y)*cos(z) - sin(y)*sin(z))
        return  matrix
        pass
    def draw_picture(self):
        # x_track = gen_path()
        # print(x_track)
        # xiaobai.IK(x_track_s[0], x_track_s[1], x_track_s[2])
        # xiaobai.FK((int)(xiaobai.angle1 - 90), (int)(xiaobai.angle2 - 90), (int)(xiaobai.angle3 - 90))
        # print(np.round(np.dot(xiaobai.joint_hm[2], np.array([[0], [0], [0], [1]])),decimals=4)[0])
        # print(x_track_s)
        mouduan = np.round(np.dot(xiaobai.joint_hm[2], np.array([[1], [0], [0], [1]])), decimals=4)
        #print(mouduan)
        xlist = []
        xlist = xlist + ([hm[0, 3] for hm in self.joint_hm])
        xlist.append(mouduan[0])
        ylist = []
        ylist = ylist + ([hm[1, 3] for hm in self.joint_hm])
        ylist.append(mouduan[1])
        zlist = []
        zlist = zlist + ([hm[2, 3] for hm in self.joint_hm])
        zlist.append(mouduan[2])
        #print(xlist, ylist, zlist)
        self.ax.plot3D(xlist, ylist, zlist, 'blue')
        self.joint_hm.clear()
        xlist.clear()
        ylist.clear()
        zlist.clear()
        plt.pause(0.05)
        # end = time.clock()
        pass
    def move_point_to_point(self,x1,y1,z1,x2,y2,z2):
        self.IK(x1,y1,z1)
        chazhi_list=[0,0,0]
        abs_chaizhi_list=[0,0,0]
        first_theta1=[0,0,0]
        second_theta2=[0,0,0]
        first_theta1[0]=self.angle1
        first_theta1[1] = self.angle2
        first_theta1[2] = self.angle3
        self.IK(x2,y2,z2)
        second_theta2[0] = self.angle1
        second_theta2[1] = self.angle2
        second_theta2[2] = self.angle3
        for i in range(3):
            chazhi_list[i]=second_theta2[i]-first_theta1[i]
            if chazhi_list[i]<0:
                abs_chaizhi_list[i]=(int)(-chazhi_list[i]+0.5)
            else:
                abs_chaizhi_list[i]=(int)(chazhi_list[i]+0.5)
        # print(abs_chaizhi_list,chazhi_list)
        max_value=max(abs_chaizhi_list)
        for i in range(max_value):
            for i in range(3):
                if(abs_chaizhi_list[i]>=0):
                    abs_chaizhi_list[i] = abs_chaizhi_list[i] - 1
                    if(chazhi_list[i] < 0):
                        first_theta1[i]=first_theta1[i]-1
                    else:
                        first_theta1[i] = first_theta1[i] +1
            self.FK((int)(first_theta1[0]-90),(int)(first_theta1[1]-90),(int)(first_theta1[2])-90)
            self.draw_picture()
            time.sleep(0.01)
        pass
    def draw_light(self,x1,y1,z1,x2,y2,z2,speed):
        pass

################以下是测试代码#############################
# Link_list=[45,35,80,70]
# oring_zuobiao=np.array([[Link_list[3]],[0],[0],[1]])
# xiaobai=Arm(0,0,0 ,1,1,1,Link_list)
# xiaobai.move_point_to_point(56, 20, 160, 0, 100, 40)