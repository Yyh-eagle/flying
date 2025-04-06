import rclpy                           
from rclpy.node import Node             
from sensor_msgs.msg import Image       # 图像消息类型
from sensor_msgs.msg import CameraInfo  #相机参数消息
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
from rcl_interfaces.msg import ParameterDescriptor
from learning_interface.msg import ObjectPosition,MyState,STM32  # 自定义的目标位置消息
import pyrealsense2 as rs2#python内置的d435i库

import cv2                             
import numpy as np                      
import math

import sys
sys.path.append('/home/yyh/ros2_ws/src/yyh_object/yyh_object/')
sys.path.append('/home/yyh/ros2_ws/src/yyh_nav/yyh_nav/')
import os
import datetime
from yolo_function import *
from ament_index_python.packages import get_package_share_directory#用于
from yolov5 import YOLOv5


package_share_directory = get_package_share_directory('yolov5_ros2')

class Param():#当做参数传递
    def __init__(self,logger):
        self.ifarrive = None
        self.task_state = None
        self.task_id = None
        self.D435i_yaw = None
        self.color_d435i = None
        self.depth_d435i = None
        self.intrinsics =None
        self.logger = logger
        #self.usb =None
    
    def update_param(self,ifarrive,task_state,task_id,D435i_yaw):
        self.ifarrive = ifarrive
        self.task_state = task_state
        self.task_id = task_id
        self.D435i_yaw = D435i_yaw



class ImageSubscriber(Node):
    def __init__(self, name):
        
        super().__init__(name)    
        # ROS2节点父类初始化
        self.frame_cnt =0#todo 干啥用的
        self.param = Param(self.get_logger())
        self.colord435i = None
        self.depthd435i = None
        self.intrinsics = None
        self.sub_stm = self.create_subscription(#验证成功
            STM32, "/stm_info", self.listener_callback_stm, 10)
        #D435i接收
        self.sub_color = self.create_subscription(
             Image, '/D435i/color/image_raw', self.listener_callback_d435, 10
        )
        self.sub_depth = self.create_subscription(
             Image, '/D435i/aligned_depth_to_color/image_raw', self.listener_callback_depth, 10
        )
        self.subscription = self.create_subscription(
            CameraInfo,'/D435i/aligned_depth_to_color/camera_info',self.camera_info_callback,10
        )
    
        self.cv_bridge = CvBridge()# 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换

        #self.usb = cv2.VideoCapture(6)#小相机
        
        ########################################pub#########################################
        self.pub_d435 = self.create_publisher(
            ObjectPosition, "d435_object_position", 10)              # 创建发布者对象（消息类型、话题名、队列长度）

        self.pub_usb = self.create_publisher(
            ObjectPosition, "usb_object_position", 10)    
        #########################################yolo#########################################


        model_path = f"{package_share_directory}/config/best_8_1.pt"#模型的路径

        self.yolov5 = YOLOv5(model_path=model_path, device="cpu")


    def task_plan(self,param):#任务规划
        if param.task_state == 0:
            if self.param.task_state == 0:
                aim = yolo_root_d4(param,self.yolov5)
                if aim is not None:
                    object = self.Aim2Object(aim)
                    self.pub_d435.publish(object)
    def Aim2Object(self,aim):
        object = ObjectPosition()
        object.x = aim.x
        object.y = aim.y
        object.z = aim.z
        object.f = aim.f
        object.kind = aim.kind
        return object



    # 图像回调函数
    def listener_callback_depth(self , data):
        
        self.depthd435i = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')   # 将ROS的图像消息转化成OpenCV图像  
        self.param.depth_d435i = self.depthd435i
    def listener_callback_d435(self, data):
        
        self.colord435i = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.param.color_d435i = self.colord435i
        if self.param.task_state is not None and self.param.color_d435i is not None and self.param.depth_d435i is not None and self.param.intrinsics is not None:
            cv2.putText(self.param.color_d435i,"task_state: "+str(self.task_state),(10,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
            cv2.putText(self.param.color_d435i,"task_id: "+str(self.task_id),(10,90),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
            self.task_plan(self.param)
        cv2.imshow("d435i",self.param.color_d435i)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return
        
    def camera_info_callback(self, msg):
        # 获取内参
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.fx = msg.k[0]  # 焦距 fx
        self.intrinsics.fy = msg.k[4]  # 焦距 fy
        self.intrinsics.ppx = msg.k[2]  # 主点 x 坐标
        self.intrinsics.ppy = msg.k[5]  # 主点 y 坐标
        self.intrinsics.model = rs2.distortion.brown_conrady  # 使用的畸变模型
        self.intrinsics.coeffs = [msg.d[0], msg.d[1], msg.d[2], msg.d[3], msg.d[4]]
        self.param.intrinsics = self.intrinsics         
    

    def listener_callback_stm(self,msg):#主任务在这里写，可以保证周期话运行
    #每次收到stm32的信息，都赋值给全局变量
        self.ifarrive = msg.ifarrive
        self.task_id = msg.id
        self.task_state = msg.state
        self.D435i_yaw = msg.yaw
        self.param.update_param(self.ifarrive,self.task_state,self.task_id,self.D435i_yaw)

    


            





def main(args=None):                            # ROS2节点主入口main函数()
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ImageSubscriber("image")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口