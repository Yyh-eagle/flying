import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
from rcl_interfaces.msg import ParameterDescriptor

from learning_interface.msg import ObjectPosition,MyState  # 自定义的目标位置消息
import pyrealsense2 as rs2

import cv2                              # Opencv图像处理库
import numpy as np                      # Python数值计算库
import math



import pyzbar.pyzbar as pyzbar# 
from ament_index_python.packages import get_package_share_directory
from yolov5 import YOLOv5
lower_red = np.array([0, 90, 128])      # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])   # 红色的HSV阈值上限

package_share_directory = get_package_share_directory('yolov5_ros2')

#创建一个订阅者节点

class ImageSubscriber(Node):
    def __init__(self, name):
        
        super().__init__(name)    
        # ROS2节点父类初始化
        self.frame_cnt =0
        
        
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
        
        self.task_state=-1# the state machine
        self.d435_location = 1

        self.cv_bridge = CvBridge()# 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换
        self.colord435i = None
        self.depthd435i = None
        self.intrinsics =None
        #self.usb = cv2.VideoCapture(0)#little usb
        
        ########################################pub#########################################
        self.pub_d435 = self.create_publisher(
            ObjectPosition, "d435_object_position", 10)              # 创建发布者对象（消息类型、话题名、队列长度）
        self.pub_usb = self.create_publisher(
            ObjectPosition, "usb_object_position", 10)    
        #########################################yolo#########################################
        
        self.declare_parameter("device", "cpu", ParameterDescriptor(
            name="device", description="calculate_device default:cpu optional:cuda:0"))

        self.declare_parameter("model", f"{package_share_directory}/config/best_8_1_2.pt", ParameterDescriptor(
            name="model", description=f"default: {package_share_directory}/config/best_landing1.pt"))

        self.declare_parameter("image_topic", "/image_raw", ParameterDescriptor(
            name="image_topic", description=f"default: /image_raw"))

        self.declare_parameter("camera_info_topic", "/camera/camera_info", ParameterDescriptor(
            name="camera_info_topic", description=f"default: /camera/camera_info"))

        # 默认从camera_info中读取参数,如果可以从话题接收到参数则覆盖文件中的参数
        self.declare_parameter("camera_info_file", f"{package_share_directory}/config/camera_info.yaml", ParameterDescriptor(
            name="camera_info", description=f"{package_share_directory}/config/camera_info.yaml"))
        
        
        # 1.load model
        model = self.get_parameter('model').value
        device = self.get_parameter('device').value
        self.yolov5 = YOLOv5(model_path=model, device=device)
        
    # 图像回调函数
    def listener_callback_depth(self,data):
        
        self.depthd435i = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')   # 将ROS的图像消息转化成OpenCV图像  
    
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
    
    def listener_callback_d435(self, data):
        
        self.get_logger().info('')     # 输出日志信息，提示已进入回调函数
        self.colord435i = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        #_,self.usb_frame = self.usb.read()
        #if self.colord435i is not None and self.depthd435i is not None and self.intrinsics is not None and self.usb_frame is not None:
        if self.colord435i is not None and self.depthd435i is not None and self.intrinsics is not None :
            self.locate_yolo_d4(self.colord435i,self.depthd435i)
            
            
##########################################################################################################################################################33
    def locate_yolo_d4(self,color_image,depth_image):
        real_x_int=0
        real_y_int=0
        real_z_int=0
        index=0
        position = ObjectPosition()#create a object for message transfer

        d_y, d_x = depth_image.shape[0:2] #dy = depth_image.shape[0],the number of the line
        c_y, c_x = color_image.shape[0:2]
        #self.get_logger().info('d_x=%d,d_y=%d  c_x=%d,c_y=%d,c_z=%d' % (d_x,d_y,c_x,c_y,c_z))   
        
        results=self.yolo_recog(color_image,0.8)#threshold =0.8
        for object in results:
            
            rate=d_x/c_x
            
            (x1,y1,x2,y2) = object[2:6]
            kind_order=object[7]
            probability=object[6]
            
            center_x_int=int(rate*object[0])
            center_y_int=int(rate*object[1])
            real_z=depth_image[center_y_int,center_x_int]
            
            
            if real_z>1 and index <6:
                self.flag_d=1
                #camera position
                camera_coordinate = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [int(object[0]),int(object[1])], real_z)
                (real_x,real_y,real_z) = camera_coordinate[0:3]
              
                #self.get_logger().info('real_x=%f real_y=%f real_z=%f' % (real_x,real_y,real_z))
                if self.d435_location==1:
                    real_y_int=int(real_x)
                    real_x_int=int((real_y-real_z)*0.707)
                    real_z_int=int(-(real_z+real_y)*0.707)
                    
                if self.d435_location==0:
                    real_x_int=int(real_x)
                    real_y_int=int(real_z)
                    real_z_int=-int(real_y)
                    

                cv2.putText(color_image,str(kind_order),(x1,y1 ), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                if kind_order ==0:
                    self.get_logger().info('real_x=%f real_y=%f real_z=%f' % (real_x_int-400,real_y_int,real_z_int))
                    position.x[index], position.y[index],position.z[index] = real_x_int-400,real_y_int,real_z_int
                    index=index+1          
        position.f,position.kind=index,3
        
        self.pub_d435.publish(position)
        cv2.imshow("color",color_image)                          # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(1)
    def locate_yolo_usb(self,color_image):
        #self.get_logger().info('d_x=%d,d_y=%d  c_x=%d,c_y=%d,c_z=%d' % (d_x,d_y,c_x,c_y,c_z))   
        flag_find=0
        target_x=0
        target_y=0
        
        results=self.yolo_recog(color_image,0.80)
        kind_order=-1
        for object in results:
            (x1,y1,x2,y2) = object[2:6]
            probability=object[6]
            kind_order=object[7]
            
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)   
            cv2.putText(color_image,str(kind_order),(x1,y1 ), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            
            target_x=(int(color_image.shape[1]//2)-int(object[0]))
            target_y=(int(object[1])-int(color_image.shape[0]//2))
            flag_find=1
    
        position = ObjectPosition()
        if kind_order==0:
            position.x[0], position.y[0],position.z[0],position.f,position.kind = target_y,-target_x,0,flag_find,13
            print(position.x[0], position.y[0],position.z[0])
        self.pub_usb.publish(position)
        cv2.imshow("color",color_image)                          # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(1)
        
    def yolo_recog(self,img,threshold):#yolo for d435i
        detect_result = self.yolov5.predict(img)#predict
        predictions = detect_result.pred[0]#result
        boxes = predictions[:, :4]  # x1, y1, x2, y2  = predictions[:,0],predictions[:,1],predictions[:,2],predictions[:,3]
        scores = predictions[:, 4]#score = predictions[:,4]
        categories = predictions[:, 5]#cate = predictions[:,5]
        result=[]
        kinds=['landing_cross','one','two','three','four','A']
        for index in range(len(categories)):
            name = detect_result.names[int(categories[index])]
            for j in range(len(kinds)):
                if kinds[j] ==name:

                    name_order=j#find the same kind the same with category
                    break
                
            x1, y1, x2, y2 = boxes[index]#find th center of the box
            probability= detect_result.pred[0][index][4]
            if probability>threshold:
                pack=(int((x1+x2)/2),int((y1+y2)/2),int(x1),int(y1),int(x2),int(y2),probability,name_order)
                result.append(pack)#turple
    
        return result
   

    


def main(args=None):                            # ROS2节点主入口main函数()
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ImageSubscriber("image")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口