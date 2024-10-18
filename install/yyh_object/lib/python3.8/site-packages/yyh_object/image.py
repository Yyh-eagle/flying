import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
from rcl_interfaces.msg import ParameterDescriptor
import message_filters
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
        self.intrinsics =None
        #self.sub_color=message_filters.Subscriber(self.sub_color,Image,'/D435i/color/image_raw')
        #self.sub_depth=message_filters.Subscriber(self.sub_depth,Image,'/D435i/aligned_depth_to_color/image_raw')
        #self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_color, self.sub_depth], 10, 0.1, allow_headerless=True)
        #self.ts.registerCallback(self.listener_callback_d435)
        #D435i接收
        self.sub_color = self.create_subscription(
             Image, '/D435i/color/image_raw', self.listener_callback_d435, 10
        )
        self.sub_depth = self.create_subscription(
             Image, '/D435i/aligned_depth_to_color/image_raw', self.listener_callback_depth, 10
        )
        self.subscription = self.create_subscription(
            CameraInfo,
            '/D435i/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10)
        
        
        self.task_state=-1# the state machine
        self.d435_location = 1
        ######################确认相机内参########################
        # 1280*720 D435i 
        self.fx = 912.0516357421875
        self.fy = 911.5665283203125
        self.ppx =649.6558837890625
        self.ppy =377.07421875

      
        
        self.cv_bridge = CvBridge()                           # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换
        self.colord435i = None
        self.depthd435i = None
        self.usb = None
        
        ########################################pub#########################################
        self.pub_d435 = self.create_publisher(
            ObjectPosition, "d435_object_position", 10)              # 创建发布者对象（消息类型、话题名、队列长度）
        
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
            center_x=(x1+x2)/2
            center_y=(y1+y2)/2
            x1_int=int(x1)
            x2_int=int(x2)
            y1_int=int(y1)
            y2_int=int(y2)
            center_x_int=int(center_x)
            center_y_int=int(center_y)

            probability= detect_result.pred[0][index][4]
            if probability>threshold:
                pack=(center_x_int,center_y_int,x1_int,y1_int,x2_int,y2_int,probability,name_order)
                result.append(pack)#turple
    
        return result
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
            rate=d_x/c_x#find the position in  depth image
            x1=object[2]
            y1=object[3]
            x2=object[4]
            y2=object[5]
            kind_order=object[7]
            probability=object[6]
            center_x_int=int(rate*object[0])
            center_y_int=int(rate*object[1])
            xx = int(object[0])
            yy = int(object[1])
            real_z=depth_image[center_y_int,center_x_int]
            if real_z>1 and index <6:
                self.flag_d=1
                
                camera_coordinate = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xx,yy], real_z)
                real_x = camera_coordinate[0]
                real_y = camera_coordinate[1]
                real_z = camera_coordinate[2]
                #self.get_logger().info('real_x=%f real_y=%f real_z=%f' % (real_x,real_y,real_z))
                if self.d435_location==1:
                    real_y_int=int(real_x)
                    real_x_int=int((real_y-real_z)*0.707)
                    #if real_y_int <700:
                    #    real_y_int=real_y_int-250
                    real_z_int=int(-(real_z+real_y)*0.707)
                    
                if self.d435_location==0:
                    real_x_int=int(real_x)
                    real_y_int=int(real_z)
                    real_z_int=-int(real_y)
                    
                if self.d435_location==2:
                    real_x_int=1.5*int(real_x)
                    real_y_int=-1.5*int(real_y)
                    real_z_int=-int(real_z)
                    shape_rate=(y2-y1)/(x2-x1)
                    self.get_logger().info('rate=%f height=%d' % (shape_rate,real_z_int)) #输出图像中心点在相机坐标系下的x,y,z
                    if  (shape_rate>1.15 or shape_rate<0.85) and real_z_int>-700 :
                        real_x_int=0
                        real_y_int=0
                        real_z_int=0
            
                cv2.putText(color_image,str(kind_order),(x1,y1 ), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                if kind_order ==0:
                    self.get_logger().info('real_x=%f real_y=%f real_z=%f' % (real_x_int-400,real_y_int,real_z_int))
                    position.x[index], position.y[index],position.z[index] = real_x_int-400,real_y_int,real_z_int
                    index=index+1          
        position.f,position.kind=index,3
        
        self.pub_d435.publish(position)
        # cv2.imshow('depth_image',depth_image)
        cv2.imshow("color",color_image)                          # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(1)

    
    
    def decodeDisplay(self,image):
        img=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(self.usb)
        rects_list = []
        QR_info = []
        results=[]
        # 这里循环，因为画面中可能有多个二维码
        for barcode in barcodes:
            # 提取条形码的边界框的位置
            # 画出图像中条形码的边界框
            (x, y, w, h) = barcode.rect
            cx=x+w//2
            cy=y+h//2
            rects_list.append((x, y, w, h))
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # 条形码数据为字节对象，所以如果我们想在输出图像上画出来，就需要先将它转换成字符串
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            location=[cx,cy]
            # 绘出图像上条形码的数据和条形码类型
            text = str(barcodeData)
            result=[text,location]
            results.append(result)
            cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0, 0, 125), 2)
            # 向终端打印条形码数据和条形码类型
            # print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return results

    #颜色识别
    def color_detect(self,image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)      # 图像从BGR颜色模型转换为HSV模型
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red) # 图像二值化
        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)   # 图像中轮廓检测
        results = []
        for cnt in contours:                                  # 去除一些轮廓面积太小的噪声
            if cnt.shape[0] < 150:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)              # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)# 将苹果的轮廓勾勒出来
            cv2.circle(image, (int(x+w/2), int(y+h/2)), 5,
                       (0, 255, 0), -1)                       # 将苹果的图像中心点画出来
            results.append([int(y+h/2),int(x+w/2)])
 
        cv2.imshow("object", image)                           # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(10)
        
    #circle detect
    def houf_circle(self,image):
        
        #time =0.02s
        minRadius=25
        maxRadius=600
        circles = []
        e1=cv2.getTickCount()
        gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        blur_image = cv2.GaussianBlur(gray_image,(5,5),0)
        ret,thresh = cv2.threshold(gray_image,125,255,cv2.THRESH_BINARY)
        
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5), (-1, -1))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, (-1, -1))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, (-1, -1))

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if len(cnt) <5:
                continue
                
            area = cv2.contourArea(cnt)
            if area < (minRadius**2) * math.pi or area > (maxRadius**2) * math.pi:
                continue
            
            arc_length = cv2.arcLength(cnt, True)
            radius = arc_length / (2 * math.pi)
            
            if not (minRadius < radius and radius < maxRadius):
                continue
            
            ellipse = cv2.fitEllipse(cnt)
            ratio = float(ellipse[1][0]) / float(ellipse[1][1])
            
            if ratio > 0.8 and ratio < 1.2:#e
                corner = cv2.approxPolyDP(cnt, 0.02 * arc_length, True)
                cornerNum = len(corner)
                if cornerNum > 4: # 当cornerNum=4时，识别矩形；而cornerNum>4时，识别圆
                    circles.append(ellipse)
                    
            for circle in circles:
                #print(circle)
                cv2.circle(image, (int(circle[0][0]), int(circle[0][1])), int(0.25*(circle[1][0]+circle[1][1])), (0, 255, 0), thickness=5)
        
        e2 = cv2.getTickCount()
        time = (e2 - e1)/ cv2.getTickFrequency()
        #print(circles)
        cv2.imshow("hourf", image)                           # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(10)  
        #print(time)
        
    
                    
                    
    def listener_callback_depth(self,data):
        
        self.depthd435i = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')   # 将ROS的图像消息转化成OpenCV图像  
                    
    # 图像回调函数
    def listener_callback_d435(self, data):
        
        self.get_logger().info('')     # 输出日志信息，提示已进入回调函数
        self.colord435i = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        #print(self.colord435i.shape)
        if self.colord435i is not None and self.depthd435i is not None and self.intrinsics is not None:

            #self.houf_circle()
            
            self.locate_yolo_d4(self.colord435i,self.depthd435i)
            #self.color_detect(self.colord435i)

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
        #print(self.intrinsics)
        

        
 
def main(args=None):                            # ROS2节点主入口main函数()
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ImageSubscriber("image")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口