import rclpy                                       # ROS2 Python接口库
from rclpy.node   import Node                      # ROS2 节点类
from std_msgs.msg import String                    # 字符串消息类型
from tf2_msgs.msg import TFMessage
from learning_interface.msg import ObjectPosition  # 自定义的目标位置消息



import numpy as np 
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot 


import serial
import serial.tools.list_ports
import time

"""
创建一个订阅者节点
"""


def data_transform(data):
    data = int(data *1000)
    print(data/10)
    data=int(data)
    if data <0:
        data=data+256*256
    data_high   =data//256
    data_low    =data%256
    return [data_high,data_low]


    
class SerialPort():
    def __init__(self):

        self.serial_port = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self.data = None
        self.reiceive()
    
    def reiceive(self):
        data = []
        response =None
        size = self.serial_port.inWaiting()
        
        if  size>0:
            response = self.serial_port.read(8)
            #print(response)
        if response is not None:
            if response[0]==179 and  response[1]==179 :
                print("in")
                data=[response[2],response[3],response[4],response[5],response[6]]
                self.data = data
                
        self.serial_port.flushInput()
            
    def Send_message(self,t265_x,t265_y,t265_z,flag_t):
        #self.get_logger().info(str([-t265_y,t265_x,t265_z]))
        mysum=0
        t265_x_high,t265_x_low=data_transform(-t265_y)
        t265_y_high,t265_y_low=data_transform(t265_x)
        t265_z_high,t265_z_low=data_transform(t265_z)
        #print(t265_x_high,t265_x_low)
        temp = bytearray([0xb3,0xb3, t265_x_high,t265_x_low,t265_y_high,t265_y_low,t265_z_high,t265_z_low,flag_t,0x5b])
        for i in range(2,8):    
            mysum=mysum+temp[i]
        mysum=mysum%256
        s=bytearray([mysum])[0]
        # print('sum=',s)
        transdata=bytearray([0xb3,0xb3, t265_x_low,t265_x_high,t265_y_low,t265_y_high,t265_z_low,t265_z_high,flag_t,0x5b])
        #transdata=bytearray([0xb3,0xb3, t265_x_high,t265_x_low,t265_y_high,t265_y_low,t265_z_low,t265_z_high,flag_t,0x5b])
        print(transdata)
        self.serial_port.write(transdata)
        #print(transdata)
        time.sleep(0.002)
    
def quaternion_to_euler(qw,qx,qy,qz):
    """
    将四元数转换为欧拉角
    :param q: 四元数，格式为 (w, x, y, z)
    :return: 欧拉角，格式为 (roll, pitch, yaw)（弧度）
    """
    w, x, y, z = qw,qx,qy,qz

    # 计算欧拉角
    roll_x = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_y = np.arcsin(2.0 * (w * y - z * x))
    yaw_z = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    return roll_x, pitch_y, yaw_z

# 示例
quaternion = (0.7071, 0.0, 0.7071, 0.0)  # 示例四元数
euler_angles = quaternion_to_euler(quaternion)
print("欧拉角 (roll, pitch, yaw):", euler_angles)


class SubscriberNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.serial = SerialPort()
        self.data =self.serial.data
        self.l0_vector = np.array([-0.262,0,0])
        #the data from t265 included xyz and quaternion
        self.t2x = 0
        self.t2y = 0
        self.t2z = 0
        self.q0  = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.d435_x = 0
        self.d435_y = 0
        self.d435_z = 0
        self.flag_d = 0
        self.euler =None
        self.rot =None
        # 创建订阅者
        self.sub = self.create_subscription(\
            TFMessage, "/tf", self.T2_listener_callback, 10)       # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度
        self.sub_d435 = self.create_subscription(\
            ObjectPosition, "/d435_object_position", self.listener_callback_d435, 10)       # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度
        self.tim = self.create_timer(0.5, self.timer_callback) #to see if data is right
        
        
    def timer_callback(self):
        self.serial.reiceive()
        self.data =self.serial.data
        #print(self.data)
    def T2_listener_callback(self, msg):                                             # 创建回调函数，执行收到话题消息后对数据的处理
        #self.get_logger().info('GET t2')    # 输出日志信息，提示订阅收到的话题消息
        for transform in msg.transforms:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.r_a =None
            self.t2x = translation.x
            self.t2y = translation.y
            self.t2z = translation.z
            self.t2v = np.array([self.t2x, self.t2y, self.t2z])
            self.q0  = rotation.w
            self.q1 = rotation.x
            self.q2 = rotation.y
            self.q3 = rotation.z
            self.euler = quaternion_to_euler(self.q0,self.q1,self.q2,self.q3)
            
            vector = np.array([self.t2x, self.t2y, self.t2z])
            quaternion = np.array([self.q0, self.q1, self.q2, self.q3])
            #self.get_logger().info("t2x: %f, t2y: %f, t2z: %f" % (self.t2x, self.t2y, self.t2z))
        #    self.get_logger().info("q0: %f, q1: %f, q2: %f, q3: %f" % (self.q0, self.q1, self.q2, self.q3))
            self.Q2E_Q2R()
            self.correctT265()
    def Q2E_Q2R(self):
        quaternion = [self.q1, self.q2, self.q3, self.q0]
        r = R.from_quat(quaternion)
        self.euler=r.as_euler('xyz', degrees=True)
        #self.get_logger().info("euler: yaw%f, pitch%f, roll%f" % (self.euler[0], self.euler[1], self.euler[2]))
        self.rot = R.from_quat(quaternion).as_matrix()
        #print(self.rot)
    
    def correctT265(self):
        #print(self.l0_vector)
        r_r = np.dot(self.rot,self.l0_vector)
        r_e = np.array([self.t2x,self.t2y,self.t2z])
        
        
        self.r_a = r_r + r_e - self.l0_vector
        #print("1",r_r)
        
        #print(type(self.r_a[0]))
        po=str([int(-self.r_a[1]*100),int(self.r_a[0]*100),int(self.r_a[2]*100)])
        #self.get_logger().info(po)
        self.serial.Send_message(self.r_a[0],self.r_a[1],self.r_a[2],1)
        
    def listener_callback_d435(self, msg):                             # 创建回调函数，执行收到话题消息后对数据的处理
        self.d435_x=msg.x
        self.d435_y=msg.y
        self.d435_z=msg.z
        self.flag_d=msg.f
        print("2",msg.x)
        self.get_logger().info('Target Position: "(%d,%d,%d)"' % (self.d435_x[0], self.d435_y[0],self.d435_z[0]))
        self.get_world_point()
        
    def get_world_point(self):
        #只考虑yaw角
        #self.serial.Send_message(self.d435_x,self.d435_y,self.d435_z,self.flag_d)
        
        x_d = np.array([self.d435_x[0],self.d435_y[0],self.d435_z[0]])
        self.get_logger().info(str(self.rot))
        x_w = np.dot(self.rot,x_d) + self.t2v
        #最后为了和机身坐标统一，注意要减去l0，
        self.get_logger().info("world: %f, %f, %f" % (x_w[0], x_w[1], x_w[2]))
        

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = SubscriberNode("tft265")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口