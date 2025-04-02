import cv2                             
import numpy as np                      

import pyrealsense2 as rs2#python内置的d435i库
from learning_interface.msg import ObjectPosition


def locate_yolo_d4(param,yolo):

    angle = param.D435i_yaw
    position = ObjectPosition()
    d_y, d_x = param.depth_d435i.shape[0:2] #dy = depth_image.shape[0],the number of the line
    c_y, c_x = param.color_d435i.shape[0:2]
    rate=d_x/c_x
    #cv2.putText(param.color_d435i,"ininin",(50,50 ), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

    results=yolo_recog(param.color_d435i,0.8,yolo)#threshold =0.8
    for object in results:
        kind_order=object[7]
        if kind_order == param.task_id:#todo到时候会有任务id到任务的具体映射

            real_x_int,real_y_int,real_z_int=GetD435iObject(object,rate,param)#得到目标的T265坐标
            position.x[index], position.y[index],position.z[index] = real_x_int,real_y_int,real_z_int
            position.f,position.kind=1,kind_order
            cv2.putText(param.color_d435i,str(kind_order),(x1,y1 ), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            cv2.rectangle(param.color_d435i, (x1, y1), (x2, y2), (0, 255, 0), 2)
            return position
    #todo还要写出独立测试程序


def locate_yolo_usb(self,color_image):#todo小相机的yolo识别

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
    cv2.imshow("usb",color_image)                          # 使用OpenCV显示处理后的图像效果
    cv2.waitKey(1)
    
def yolo_recog(img,threshold,yolo):#yolo 
    detect_result = yolo.predict(img)#predict
    predictions = detect_result.pred[0]#结果
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
        if probability>threshold:#置信度修改
            pack=(int((x1+x2)/2),int((y1+y2)/2),int(x1),int(y1),int(x2),int(y2),probability,name_order)
            result.append(pack)#turple

    return result

def GetD435iObject(object,rate,param):
    center_x_int=int(rate*object[0])#中心点坐标
    center_y_int=int(rate*object[1])
    (x1,y1,x2,y2) = object[2:6]#目标矩形框的两个端点，用于画图
    probability=object[6]#置信度
    real_z=param.depth_d435i[center_y_int,center_x_int]#目标深度值获取
    camera_coordinate = rs2.rs2_deproject_pixel_to_point(param.intrinsics, [int(object[0]),int(object[1])], real_z)
    (real_x,real_y,real_z) = camera_coordinate[0:3]#直接小孔成像模型计算了

    #先假设都是45度#todo少一个根据角度计算到T265坐标系的转换函数#todoT265坐标系和相机坐标系的方向
    real_y_int=int(real_x)
    real_x_int=int((real_y-real_z)*0.707)
    real_z_int=int(-(real_z+real_y)*0.707)

    return real_x_int-337,real_y_int,real_z_int