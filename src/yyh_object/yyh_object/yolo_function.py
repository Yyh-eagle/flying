import cv2                             
import numpy as np                      

import pyrealsense2 as rs2#python内置的d435i库
from learning_interface.msg import ObjectPosition
from Coord_Trans import locate_d4
YOLO_THRESHOLD = 0.8
def yolo_root_d4(param,yolo):
    """
    yolo的总功能函数
    """
    result=yolo_recog(param,YOLO_THRESHOLD,yolo)#threshold =0.8
    if result==None:#保证有结果的时候再坐标转换
        return None
    return locate_d4(param,result)


    
def yolo_recog(param,threshold,yolo):#yolo 
    color_d435i = param.color_d435i
    detect_result = yolo.predict(param.color_d435i)#用彩色图做预测
    predictions = detect_result.pred[0]#获得预测结果
    boxes = predictions[:, :4]  # x1, y1, x2, y2  = predictions[:,0],predictions[:,1],predictions[:,2],predictions[:,3]
    scores = predictions[:, 4]#score = predictions[:,4]
    kinds = predictions[:, 5]#cate = predictions[:,5]

    for index in range(len(kinds)):
        name = detect_result.names[int(kinds[index])]
        if name ==param.task_id:#todo定义目标字典
            x1, y1, x2, y2 = boxes[index]#find th center of the box
            probability= detect_result.pred[0][index][4]
            if probability>threshold:#置信度修改
                result=(int((x1+x2)/2),int((y1+y2)/2),int(x1),int(y1),int(x2),int(y2),probability,name_order)
                cv2.putText(param.color_d435i,str(name_order),(x1,y1 ), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                cv2.rectangle(param.color_d435i, (x1, y1), (x2, y2), (0, 255, 0), 2)
                return result#result可能为空
            else:
                return None
        else:
            return None

    

