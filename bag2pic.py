#!/usr/bin/env python
#coding:utf-8

from cv2 import CV_16UC1, imshow
import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np

path = "picOutput/" #存储图像的地址

bag_file = "rgbd_dataset_freiburg1_desk.bag"
bag = rosbag.Bag(bag_file, "r")   # 载入bag文件
bag_data = bag.read_messages()    # 利用迭代器返回三个值：{topic标签, msg数据, t时间戳}

bridge = CvBridge()
for topic, msg, t in bag_data:
    if topic == "/camera/depth/image":
      # print(msg.format)
      cv_image = bridge.imgmsg_to_cv2(msg, "32FC1")  # 这里要注意图像的格式 
      print(cv_image.shape)                          # 避免图像是Nonetype
      image = np.array(cv_image, dtype=np.float)
      image=image*5000 # unit: m to mm factor 16:5000 32:1
      NewImg = np.round(image).astype(np.uint16)
      #转为为16位图像
      # output16 = cv2.cvtColor(cv_image,CV_16UC1)
      # gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
      # cv2.imshow("Image window", NewImg)
      # cv2.waitKey(0)
      timestr = "%.6f" %  msg.header.stamp.to_sec()  # %.6f表示小数点后带有6位，可根据精确度需要修改；
      image_name = timestr+ ".png"                   # 图像命名：时间戳.png
      print(image_name)
      cv2.imwrite(path+ "depth/"+image_name, NewImg)        # 保存；
        
    if topic == "/camera/rgb/image_color":
      cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")  # 这里要注意图像的格式 
      print(cv_image.shape)                          # 避免图像是Nonetype


      # cv2.imshow("Image window", cv_image)
      # cv2.waitKey(0)
      timestr = "%.6f" %  msg.header.stamp.to_sec()  # %.6f表示小数点后带有6位，可根据精确度需要修改；
      image_name = timestr+ ".png"                   # 图像命名：时间戳.png
      cv2.imwrite(path+ "rgb/"+image_name, cv_image)        # 保存；
      

