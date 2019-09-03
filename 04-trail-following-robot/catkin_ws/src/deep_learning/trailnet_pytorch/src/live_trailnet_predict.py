#!/usr/bin/python
# -*- coding: UTF-8 -*-

import os
import sys
import random
import cv2
import PIL
import numpy as np
import h5py
import gdown
import time

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import torch
from torch import nn
import torch.backends.cudnn as cudnn
import torchvision
from torchvision import transforms

'''
    Pytorch code
'''

class TrailNet3Class(nn.Module):
    def __init__(self):
        super(TrailNet3Class, self).__init__()
        # 1 input image channel, 6 output channels, 5x5 square convolution
        self.conv1 = nn.Conv2d(3, 32, 4)
        self.pool1 = nn.MaxPool2d((2, 2), stride=2)
        self.conv2 = nn.Conv2d(32, 32, 4)
        self.pool2 = nn.MaxPool2d((2, 2), stride=2)
        self.conv3 = nn.Conv2d(32, 32, 4)
        self.pool3 = nn.MaxPool2d((2, 2), stride=2)
        self.conv4 = nn.Conv2d(32, 32, 4, padding=(2, 2))
        self.pool4 = nn.MaxPool2d((2, 2), stride=2)
        # an affine operation: y = Wx + b
        self.fc1 = nn.Linear(800, 200)
        self.fc2 = nn.Linear(200, 3)

    def forward(self, x):
        x = self.pool1(self.conv1(x))
        x = self.pool2(self.conv2(x))
        x = self.pool3(self.conv3(x)) 
        x = self.pool4(self.conv4(x))    
        x = x.view(-1, self.num_flat_features(x))
        x = self.fc1(x)
        x = self.fc2(x)
        return x

    def num_flat_features(self, x):
        size = x.size()[1:]  # all dimensions except the batch dimension
        num_features = 1
        for s in size:
            num_features *= s
        return num_features



class TrailnetNode(object):
    """docstring for TrailnetPredictNode"""
    def __init__(self):
        self.bridge = CvBridge()
        rospy.on_shutdown(self.shutdown_cb)

        # FLAG!!
        self.flag_auto = False
        self.old_btn_state = 0

        # ROS subscriber
        self.sub_image = rospy.Subscriber('image_raw', Image, self.image_cb)
        self.sub_joy = rospy.Subscriber('joy', Joy, self.joy_cb)
        self.pub_nav = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # ROS get parameters
        MODEL_NAME = rospy.get_param('~model', 'trailnet_3class_epoch0_loss0.02158.pth')
        MODEL_URL = rospy.get_param('~model_url', 'https://drive.google.com/uc?id=1lZrKBSLlZJ5GC0aNNDEi57SGG_fq1M6R')
        USE_CUDA = rospy.get_param('~use_cuda', True)
        CLASS_URL = 'https://drive.google.com/uc?id=1QrUIUSQSCSagcMgV_1RnQjQKqRhuaBFf'
        INPUT_IMG_SIZE = (101, 101)

        
        # Check the model folder is existing
        pkg_path = rospkg.RosPack().get_path('trailnet_pytorch')
        if not os.path.isdir(os.path.join(pkg_path, 'models')):
            os.mkdir(os.path.join(pkg_path, 'models'))
        # Prepare the model file
        model_path = os.path.join(pkg_path, 'models', MODEL_NAME)
        if not os.path.isfile(model_path):
            gdown.download(MODEL_URL, output=model_path, quiet=False)
        # Prepare the class file
        class_path = os.path.join(pkg_path, 'class_id.txt')
        if not os.path.isfile(class_path):
            gdown.download(CLASS_URL, output=class_path, quiet=False)
        self.CLASSES = np.loadtxt(class_path, str, delimiter='\n')

        # Pytorch image transformer
        self.data_transform = transforms.Compose([ 
                                transforms.ToPILImage(),
                                transforms.Resize(INPUT_IMG_SIZE), \
                                transforms.ToTensor(), \
                                transforms.Normalize(mean=[0.5, 0.5, 0.5], \
                                                     std=[1, 1, 1]), \
                                ])

        # Load TrailNet model
        rospy.loginfo('Loading model...')
        self.model = TrailNet3Class()
        self.load_pretrained_model(self.model, model_path, use_cuda=USE_CUDA)
        self.model.eval()
        time.sleep(5)
        rospy.loginfo('TrailNet Model has been loaded.')
        

        with torch.no_grad():
            rospy.loginfo('Testing model by input a blank image...')
            blank_image = np.zeros((480, 640, 3), np.uint8)
            input_image = self.data_transform(blank_image).unsqueeze(0)
            output_idx = np.argmax(self.model(input_image.cuda()).cpu())
            output_idx = np.argmax(self.model(input_image.cuda()).cpu())
            # print self.CLASSES[output_idx]
        rospy.loginfo('TrailNet is ready.')
    

    def joy_cb(self, msg):
        # Joystick button 'B' can start/stop auto navigation mode
        if self.old_btn_state == 0 and msg.buttons[1] == 1:
            if self.flag_auto == False:
                self.flag_auto = True
                rospy.loginfo('Start auto-navigation mode.')
                # time.sleep(1)
            else:
                self.flag_auto = False
                self.pub_nav.publish(Twist())
                self.pub_nav.publish(Twist())
                self.pub_nav.publish(Twist())
                rospy.loginfo('Change to joystick control mode.')

        self.old_btn_state = msg.buttons[1]

        
    def image_cb(self, msg):
        if self.flag_auto:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            except CvBridgeError as e:
                print(e)

            with torch.no_grad():
                input_image = self.data_transform(cv_image).unsqueeze(0)
                output_idx = np.argmax(self.model(input_image.cuda()).cpu())
                prediction = self.CLASSES[output_idx]
                
                # Car command 
                cmd_msg = Twist()
                if prediction == 'S':
                    cmd_msg.linear.x = 0.3
                    cmd_msg.angular.z = 0.0
                elif prediction == 'L':
                    cmd_msg.linear.x = 0.2
                    cmd_msg.angular.z = -0.5
                elif prediction == 'R':
                    cmd_msg.linear.x = 0.2
                    cmd_msg.angular.z = 0.5
                
                self.pub_nav.publish(cmd_msg)
            

    def shutdown_cb(self):
        rospy.loginfo("Shutdown " + rospy.get_name() + '!')
        if hasattr(self, 'model'):
            del self.model
        if hasattr(self, 'pub_nav'):
            self.pub_nav.publish(Twist())


    def load_pretrained_model(self, model, file_path, use_cuda):
        if file_path.lower().endswith('.h5'):
            state_dict = h5py.File(file_path, 'r')
            model.load_state_dict({l : torch.from_numpy(np.array(v)).view_as(p) for k, v in state_dict.items() for l, p in model.named_parameters() if k in l})
            
        elif file_path.lower().endswith('.pth'):
            state_dict = torch.load(file_path)
            model.load_state_dict(state_dict)
            
        if torch.cuda.is_available() and use_cuda:
            cudnn.benchmark = True
            model.cuda()


if __name__ == '__main__':
    rospy.init_node("trailnet_prediction_node", anonymous=False)
    node = TrailnetNode()
    rospy.spin()