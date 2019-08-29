#!/usr/bin/env python
# coding: utf-8

# In[1]:


import torch
from torch import nn
import torch.backends.cudnn as cudnn
from torch.utils.data import Dataset, DataLoader
from torch.utils.data.sampler import SubsetRandomSampler
from torch.optim.lr_scheduler import CosineAnnealingLR, MultiStepLR
import torchvision
from torchvision import transforms, utils, datasets

import os
import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import h5py
import PIL
import numpy as np
import random
import logging
import gdown
from zipfile import ZipFile
import matplotlib.pyplot as plt


# In[2]:


MODELS_ROOT = './models'


# In[8]:


class TrailNet3Class(nn.Module):
    def __init__(self):
        super(TrailNet3Class, self).__init__()
        # 1 input image channel, 6 output channels, 5x5 square convolution
        # kernel
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
        # print 'x size: ', x.size()   
        x = x.view(-1, self.num_flat_features(x))
        # print 'x size: ', x.size()   
        x = self.fc1(x)
        x = self.fc2(x)
        return x

    def num_flat_features(self, x):
        size = x.size()[1:]  # all dimensions except the batch dimension
        num_features = 1
        for s in size:
            num_features *= s
        return num_features


# In[9]:


def load_pretrained_model(model, filename, use_cuda=True):
    if filename.lower().endswith('.h5'):
        state_dict = h5py.File(os.path.join(MODELS_ROOT,filename), 'r')
        model.load_state_dict({l : torch.from_numpy(np.array(v)).view_as(p)                      for k, v in state_dict.items()                      for l, p in model.named_parameters() if k in l})
        
    elif filename.lower().endswith('.pth'):
        state_dict = torch.load(os.path.join(MODELS_ROOT, filename))
        model.load_state_dict(state_dict)
        
    if torch.cuda.is_available() and use_cuda:
        cudnn.benchmark = True
        model.cuda()


# In[5]:


model = TrailNet3Class()
load_pretrained_model(model, 'trailnet_3class_real_vr_mix_color.h5')


# In[13]:


model.state_dict()


# In[ ]:





# In[ ]:




