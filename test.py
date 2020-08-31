import glob, os
import rosbag
import csv
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import string

root_path = '/home/innovations/BagFiles/2018-10-06-19-01-54/'
os.chdir(root_path)
bagFileList = glob.glob('*.bag')

bridge = CvBridge()

for bagFile in bagFileList:
    bag = rosbag.Bag(bagFile)
    sub_path = root_path + bagFile[0:-7]
    os.makedirs(sub_path)
    
    nameList = 'scan'
    fileName = sub_path+'/'+nameList+'.csv'
    with open(fileName, 'w+') as csvfile:
        filewriter = csv.writer(csvfile,delimiter=',')
        filewriter.writerow(['rostimestamp','rosntimestamp','data'])
        for topic, msg, t in bag.read_messages('/right/scan'):
            timeValue = t.secs
            ntimeValue = t.nsecs
            dataValue = msg.ranges
            filewriter.writerow([timeValue, ntimeValue, dataValue])
    csvfile.close()
    
    nameList = 'image_090'
    fileName = sub_path+'/'+nameList+'.csv'
    n = 0
    os.makedirs(sub_path+'/image_090/')
    with open(fileName, 'w+') as csvfile:
        filewriter = csv.writer(csvfile,delimiter=',')
        filewriter.writerow(['rostimestamp','rosntimestamp','data'])
        for topic, msg, t in bag.read_messages('/rtsp_1/image_raw'):
            timeValue = t.secs
            ntimeValue = t.nsecs
            dataValue = bridge.imgmsg_to_cv2(msg)
            imagename = root_path + bagFile[0:-7] + '/image_090/' + np.str(n) + '.png'
            cv2.imwrite(imagename, dataValue)
            n += 1
            filewriter.writerow([timeValue, ntimeValue, n])
    csvfile.close()
    
    nameList = 'image_120'
    fileName = sub_path+'/'+nameList+'.csv'
    n = 0
    os.makedirs(root_path+bagFile[0:-7]+'/image_120/')
    with open(fileName, 'w+') as csvfile:
        filewriter = csv.writer(csvfile,delimiter=',')
        filewriter.writerow(['rostimestamp','rosntimestamp','data'])
        for topic, msg, t in bag.read_messages('/rtsp_2/image_raw'):
            timeValue = t.secs
            ntimeValue = t.nsecs
            dataValue = bridge.imgmsg_to_cv2(msg)
            imagename = root_path + bagFile[0:-7] + '/image_120/' + np.str(n) + '.png'
            cv2.imwrite(imagename, dataValue)
            n += 1
            filewriter.writerow([timeValue, ntimeValue, n])
    csvfile.close()

    nameList = 'can0'
    fileName = sub_path+'/'+nameList+'.csv'
    with open(fileName, 'w+') as csvfile:
        filewriter = csv.writer(csvfile,delimiter=',')
        filewriter.writerow(['rostimestamp','rosntimestamp','data'])
        for topic, msg, t in bag.read_messages('/can0/received_messages'):
            timeValue = t.secs
            ntimeValue = t.nsecs
            idValue = msg.id
            msgString = str(msg)
            msgList = string.split(msgString, '\n')
            dataList = string.split(msgList[-1], ':')
            dataValue = string.strip(dataList[1])
            data = string.split(dataValue,',')
            data_split = []
            for data_value in data:
                valueString = string.strip(data_value,'[ ]')
                data_split.append(valueString)
            filewriter.writerow([timeValue, ntimeValue, idValue, data_split[0], data_split[1], data_split[2], data_split[3], data_split[4], data_split[5], data_split[6], data_split[7]])
    csvfile.close()
    
    nameList = 'can1'
    fileName = sub_path+'/'+nameList+'.csv'
    with open(fileName, 'w+') as csvfile:
        filewriter = csv.writer(csvfile,delimiter=',')
        filewriter.writerow(['rostimestamp','rosntimestamp','data'])
        for topic, msg, t in bag.read_messages('/can1/received_messages'):
            timeValue = t.secs
            ntimeValue = t.nsecs
            idValue = msg.id
            msgString = str(msg)
            msgList = string.split(msgString, '\n')
            dataList = string.split(msgList[-1], ':')
            dataValue = string.strip(dataList[1])
            data = string.split(dataValue,',')
            data_split = []
            for data_value in data:
                valueString = string.strip(data_value,'[ ]')
                data_split.append(valueString)
            filewriter.writerow([timeValue, ntimeValue, idValue, data_split[0], data_split[1], data_split[2], data_split[3], data_split[4], data_split[5], data_split[6], data_split[7]])
    csvfile.close()

    nameList = 'can2'
    fileName = sub_path+'/'+nameList+'.csv'
    with open(fileName, 'w+') as csvfile:
        filewriter = csv.writer(csvfile,delimiter=',')
        filewriter.writerow(['rostimestamp','rosntimestamp','data'])
        for topic, msg, t in bag.read_messages('/can2/received_messages'):
            timeValue = t.secs
            ntimeValue = t.nsecs
            idValue = msg.id
            msgString = str(msg)
            msgList = string.split(msgString, '\n')
            dataList = string.split(msgList[-1], ':')
            dataValue = string.strip(dataList[1])
            data = string.split(dataValue,',')
            data_split = []
            for data_value in data:
                valueString = string.strip(data_value,'[ ]')
                data_split.append(valueString)
            filewriter.writerow([timeValue, ntimeValue, idValue, data_split[0], data_split[1], data_split[2], data_split[3], data_split[4], data_split[5], data_split[6], data_split[7]])
    csvfile.close()
    
    bag.close()
    
