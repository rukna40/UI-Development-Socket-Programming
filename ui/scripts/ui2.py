import rospy
import os
import sys
import csv
import cv2
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from PyQt5 import QtGui
from ui.srv import datatype
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QPushButton
)   

def to_euler_angles(x, y, z, w):

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = math.sqrt(1 + 2 * (w * y - x * z))
    cosp = math.sqrt(1 - 2 * (w * y - x * z))   
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class GUI(QMainWindow):
    def __init__(self):
        super(GUI,self).__init__()
        self.setWindowTitle("GUI for Sensor Data")
        self.setGeometry(100, 100, 800, 600)
        
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout=QVBoxLayout(self.central_widget)
        
        self.vidlabel_text = QLabel("Video Feed:\n")
        self.layout.addWidget(self.vidlabel_text)

        self.vidlabel = QLabel(self)
        self.vidlabel.setFixedSize(400, 300)        
        self.layout.addWidget(self.vidlabel)

        self.ssbutton = QPushButton("Take Screenshot")
        self.ssbutton.clicked.connect(self.screenshot)
        self.layout.addWidget(self.ssbutton)
        
        self.vid_record_button=QPushButton("Start Recording Video")
        self.vid_record_button.setCheckable(True)
        self.vid_record_button.clicked.connect(self.vid_record_toggle)
        self.layout.addWidget(self.vid_record_button)

        self.imulabel = QLabel("IMU Data:")
        self.layout.addWidget(self.imulabel)

        self.imu_toggle_button=QPushButton("Change to Euler Angles")
        self.imu_toggle_button.setCheckable(True)
        self.imu_toggle_button.clicked.connect(self.imu_toggle)
        self.layout.addWidget(self.imu_toggle_button)

        self.gpslabel = QLabel("GPS Data:")
        self.layout.addWidget(self.gpslabel)

        self.obstaclelabel = QLabel("Obstacle at:")
        self.layout.addWidget(self.obstaclelabel)
        
        self.img_sub=rospy.Subscriber("/camera/rgb/image_raw", Image, self.imagecallback)
        self.imu_sub=rospy.Subscriber("/imu_phone", Imu, self.imucallback)
        self.gps_sub=rospy.Subscriber("/gps/fix", NavSatFix, self.gpscallback)
        self.lidar_sub=rospy.Subscriber("/scan", LaserScan, self.lidarcallback)
        self.s = rospy.Service('imu_toggle', datatype, self.imu_toggle)

        self.imu_msg=None

        self.gps_msg=None
        self.img=None
        self.result=None
        
        self.ss_count=1
        self.vid_count=1

        self.output_vid=None

        self.regions=None

        self.rate = rospy.Rate(10)


            
    def imu_toggle(self, req):

        if self.imu_toggle_button.isChecked():
            r,p,y = to_euler_angles(self.imu_msg[0],self.imu_msg[1],self.imu_msg[2],self.imu_msg[3])
            imu_data = "IMU Data:\nOrientation (Euler Angles):\nx: "+str(r)+"\ny: "+str(p)+"\nz: "+str(y)+"\nAngular Velocity:\nx: "+str(self.imu_msg[4])+"\ny: "+str(self.imu_msg[5])+"\nz: "+str(self.imu_msg[6])+"\nLinear Acceleration\nx: "+str(self.imu_msg[7])+"\ny: "+str(self.imu_msg[8])+"\nz: "+str(self.imu_msg[9])
            self.imulabel.setText(imu_data)
            self.imu_toggle_button.setText("Change to Quaternions")        
            return("Changed to euler angles successfully")

        else:
            self.imulabel.setText("IMU Data :")
            imu_data = "IMU Data:\nOrientation (Quaternions):\nx: "+str(self.imu_msg[0])+"\ny: "+str(self.imu_msg[1])+"\nz: "+str(self.imu_msg[2])+"\nw: "+str(self.imu_msg[3])+"\nAngular Velocity:\nx: "+str(self.imu_msg[4])+"\ny: "+str(self.imu_msg[5])+"\nz: "+str(self.imu_msg[6])+"\nLinear Acceleration\nx: "+str(self.imu_msg[7])+"\ny: "+str(self.imu_msg[8])+"\nz: "+str(self.imu_msg[9])
            self.imulabel.setText(imu_data)
            self.imu_toggle_button.setText("Change to Euler Angles")
            return("Changed to quaternions successfully")

    def vid_record_toggle(self):
        path="/home/ankur/Pictures/gui/"

        if self.vid_record_button.isChecked():
            file="video"+str(self.vid_count)+".mp4"
            abspath=path+file
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            h, w, ch = self.img.shape
            self.output_vid = cv2.VideoWriter(abspath, fourcc, 10.0, (w, h))
            self.vid_record_button.setText("Stop Recording Video")
            
        else:
            self.output_vid.release()
            self.vid_count+=1
            self.vid_record_button.setText("Start Recording Video")
            if os.path.exists(abspath):
                print("Video saved successfully.")
            else:
                print("Error in saving video.")
            

    def screenshot(self):
        path="/home/ankur/Pictures/gui/"
        file="screenshot"+str(self.ss_count)+".jpg"
        # file="screenshot"+str(self.ss_count)+".csv"
        abspath=path+file

        cv2.imwrite(abspath, self.img)
        # arr = np.asarray(self.img)
        # img_array = (arr.flatten())
        # img_array  = img_array.reshape(-1, 1).T

        # with open(abspath, 'w') as f:
        #     np.savetxt(f, img_array, delimiter=",")
        
        if os.path.exists(abspath):
            print("Screenshot saved")
        else:
            print("Error in saving screenshot") 
        self.ss_count+=1

    def detect(self):
        obstacle = ""

        d = 1.5

        if self.regions[1] > d and self.regions[2] > d and self.regions[3] > d:
            obstacle = "None"
        elif self.regions[1] > d and self.regions[2] < d and self.regions[3] > d:
            obstacle = "Front"
        elif self.regions[1] < d and self.regions[2] > d and self.regions[3] > d:
            obstacle = "Front Right"
        elif self.regions[1] > d and self.regions[2] > d and self.regions[3] < d:
            obstacle = "Front Left"
        elif self.regions[1] < d and self.regions[2] < d and self.regions[3] > d:
            obstacle = "Front and Front Right"
        elif self.regions[1] > d and self.regions[2] < d and self.regions[3] < d:
            obstacle = "Front and Front_Left"
        elif self.regions[1] < d and self.regions[2] > d and self.regions[3] < d:
            obstacle = "Front Left and Front Right"
        elif self.regions[1] < d and self.regions[2] < d and self.regions[3] < d:
            obstacle = "Front Left and Front and Front Right"
        else:
            obstacle="Unknown"

        return obstacle
    
    def recordImuData(self):
        rows=self.imu_msg
        
        fields = ['Orientation x', 'Orientation y', 'Orientation z', 'Orientation w', 'Angular Velocity x', 'Angular Velocity y', 'Angular Velocity z', 'Linear Acceleration x', 'Linear Acceleration y', 'Linear Acceleration z'] 
        filename = "/home/ankur/Documents/MRM/data/imu_data.csv"

        with open(filename, 'a') as csvfile:

            csvwriter = csv.writer(csvfile) 
            csvwriter.writerow(fields) 
            csvwriter.writerow(rows)
    
    def recordGpsData(self):
        rows=self.gps_msg
        
        fields = ['Latitude', 'Longitude'] 
        filename = "/home/ankur/Documents/MRM/data/gps_data.csv"

        with open(filename, 'a') as csvfile:

            csvwriter = csv.writer(csvfile) 
            csvwriter.writerow(fields) 
            csvwriter.writerow(rows)

    def imagecallback(self,data):
        bridge = CvBridge()

        try:
            cv_img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        self.img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        if self.vid_record_button.isChecked():
            self.output_vid.write(self.img) 

        h, w, ch = self.img.shape
        bytes_per_line = ch * w

        qtimg = QtGui.QImage(self.img.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)

        imageMain=QtGui.QPixmap.fromImage(qtimg)
        self.vidlabel.setPixmap(imageMain.scaledToWidth(400))
        self.rate.sleep()


    def imucallback(self,data):

        self.imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w, data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z,data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z]

        rospy.wait_for_service('imu_toggle')
        try:
            imu_toggle = rospy.ServiceProxy('imu_toggle', datatype)
            response = imu_toggle()
            rospy.loginfo(response.status)        

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)        

        self.recordImuData()
        # rospy.loginfo(self.imu_msg)
        self.rate.sleep()
    
    def gpscallback(self,data):
        self.gps_msg = [data.latitude,data.longitude]
        gps_data="GPS Data:\nLatitude: "+str(self.gps_msg[0])+"\nLongitude: "+str(self.gps_msg[1])
        self.gpslabel.setText(gps_data)
        self.recordGpsData()
        self.rate.sleep()
        # rospy.loginfo(self.gps_msg)

    def lidarcallback(self,data):
        max_val = 10
        self.regions = [
            min(min(data.ranges[:108]), max_val),
            min(min(data.ranges[108:216]), max_val),
            min(min(data.ranges[216:324]), max_val),
            min(min(data.ranges[324:432]), max_val),
            min(min(data.ranges[432:540]), max_val)
        ]
        obstacle_data = "Obstacle at: "+self.detect()
        self.obstaclelabel.setText(obstacle_data)    
        # self.rate.sleep()    

def ui():
    rospy.init_node('ui_source', anonymous=True)
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        ui()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")