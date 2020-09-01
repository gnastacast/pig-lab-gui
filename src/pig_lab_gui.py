#!/usr/bin/env python
from __future__ import division, print_function
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QThread
import sys
import os
import rospy
from gl_widget import CvWidget
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, PointStamped, WrenchStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from datetime import datetime
import rosbag
from copy import deepcopy
from threading import Lock


class QRosThread(QThread):
    # Qt thread designed to allow ROS to run in the background
    def __init__(self, parent = None):
        QThread.__init__(self, parent)
        # Initialize the node
        if rospy.get_node_uri() == None:
            rospy.init_node("vtk_test")
        self.rate = rospy.Rate(100) # 100hz
    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()
    def update(self):
        pass

class Ui(QtWidgets.QMainWindow):
    bridge = CvBridge()
    def __init__(self, savePath, hasMicron = False):
        super(Ui, self).__init__()
        self.hasMicron = hasMicron
        if self.hasMicron:
            uiPath = os.path.join(os.path.dirname(__file__), 'piglabgui.ui')
        else:
            uiPath = os.path.join(os.path.dirname(__file__), 'piglabgui_robot.ui')
        uic.loadUi(uiPath, self)
        # set up recording
        self.bag = None
        self.savePath = savePath
        self.recording = False
        self.trialNo = 1
        self.lock = Lock()
        self.startTime = None
        # Set up ultrasound image
        layout = QtWidgets.QHBoxLayout()
        self.cvWidget = CvWidget(parent=self)
        self.cvWidget.timerEvent = self.timerEvent
        layout.addWidget(self.cvWidget)
        self.ui_cvFrame.setLayout(layout)
        # Set up subscribers
        self.needleTopic = "/BioImpedance"
        self.ultrasoundTopic = "/ultrasound/image_raw"
        self.ultrasoundInfoTopic = "/ultrasound/camera_info"
        rospy.Subscriber(self.needleTopic, PointStamped, self.needleCb)
        rospy.Subscriber(self.ultrasoundTopic, Image, self.ultrasoundCb)
        rospy.Subscriber(self.ultrasoundInfoTopic, CameraInfo, self.infoCb)
        # self.robotScriptPub = rospy.Publisher("/ur_hardware_interface/script_command", String, 100)
        if self.hasMicron:
            self.markerVisibility = {'REF':        [False, self.ui_refValid],
                                     'Ultrasound': [False, self.ui_ultrasoundValid],
                                     'Needle_001': [False, self.ui_needleValid]}

            for marker in self.markerVisibility.keys():
                topic = "/micron/%s/measured_cp" % marker
                rospy.Subscriber(topic, PoseStamped, self.record, topic)
                rospy.Subscriber(topic + "_valid", Bool, self.setValid, marker)
        else:
            self.markerVisibility = {'Robot': [False, self.ui_robotValid]}
            self.robotTopic = "/pose_ee"
            rospy.Subscriber(self.robotTopic, PoseStamped, self.record, self.robotTopic)
            topic = "/wrench"
            rospy.Subscriber(topic, WrenchStamped, self.record, topic)
            self.timeout = rospy.Timer(rospy.Duration(0.1), self.checkStamp)
            self.lastStamp = rospy.Time();
        # set up the UI
        # self.ui_freeDrive.clicked.connect(self.setFreeDrive)
        self.ui_startUltrasound.clicked.connect(self.startUltrasoundRecording)
        self.ui_startBioimpedance.clicked.connect(self.startBioimpedanceRecording)
        self.ui_stopUltrasound.clicked.connect(self.stop)
        self.ui_stopBioimpedance.clicked.connect(self.stop)
        self.ui_filePath.setText(unicode(self.savePath))
        self.show()

    def timerEvent(self, event):
        for marker in self.markerVisibility.values():
            marker[1].setStyleSheet("background-color:%s;" % ('green' if marker[0] else 'red'))
        if self.startTime is None:
            self.ui_duration.setText("Duration: 00:00")
        else:
            time = (rospy.Time.now() - self.startTime).secs
            mins = str(int(time) // 60).zfill(2)
            secs = str(int(time) - int(mins) * 60).zfill(2)
            self.ui_duration.setText("Duration: %s:%s"%(mins, secs))
        self.ui_trialNo.setText("Trial # %i" % self.trialNo)
        # Update changes
        self.update()
        self.cvWidget.update()

    def checkStamp(self, event):
        robotValid = False
        if event.last_duration:
            robotValid = self.lastStamp > event.last_real 
        self.setValid(Bool(robotValid), 'Robot')

    def setValid(self, msg, marker):
        self.markerVisibility[marker][0] = msg.data

    def needleCb(self, msg):
        self.record(msg, self.needleTopic)
        string = unicode("Needle Data: %.3f kOhms %.2f deg" % (msg.point.x * 10 ** (msg.point.y-3), msg.point.z))
        self.needleData.setText(string)

    def ultrasoundCb(self, msg):
        self.record(msg, self.ultrasoundTopic)
        self.cvWidget.setImage(self.bridge.imgmsg_to_cv2(msg), bgr=False)

    def infoCb(self, msg):
        self.record(msg, self.ultrasoundInfoTopic)

    def setFreeDrive(self):
        self.robotScriptPub.publish(String("def prog():\nfreedrive_mode()\nsleep(10)\nend"))

    def stop(self):
        self.recording=False
        if self.bag is None:
            return
        self.lock.acquire()
        self.bag.close()
        self.lock.release()
        self.bag = None
        self.startTime = None
        self.trialNo = self.trialNo + 1
        print("Finished Recording")

    def startUltrasoundRecording(self):
        self.start("Ultrasound")

    def startBioimpedanceRecording(self):
        self.start("Bioimpedance")

    def errorMessage(self, error):
       msg = QtWidgets.QMessageBox()
       msg.setIcon(QtWidgets.QMessageBox.Warning)
       msg.setText("Bag directory does not exist")
       msg.setWindowTitle("Error")
       msg.setDetailedText("The details are as follows: %s" % (error))
       msg.setStandardButtons(QtWidgets.QMessageBox.Ok)
       retval = msg.exec_()

    def start(self, name):
        if self.recording:
            return
        self.startTime = rospy.Time.now()
        date = datetime.utcfromtimestamp(rospy.Time.now().secs).strftime('%Y-%m-%d--%H-%M-%S')
        self.lock.acquire()
        bagPath = os.path.join(str(self.ui_filePath.text()),'%s-%s-Trial-%i.bag' % (date, name, self.trialNo))
        try:
            self.bag = rosbag.Bag(bagPath, 'w')
        except IOError as e:
            self.errorMessage(e)
            self.lock.release()
            return
        self.lock.release()
        print("Recording to %s" % bagPath)
        self.recording=True

    def record(self, msg, topic):
        if self.recording:
            self.lock.acquire()
            try:
                self.bag.write(topic, msg)
            except Exception as e:
                rospy.logwarn("Failed to write to bag")
            self.lock.release()
        if not self.hasMicron and topic == self.robotTopic:
            self.lastStamp = msg.header.stamp

if __name__ == '__main__':
    import signal
    app = QtWidgets.QApplication(sys.argv)
    def quit(*args):
        app.quit()
    signal.signal(signal.SIGINT, quit)
    rosThread = QRosThread()
    rosThread.start()
    window = Ui('/media/tracir/Seagate Desktop Drive/roboTRAC/Lab_02')
    app.exec_()