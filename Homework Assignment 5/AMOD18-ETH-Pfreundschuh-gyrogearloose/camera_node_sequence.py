#!/usr/bin/env python
import io
import thread
import numpy as np
import cv2
import yaml
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped
from duckietown_utils import get_duckiefleet_root
from picamera import PiCamera
from picamera.array import PiRGBArray
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse


class CameraNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.framerate_high = self.setupParam("~framerate_high", 30.0)
        self.framerate_low = self.setupParam("~framerate_low", 15.0)
        self.res_w = self.setupParam("~res_w", 640)
        self.res_h = self.setupParam("~res_h", 480)

        self.image_msg = CompressedImage()

        # Setup PiCamera
        self.k = rospy.get_param("~k")
        self.reduction = rospy.get_param("~reduction")

        self.camera = PiCamera()
        self.framerate = self.framerate_high  # default to high
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w, self.res_h)

        # For intrinsic calibration
        self.cali_file_folder = get_duckiefleet_root() + "/calibrations/camera_intrinsic/"

        self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"

        self.has_published = False
        self.pub_img = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1)
        self.sub_switch_high = rospy.Subscriber("~framerate_high_switch", BoolStamped, self.cbSwitchHigh, queue_size=1)

        # Create service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info", SetCameraInfo, self.cbSrvSetCameraInfo)

        self.stream = io.BytesIO()

        #self.camera.exposure_mode = 'off'
        # self.camera.awb_mode = 'off'

        self.is_shutdown = False
        self.update_framerate = False
        # Setup timer
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbSwitchHigh(self, switch_msg):
        print switch_msg
        if switch_msg.data and self.framerate != self.framerate_high:
            self.framerate = self.framerate_high
            self.update_framerate = True
        elif not switch_msg.data and self.framerate != self.framerate_low:
            self.framerate = self.framerate_low
            self.update_framerate = True

    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." % (self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            gen = self.grabAndPublish(self.stream, self.pub_img)
            try:
                self.camera.capture_sequence(gen, 'jpeg', use_video_port=True, splitter_port=0)
            except StopIteration:
                pass
            # print "updating framerate"
            self.camera.framerate = self.framerate
            self.update_framerate = False

        self.camera.close()
        rospy.loginfo("[%s] Capture Ended." % (self.node_name))

    def kmeans_algo(self, stream_data):

        sg = self.reduction
        isg = 1/float(sg)
        K = self.k
        img = np.fromstring(stream_data, np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        img = cv2.resize(img, (0,0), fx=isg, fy=isg)
        Z = img.reshape((-1,3))
        Z = np.float32(Z)

        center = self.init_step(Z,K)
        searching = True
        while(searching):
            label = self.assignment_step(center,Z,K)
            center_old = center
            center = self.update_step(label,Z,K)
            changed = False
            for i in range(0,np.shape(center)[0]):
                if(not np.array_equal(center_old[i],center[i])):
                    changed= True
                    break
            if(not changed):
                searching=False

        label = label.astype(int)
        res = center[label.flatten()]
        res2 = res.reshape((img.shape))
        res2 = cv2.resize(res2, (0,0), fx=sg, fy=sg)

        stream_data = np.array(cv2.imencode('.jpeg', res2)[1]).tostring()

        return stream_data

    def update_step(self,label,Z,K):
        center=np.zeros((K,3))
        for i in range(0,K):
            c = np.zeros((1,3))
            n = 0
            for j in range(0,np.shape(Z)[0]):
                if(label[j]==i):
                    c = c+Z[j]
                    n = n+1

            if n!=0:
                c=np.divide(c,n)
            center[i]=c
        return center

    def init_step(self,Z,K):
        center = np.zeros((K,3))
        for i in range(0,K):
            j = np.random.rand(1)*np.shape(Z)[0]
            center[i] = Z[int(j)]
        return center

    def assignment_step(self,center,Z,K):

        label= np.zeros((np.shape(Z)[0],1))
        for i in range(0,np.shape(Z)[0]):
            K_min=0
            min_dist = np.Inf
            for j in range(0,K):
                dist0 = Z[i,0]-center[j,0]
                dist1 = Z[i,1]-center[j,1]
                dist2 = Z[i,2]-center[j,2]
                dist = np.sqrt(dist0*dist0 + dist1*dist1 + dist2*dist2)
                if (dist<min_dist):
                    K_min=j
                    min_dist=dist
            label[i]=K_min
        return label

    def grabAndPublish(self, stream, publisher):
        while not self.update_framerate and not self.is_shutdown and not rospy.is_shutdown():
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()
            # Generate compressed image
            image_msg = CompressedImage()
            stream_data = self.kmeans_algo(stream_data)

            image_msg.data = stream_data
            image_msg.format = "jpeg"

            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            publisher.publish(image_msg)

            # Clear stream
            stream.seek(0)
            stream.truncate()

            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." % (self.node_name))
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." % (self.node_name))
        self.is_shutdown = True
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

    def cbSrvSetCameraInfo(self, req):
        # TODO: save req.camera_info to yaml file
        rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info, filename)
        response.status_message = "Write to %s" % filename  #TODO file name
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        rospy.loginfo("[saveCameraInfo] filename: %s" % (filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
        'image_height': camera_info_msg.height,
        'camera_name': rospy.get_name().strip("/"),  #TODO check this
        'distortion_model': camera_info_msg.distortion_model,
        'distortion_coefficients': {'data': camera_info_msg.D, 'rows':1, 'cols':5},
        'camera_matrix': {'data': camera_info_msg.K, 'rows':3, 'cols':3},
        'rectification_matrix': {'data': camera_info_msg.R, 'rows':3, 'cols':3},
        'projection_matrix': {'data': camera_info_msg.P, 'rows':3, 'cols':4}}

        rospy.loginfo("[saveCameraInfo] calib %s" % (calib))

        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False


if __name__ == '__main__':
    rospy.init_node('camera', anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    thread.start_new_thread(camera_node.startCapturing, ())
    rospy.spin()
