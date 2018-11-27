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
        label,center = self.kmeans(Z,K)
        #criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        #ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]
        res2 = res.reshape((img.shape))
        res2 = cv2.resize(res2, (0,0), fx=sg, fy=sg) 
        stream_data = np.array(cv2.imencode('.jpeg', res2)[1]).tostring()
        return stream_data
 
    def kmeans(self, Z, K):
        ''' Implementation of kmeans algorithm using K centers of image Z. 
        @param[in]	Z 	image, array(:,dim).
        @param[in]	K 	number of centers, int. '''

        def assign_points(points, centers): 
            assignments = []
            for point in points:
                shortest = np.Infinity 
                shortest_index = 0
                for i in range(len(centers)):
                    x = np.linalg.norm(point - centers[i,:])
                    if x < shortest:
                        shortest = x
                        shortest_index = i
                assignments.append(shortest_index)	
            return assignments

        dimension = np.shape(Z)[1]
        centers = np.zeros((K,dimension))
        # Initialize centers randomly in range of given array. 
        min_max_list = []
        for d in range(dimension): 
            min_value = min(Z[:,d])
            max_value = max(Z[:,d])
            min_max_list.append([min_value,max_value])
        for k in range(K): 
            center = np.zeros((dimension,))
            for d in range(dimension): 
                a = min_max_list[d][0]
                b = min_max_list[d][1]
                center[d] = np.random.uniform(a,b,1)
            centers[k,:] = center
        # Assign points to initial centers.
        assign = assign_points(Z, centers)
        previous_assign = None
        while assign != previous_assign:
            # Update centers. 
            means = {}
            center = np.zeros((K,dimension))
            for a, point in zip(assign, Z):
                if not a in means.keys(): 
                    means[a] = [point]
                else: 
                    means[a].append(point)
            k = 0
            for points in means.values(): 
                centers[k,:] = np.mean(points)
                k += 1
            previous_assign = assign
            # Reassign points. 
            assign = assign_points(Z, centers)
        return assign, center.tolist()	

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
