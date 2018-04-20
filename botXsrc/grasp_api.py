from botX.components import BaseComponent
from botX.applications import external_command_pool
from botX.utils.install_util import maybe_download_git
from socketIO_client import SocketIO, BaseNamespace
from threading import Thread
import numpy as np

import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2, CameraInfo

import matplotlib.pyplot as plt
import matplotlib.pyplot as pyplot

import sys
sys.path.append('external_modules')

from haptica_util.srv import GQCNNGraspPlanner
from haptica_util.msg import GQCNNGrasp, BoundingBox


# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
# sys.path.remove('/home/ashis/catkin_ws/devel/lib/python2.7/dist-packages')
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
# sys.path.append('/home/ashis/catkin_ws/devel/lib/python2.7/dist-packages')
# from perception import CameraIntrinsics, ColorImage, DepthImage
# from perception import RgbdDetectorFactory, RgbdSensorFactory


import time
import os

class GraspAPI(BaseComponent):

    def setup(self):
        

        command = 'roslaunch gqcnn gqcnn.launch'
        self.proc_id = external_command_pool.start_command(command)

        self.buf = []
        rospy.init_node('grasp_listener')
        rospy.Subscriber("clock", Clock, self.cache_info)
        rospy.Subscriber("/camera/image_raw", Image, self.cache_info)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.cache_info)
        rospy.Subscriber("/camera/camera_info", CameraInfo, self.cache_info)

        rospy.loginfo("Waiting for GQCNN to spin up")
        rospy.wait_for_service('plan_gqcnn_grasp')
        self.plan_grasp = rospy.ServiceProxy('plan_gqcnn_grasp', GQCNNGraspPlanner)
        rospy.loginfo("GQCNN service Initialized")
        # pass

    def cache_info(self, msg):
        # print("Message: ", msg)
        if (len(self.buf) > 2000):
            self.buf.pop(0)
        self.buf.append([msg, type(msg)])
        return

    def get_camera_info(self):
        camera_info = [x[0] for x in self.buf if x[1] is CameraInfo]
        return camera_info[-1:][0]

    def get_image(self):
        image = [x[0] for x in self.buf if x[1] is Image]
        return image[-1:]

    def get_color_image(self):
        im = Image()
        for i in range(len(self.buf)): 
            im = self.get_image()[0]
            print(im.encoding)
            if im.encoding == 'rgb8':
                return im
                # break
        

    def get_depth_image(self):
        im = Image()
        for i in range(len(self.buf)): 
            im = self.get_image()[0]
            print(im.encoding)
            if im.encoding == '32FC1':
                return im
                # break
        


    def _get_bounding_box(self, object_name, color_image):
        """ Find the bounding box for the object
        params
        ---
        object_name: the string name of the object
        color_image: the color image for the object

        returns
        ---
        bounding_box: numpy array [minX, minY, maxX, maxY] in pixels around the image 
        on the depth image
        """
        return np.array([200,270,300,370]) 
        #return np.array([500,500,1000,800])

    def _bbox_to_msg(self, bbox):
        """
        Params
        ---
        bbox: numpy array [minX, minY, maxX, maxY] in pixels around the image 
        Returns
        ---
        a bondingBox message type
        """       
        boundingBox = BoundingBox()
        boundingBox.minX = bbox[0]
        boundingBox.minY = bbox[1]
        boundingBox.maxX = bbox[2]
        boundingBox.maxY = bbox[3]
        return boundingBox

    def object_to_grasp(self, object_name):

        color_image = self.get_color_image()
        depth_image = self.get_depth_image()

        bounding_box = self._get_bounding_box(object_name, color_image)
        print("Bounding Box: ", bounding_box)

        grasp_pose = self.get_grasp_plan(bounding_box, color_image, depth_image)
        print("Grasp plan completed.")
        print("Grasp pose: ", grasp_pose)
        return grasp_pose
     
    def get_grasp_plan(self, bounding_box, color_image, depth_image):
        boundingBox = self._bbox_to_msg(bounding_box)
        camera_info = self.get_camera_info()

        # rename
        color_im = color_image
        print ("Color Image enc: ", color_im.encoding)
        depth_im = depth_image
        print ("Depth Image enc: ", depth_im.encoding)

        try:
            rospy.loginfo("Sending grasp plan request to gqcnn server")
            planned_grasp_data = self.plan_grasp(color_im, depth_im, camera_info, boundingBox)

            planned_grasp_pose_msg = planned_grasp_data.grasp.pose
            grasp_succes_prob = planned_grasp_data.grasp.grasp_success_prob
            grasp = planned_grasp_data.grasp

            rospy.loginfo("Grasp service request response: {}".format(planned_grasp_data))
            return grasp

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: \n %s" % e)  
        

    def shutdown(self):
        # external_command_pool.end_command(self.proc_id)
        pass

