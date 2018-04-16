from botX.components import BaseComponent
from botX.applications import external_command_pool
from botX.utils.install_util import maybe_download_git
from socketIO_client import SocketIO, BaseNamespace
from threading import Thread

# from perception import CameraIntrinsics, ColorImage, DepthImage
# from perception import RgbdDetectorFactory, RgbdSensorFactory


import time
import os

class GraspAPI(BaseComponent):

    def setup(self):
        command = 'roslaunch gqcnn gqcnn.launch'
        self.proc_id = external_command_pool.start_command(command)
        
     

    def shutdown(self):
        external_command_pool.end_command(self.proc_id)

