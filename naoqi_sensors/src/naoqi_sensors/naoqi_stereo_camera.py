#!/usr/bin/env python

#
# ROS node to provide access to the camera by wrapping NaoQI access (may not
# be the most efficient way...)
#
# Copyright 2012 Daniel Maier, University of Freiburg
# Copyright 2015 Aldebaran Robotics
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

from collections import defaultdict
from distutils.version import LooseVersion
import rospy
from sensor_msgs.msg import Image, CameraInfo
from naoqi_driver.naoqi_node import NaoqiNode
import camera_info_manager

from dynamic_reconfigure.server import Server
from naoqi_sensors.cfg import NaoqiStereoCameraConfig

# import resolutions
from naoqi_sensors.vision_definitions import k960p, k4VGA, kVGA, kQVGA, kQQVGA
# import color spaces
from naoqi_sensors.vision_definitions import kYUV422ColorSpace, kYUVColorSpace, \
                    kRGBColorSpace, kBGRColorSpace, kDepthColorSpace, kRawDepthColorSpace
# import extra parameters
from naoqi_sensors.vision_definitions import kCameraSelectID, kCameraAutoExpositionID, kCameraAecAlgorithmID, \
                  kCameraContrastID, kCameraSaturationID, kCameraHueID, kCameraSharpnessID, kCameraAutoWhiteBalanceID, \
                  kCameraExposureID, kCameraAutoGainID, kCameraGainID, kCameraBrightnessID, kCameraWhiteBalanceID

# those should appear in vision_definitions.py at some point
kLeftCamera = 0
kRightCamera = 1

class NaoqiStereoCam (NaoqiNode):
    def __init__(self, node_name='naoqi_stereo_camera'):
        NaoqiNode.__init__(self, node_name)

        self.camProxy = self.get_proxy("ALVideoDevice")
        if self.camProxy is None:
            exit(1)
        self.nameId = None
        self.camera_infos = {}
        def returnNone():
            return None
        self.config = defaultdict(returnNone)

        # ROS publishers
        self.pub_img_left_ = rospy.Publisher('left/image_raw', Image, queue_size=5)
        self.pub_info_left_ = rospy.Publisher('left/camera_info', CameraInfo, queue_size=5)
        self.pub_img_right_ = rospy.Publisher('right/image_raw', Image, queue_size=5)
        self.pub_info_right_ = rospy.Publisher('right/camera_info', CameraInfo, queue_size=5)

        # initialize the parameter server
        self.srv = Server(NaoqiStereoCameraConfig, self.reconfigure)

        # initial load from param server
        self.init_config()

        # initially load configurations
        self.reconfigure(self.config, 0)

    def init_config( self ):
        # mandatory configurations to be set
        self.config['frame_rate'] = rospy.get_param('~frame_rate')
        self.config['resolution'] = rospy.get_param('~resolution')
        self.config['color_space'] = rospy.get_param('~color_space')

        # optional for camera frames
        # top camera with default
        if rospy.has_param('~camera_left_frame'):
            self.config['camera_left_frame'] = rospy.get_param('~camera_left_frame')
        else:
            self.config['camera_left_frame'] = "/CameraLeft_optical_frame"
        # bottom camera with default
        if rospy.has_param('~camera_right_frame'):
            self.config['camera_right_frame'] = rospy.get_param('~camera_right_frame')
        else:
            self.config['camera_right_frame'] = "/CameraRight_optical_frame"

        #load calibration files
        if rospy.has_param('~calibration_file_left'):
            self.config['calibration_file_left'] = rospy.get_param('~calibration_file_left')
        else:
            rospy.loginfo('no camera calibration for top camera found')

        if rospy.has_param('~calibration_file_right'):
            self.config['calibration_file_right'] = rospy.get_param('~calibration_file_right')
        else:
            rospy.loginfo('no camera calibration for top camera found')

        # set time reference
        if rospy.has_param('~use_ros_time'):
            self.config['use_ros_time'] = rospy.get_param('~use_ros_time')
        else:
            self.config['use_ros_time'] = False

        self.load_camera_info()

    def load_camera_info( self ):
        cim = camera_info_manager.CameraInfoManager(cname='nao_camera')
        cim.setURL(self.config['calibration_file_left'])
        cim.loadCameraInfo()
        self.infomsg_left_ = cim.getCameraInfo()

        cim.setURL(self.config['calibration_file_right'])
        cim.loadCameraInfo()
        self.infomsg_right_ = cim.getCameraInfo()

    def reconfigure( self, new_config, level ):
        """
        Reconfigure the camera
        """
        rospy.loginfo('reconfigure changed')
        if self.pub_img_left_.get_num_connections() == 0 and self.pub_img_right_.get_num_connections() == 0:
            rospy.loginfo('Changes recorded but not applied as nobody is subscribed to the ROS topics.')
            self.config.update(new_config)
            return self.config

        # check if we are even subscribed to a camera
        is_camera_new = self.nameId is None

        if is_camera_new:
            rospy.loginfo('subscribed to camera proxy, since this is the first camera')
            self.nameId = self.camProxy.subscribeCameras("rospy_gvm", [kLeftCamera, kRightCamera],
                                                         [new_config['resolution']]*2, [new_config['color_space']]*2,
                                                         new_config['frame_rate'])

        # set params
        camParams = self.extractParams(new_config)
        self.setParams(camParams)

        key_methods =  [ ('resolution', 'setResolutions'), ('color_space', 'setColorSpaces')]
        for key, method in key_methods:
            if self.config[key] != new_config[key] or is_camera_new:
                self.camProxy.__getattribute__(method)(self.nameId, [new_config[key]]*2)

        key_methods = [('frame_rate', 'setFrameRate')]
        for key, method in key_methods:
            if self.config[key] != new_config[key] or is_camera_new:
                self.camProxy.__getattribute__(method)(self.nameId, new_config[key])

        self.config.update(new_config)

        return self.config

    def extractParams(self, new_config):
        camParams = []

        camParams.append( (kCameraAecAlgorithmID, new_config['auto_exposure_algo']) )
        camParams.append( (kCameraContrastID, new_config['contrast']) )
        camParams.append( (kCameraSaturationID, new_config['saturation']) )
        camParams.append( (kCameraHueID, new_config['hue']) ) # Migth be deprecated
        camParams.append( (kCameraSharpnessID, new_config['sharpness']) )

        camParams.append( (kCameraAutoWhiteBalanceID, new_config['auto_white_balance']) )
        if ( new_config['auto_white_balance']==0):
            camParams.append( (kCameraWhiteBalanceID, new_config['white_balance']) )

        camParams.append( (kCameraAutoExpositionID, new_config['auto_exposition']) )
        if ( new_config['auto_exposition']==0):
            camParams.append( (kCameraGainID, new_config['gain']) )
            camParams.append( (kCameraExposureID, new_config['exposure']) )
        else:
            camParams.append( (kCameraBrightnessID, new_config['brightness']) )

        return camParams

    def setParams(self, key_list):
        for key, value in key_list:
            if self.get_version() < LooseVersion('2.0'):
                self.camProxy.setParam(key, value)
            else:
                self.camProxy.setCameraParameter(self.nameId, key, value)

    def run(self):
        img = Image()
        r = rospy.Rate(self.config['frame_rate'])

        while self.is_looping():
            if self.pub_img_left_.get_num_connections() == 0 and self.pub_img_right_.get_num_connections() == 0:
                if self.nameId:
                    rospy.loginfo('Unsubscribing from camera as nobody listens to the topics.')
                    self.camProxy.unsubscribe(self.nameId)
                    self.nameId = None
                r.sleep()
                continue
            if self.nameId is None:
                self.reconfigure(self.config, 0)
                r.sleep()
                continue
            images = self.camProxy.getImagesRemote(self.nameId)
            if images is None:
                continue

            now = rospy.Time.now()
            for index in [kLeftCamera, kRightCamera]:
              image = images[index]
              # Deal with the image
              if self.config['use_ros_time']:
                  img.header.stamp = now
              else:
                  img.header.stamp = rospy.Time(image[4] + image[5]*1e-6)
              img.height = image[1]
              img.width = image[0]
              nbLayers = image[2]
              if image[3] == kYUVColorSpace:
                  encoding = "mono8"
              elif image[3] == kRGBColorSpace:
                  encoding = "rgb8"
              elif image[3] == kBGRColorSpace:
                  encoding = "bgr8"
              elif image[3] == kYUV422ColorSpace:
                  encoding = "yuv422" # this works only in ROS groovy and later
              else:
                  rospy.logerr("Received unknown encoding: {0}".format(image[3]))
              img.encoding = encoding
              img.step = img.width * nbLayers
              img.data = image[6]

              if index == kLeftCamera:
                img.header.frame_id = self.config['camera_left_frame']
                self.pub_img_left_.publish(img)
                self.infomsg_left_.header = img.header
                self.pub_info_left_.publish(self.infomsg_left_)
              else:
                img.header.frame_id = self.config['camera_right_frame']
                self.pub_img_right_.publish(img)
                self.infomsg_right_.header = img.header
                self.pub_info_right_.publish(self.infomsg_right_)

            r.sleep()

        if (self.nameId):
            rospy.loginfo("unsubscribing from cameras ")
            self.camProxy.unsubscribe(self.nameId)
