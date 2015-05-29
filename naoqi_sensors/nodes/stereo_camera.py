#!/usr/bin/env python
import rospy
from naoqi_sensors.naoqi_stereo_camera import NaoqiStereoCam

if __name__ == "__main__":
  naocam = NaoqiStereoCam()
  naocam.start()
  rospy.spin()
