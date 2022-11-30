#! /usr/bin/env python
"""
    Copied from lab4
"""

import rospy
from sensor_msgs.msg import Image, CameraInfo
from intera_core_msgs.srv._IOComponentCommandSrv import IOComponentCommandSrv
from intera_core_msgs.msg._IOComponentCommand import IOComponentCommand

from sensing.srv import ImageSrv, ImageSrvResponse, CamInfoSrv, CamInfoSrvResponse

class CameraServer:
    
    def __init__(self):
        rospy.init_node('camera_server')

        rospy.Subscriber("/io/internal_camera/head_camera/image_rect_color", Image, self.imgReceived)
        rospy.Subscriber("/io/internal_camera/head_camera/camera_info", CameraInfo, self.camInfoReceived)

        rospy.Service('image', ImageSrv, self.getLastImage)
        rospy.Service('caminfo', CamInfoSrv, self.getLastCamInfo)
    
    def run(self):
        self.activate_head()
        rospy.spin()

    def imgReceived(self, message):
        self.lastImage = message
    
    def camInfoReceived(self, message):
        self.lastCamInfo = message

    def getLastImage(self, request):
        return ImageSrvResponse(self.lastImage)

    def getLastCamInfo(self, request):
        return CamInfoSrvResponse(self.lastCamInfo)

    def activate_head(self, timeout=0.0):
        rospy.wait_for_service('/io/internal_camera/head_camera/command')
        try:
            head_cam_control = rospy.ServiceProxy('/io/internal_camera/head_camera/command', IOComponentCommandSrv)
            cmd = IOComponentCommand()
            cmd.time = rospy.Time.now()
            cmd.op = 'set'
            cmd.args = '{"signals": {"camera_streaming": {"data": [true], "format": {"type": "bool"}}}}'
            
            resp = head_cam_control(cmd, timeout)
            print(resp)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)





if __name__ == "__main__":
    node = CameraServer()
    node.run()
    