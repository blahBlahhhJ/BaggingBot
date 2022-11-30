"""
    Copied from lab4
"""

import rospy
from sensor_msgs.msg import Image, CameraInfo

from sensing.srv import ImageSrv, ImageSrvResponse, CamInfoSrv, CamInfoSrvResponse

class CameraServer:
    
    def __init__(self):
        rospy.init_node('camera_server')

        rospy.Subscriber("/io/internal_camera/head_camera/image_rect_color", Image, self.imgReceived)
        rospy.Subscriber("/io/internal_camera/head_camera/camera_info", CameraInfo, self.camInfoReceived)

        rospy.Service('image', ImageSrv, self.getLastImage)
        rospy.Service('caminfo', CamInfoSrv, self.getLastCamInfo)
    
    def run(self):
        rospy.spin()

    def imgReceived(self, message):
        self.lastImage = message
    
    def camInfoReceived(self, message):
        self.lastCamInfo = message

    def getLastImage(self, request):
        return ImageSrvResponse(self.lastImage)

    def getLastCamInfo(self, request):
        return CamInfoSrvResponse(self.lastCamInfo)


if __name__ == "__main__":
    node = CameraServer()
    node.run()
    