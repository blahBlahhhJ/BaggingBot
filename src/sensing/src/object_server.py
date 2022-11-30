import rospy
import tf2_ros as tf
import image_geometry
import ros_numpy
from geometry_msgs.msg import Point

import cv2
import numpy as np

from util import *
from sensing.srv import ImageSrv, CamInfoSrv, ObjectSrv, ObjectSrvResponse


class ObjectServer:
    def __init__(self):
        rospy.init_node('object_server')
        rospy.Service('objects', ObjectSrv, self.get_centroids)

        # setup tf
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
    
    def run(self):
        rospy.spin()

    def get_centroids(self, _):
        centroids = self._get_centroids()
        return ObjectSrvResponse(centroids)

    def _get_centroids(self):
        # dependency services
        rospy.wait_for_service('image')
        rospy.wait_for_service('caminfo')

        try:
            # cv stuff
            img = self._get_image()
            contours = self._process_contours(img)
            centroids2D, centroids3D = self._process_centroids(contours)

            self.cv_debug(img, contours, centroids2D)

        except KeyboardInterrupt:
            print('keyboard interrupt')
            cv2.destroyAllWindows()
            return []
        except rospy.ServiceException as e:
            print(e)
            return []
        except tf.LookupException:
            print('lookup exception!')
            return []
        except tf.ConnectivityException:
            print('connectivity exception!')
            return []
        except tf.ExtrapolationException:
            print('extrapolation exception!')
            return []
        except Exception as e:
            print('unrecognized error')
            print(e)
            return []

        return centroids3D

    def _get_transform(self, source, target):
        t = rospy.Time()
        r = rospy.Rate(5)
        print('spinning for tf transform:', source, target)
        while not self.tf_buffer.can_transform(target, source, t):
            t = rospy.Time()
            r.sleep()
        print('retrieved tf transform:', source, target)
        
        return ros_numpy.numpify(self.tf_buffer.lookupTransform(target, source, t).transform) # this is se3

    def _get_image():
        image_proxy = rospy.ServiceProxy('image', ImageSrv)
        return ros_numpy.numpify(image_proxy().image_data)
    
    def _process_contours(img):
        # blur
        blur = cv2.medianBlur(img, 9)

        # saturation
        hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)
        h, s, v = cv2.split(hsv)
        value = 100
        lim = 255 - value
        s[s > lim] = 255
        s[s <= lim] += value
        hsv = cv2.merge((h, s, v))

        # threshold
        hue = 70
        hue_range = 20
        hued = cv2.inRange(hsv, (hue-hue_range, 80, 80), (hue+hue_range, 255, 255))

        # contour
        _, contours, hierarchy = cv2.findContours(hued, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        thresh = 25
        filtered = [cnt for cnt in contours if cv2.contourArea(cnt) > thresh]
        print('got', str(len(filtered)), 'contours')
        return filtered

    def _process_centroids(self, contours):
        centroids2D = []
        centroids3D = []

        caminfo_proxy = rospy.ServiceProxy('caminfo', CamInfoSrv)
        caminfo = caminfo_proxy().cam_info

        headcam = image_geometry.PinholeCameraModel()
        headcam.fromCameraInfo(caminfo)

        self.head2base = self._get_transform('head_camera', 'base')
        self.table2base = self._get_transform('table', 'base')

        origin = np.array([0, 0, 0, 1])
        normal = np.array([0, 0, 1, 1])

        p = head_origin = (self.head2base @ origin)[:3]
        x = table_origin = (self.table2base @ origin)[:3]
        w = table_normal = (self.table2base @ normal)[:3]
        b = -w @ x  # table equation: wx + b = 0

        for contour in contours:
            M = cv2.moments(contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centroids2D.append((cX, cY))

            r = ray = (self.head2base @ headcam.projectPixelTo3dRay(headcam.rectifyPoint((cX, cY))))[:3]   # ray equation: x = p + lam * r

            # solve for ray-table intersection: w(p + lambda * r) + b = 0
            lam = (-b - w@p) / (w@r)
            # TODO: more verification?
            assert lam >= 0
            centroid = p + lam * r

            point = Point(centroid[0], centroid[1], centroid[2])
            centroids3D.append(point)

        return centroids3D

    def cv_debug(img, contours, centroids2D):
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        cv2.drawContours(img_bgr, contours, -1, (0, 255, 0, 2))

        for centroid in centroids2D:
            cv2.circle(img_bgr, centroid, 1, (0, 0, 255), 2)

        cv2.imshow("debug", img_bgr)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    node = ObjectServer()
    node.run()