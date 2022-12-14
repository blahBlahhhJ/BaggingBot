#! /usr/bin/env python
import rospy
import tf2_ros as tf
import image_geometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import cv2
import numpy as np

import ros_numpy
from sensing.srv import ImageSrv, CamInfoSrv, BagSrv, BagSrvResponse


class BagServer:
    def __init__(self):
        rospy.init_node('bag_server')

        if not self._load_parameters():
            return

        rospy.Service(self._bag_service, BagSrv, self.get_centroid)
        self.marker_pub = rospy.Publisher(
            'object_marker', Marker, queue_size=10)

        # setup tf
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

    def run(self):
        rospy.spin()

    def get_centroid(self, _):
        # dependency services
        rospy.wait_for_service(self._image_service)
        rospy.wait_for_service(self._caminfo_service)

        centroid = self._get_centroid()
        return BagSrvResponse(centroid)

    def _load_parameters(self):
        try:
            self._image_service = rospy.get_param('services/image')
            self._caminfo_service = rospy.get_param('services/caminfo')
            self._bag_service = rospy.get_param('services/bag')

            self._base_frame = rospy.get_param('frames/base')
            self._head_frame = rospy.get_param('frames/head')
            self._table_frame = rospy.get_param('frames/table')

            self._bag_height = rospy.get_param('~bag_height')
            self._bag_width = rospy.get_param('~bag_width')
            self._bag_length = rospy.get_param('~bag_length')
            self._bag_handle_offset = rospy.get_param('~bag_handle_offset')

            return True

        except rospy.ROSException as e:
            print(e)
            return False

    def _get_centroid(self):
        try:
            # cv stuff
            img = self._get_image()
            # np.save('/home/cc/ee106a/fa22/class/ee106a-abi/ros_workspaces/proj/assets/bag.npy', img)
            contour = self._process_contours(img)
            if contour is None:
                return []

            centroids2D, centroids3D = self._process_centroids(contour)

            # self.cv_debug(img, contour, centroids2D)

        except KeyboardInterrupt:
            print('keyboard interrupt')
            cv2.destroyAllWindows()
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

        # this is se3
        return ros_numpy.numpify(self.tf_buffer.lookup_transform(target, source, t).transform)

    def _get_image(self):
        image_proxy = rospy.ServiceProxy(self._image_service, ImageSrv)
        return ros_numpy.numpify(image_proxy().image_data)

    def _process_contours(self, img):
        # blur
        blur = cv2.medianBlur(img, 9)

        # saturation
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        value = 100
        lim = 255 - value
        s[s > lim] = 255
        s[s <= lim] += value
        hsv = cv2.merge((h, s, v))

        # debug = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        # cv2.imshow("debug_hsv", debug)
        # cv2.waitKey(0)

        # threshold
        hue_lo, hue_hi = rospy.get_param('~hue_lo'), rospy.get_param('~hue_hi')
        sat_lo, sat_hi = rospy.get_param('~sat_lo'), rospy.get_param('~sat_hi')
        val_lo, val_hi = rospy.get_param('~val_lo'), rospy.get_param('~val_hi')
        hued = cv2.inRange(hsv, (hue_lo, sat_lo, val_lo),
                           (hue_hi, sat_hi, val_hi))

        # debug = cv2.cvtColor((hued[:, :, None] != 0) * hsv, cv2.COLOR_HSV2BGR)
        # cv2.imshow("debug_threshold", debug)
        # cv2.waitKey(0)

        # mask
        x_lo, x_hi = rospy.get_param('~x_lo'), rospy.get_param('~x_hi')
        y_lo, y_hi = rospy.get_param('~y_lo'), rospy.get_param('~y_hi')
        table_mask = np.zeros_like(hued)
        table_mask[x_lo:x_hi, y_lo:y_hi] = 1
        print(table_mask.shape, hued.shape)
        masked = table_mask * hued

        # debug = cv2.cvtColor((table_mask[:, :, None] != 1) * hsv, cv2.COLOR_HSV2BGR)
        # cv2.imshow("debug_mask", debug)
        # cv2.waitKey(0)

        # contour
        contours, _ = cv2.findContours(
            masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # pick the largest contour

        # assert len(contours) > 0
        if len(contours) == 0:
            return None

        sortedContours = sorted(contours, key=cv2.contourArea)
        maxAreaContour = sortedContours[-1]

        return maxAreaContour

    def _process_centroids(self, contour):
        centroids2D = None
        centroids3D = None

        caminfo_proxy = rospy.ServiceProxy(self._caminfo_service, CamInfoSrv)
        caminfo = caminfo_proxy().cam_info

        headcam = image_geometry.PinholeCameraModel()
        headcam.fromCameraInfo(caminfo)

        self.head2base = self._get_transform(
            self._head_frame, self._base_frame)
        self.table2base = self._get_transform(
            self._table_frame, self._base_frame)

        origin = np.array([0, 0, 0, 1])
        normal = np.array([0, 0, 1, 0])

        p = head_origin = (self.head2base @ origin)[:3]
        x = table_origin = (self.table2base @ origin)[:3]
        x[2] += self._bag_height - 0.05  # use table+bag height plane
        w = table_normal = (self.table2base @ normal)[:3]
        b = -w @ x  # table equation: wx + b = 0

        i = 1
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centroids2D = (cX, cY)

        rx, ry, rz = headcam.projectPixelTo3dRay(
            headcam.rectifyPoint((cX, cY)))
        head_r = np.array([rx, ry, rz, 0])
        # ray equation: x = p + lam * r
        r = ray = (self.head2base @ head_r)[:3]

        # solve for ray-table intersection: w(p + lambda * r) + b = 0
        lam = (-b - w@p) / (w@r)
        # TODO: more verification?
        assert lam >= 0
        centroid = p + lam * r

        centroids3D = Point(centroid[0] + self._bag_handle_offset + self._bag_width / 2,
                            centroid[1], x[2] - self._bag_height / 2 + 0.05)  # point on the handle; cv point

        # self._generate_bag_handle(centroids3D, i)
        # self._generate_bag(centroids3D, i)
        # self._generate_ray(Point(head_origin[0], head_origin[1], head_origin[2]), centroids3D, i)

        return centroids2D, centroids3D

    def _generate_bag(self, point, i):

        bag_center = Point(point.x, point.y - self._bag_handle_offset *
                           2 - self._bag_height / 2, point.z - self._bag_width / 2)

        marker = Marker()
        marker.header.frame_id = self._base_frame
        marker.header.stamp = rospy.get_rostime()
        marker.pose.position = bag_center
        marker.ns = 'bag'
        marker.id = i
        marker.type = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._bag_length
        marker.scale.y = self._bag_height
        marker.scale.z = self._bag_width

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _generate_ray(self, start, end, i):
        marker = Marker()
        marker.header.frame_id = self._base_frame
        marker.header.stamp = rospy.get_rostime()
        marker.points = [start, end]
        marker.ns = 'ray'
        marker.id = i
        marker.type = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.03
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _generate_bag_handle(self, point, i):

        handle_center = Point(
            point.x, point.y - self._bag_handle_offset, point.z)

        marker = Marker()
        marker.header.frame_id = self._base_frame
        marker.header.stamp = rospy.get_rostime()
        marker.pose.position = handle_center
        marker.ns = 'bag_handle'
        marker.id = i
        marker.type = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = self._bag_handle_offset * 2
        marker.scale.z = 0.005

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def cv_debug(self, img, contours, centroids2D):
        cv2.drawContours(img, contours, -1, (0, 255, 0, 2))

        cv2.circle(img, centroids2D, 1, (0, 0, 255), 2)

        cv2.imshow("debug", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = BagServer()
    if node:
        node.run()
