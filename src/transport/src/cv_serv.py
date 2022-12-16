#! /usr/bin/python3
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Point, Quaternion
from transport.msg import Target
from transport.srv import ObjectIdent, ObjectIdentRequest, ObjectIdentResponse
from sensing.srv import ObjectSrv, BagSrv
from std_msgs.msg import String
import actionlib
import rospy
import roslib
roslib.load_manifest('transport')


class CVService(object):
    def handler(self, _: ObjectIdentRequest) -> ObjectIdentResponse:
        try:
            objects = []
            # bag
            bag = Target(
                ident="bag",
                dimension=Vector3(
                    0.15, 0.3, 0.33  # TODO
                ),
                pose=Pose(position=self.bag_proxy().bag,
                          orientation=Quaternion(0, 0, 0, 1))
            )
            objects.append(bag)

            for block in self.object_proxy().objects:
                block_obj = Target(
                    ident="block",
                    dimension=Vector3(
                        0.04, 0.04, 0.04
                    ),
                    pose=Pose(position=block,
                              orientation=Quaternion(0, 0, 0, 1))
                )
                objects.append(block_obj)

            # table
            table = Target(
                ident="table",
                dimension=Vector3(3, 3, 0.01),
                pose=Pose(position=Point(0.738, 0.243, -0.149),
                          orientation=Quaternion(0, 0, 0, 1))
            )
            objects.append(table)

            return ObjectIdentResponse(info="", objects=objects)
        except Exception as e:
            return ObjectIdentResponse(info=f"[CV] Exception: {str(e)}", objects=[])

    def __init__(self):  # params?
        rospy.init_node("cv_node")
        rospy.wait_for_service(rospy.get_param('services/object'))
        rospy.wait_for_service(rospy.get_param('services/bag'))

        try:
            pass
        except KeyError as e:
            print(f"[CV] Invalid Params: {str(e)}")
            exit()
        rospy.Service("/cv_service", ObjectIdent, self.handler)

        self.object_proxy = rospy.ServiceProxy(
            rospy.get_param('services/object'), ObjectSrv)
        self.bag_proxy = rospy.ServiceProxy(
            rospy.get_param('services/bag'), BagSrv)

        rospy.spin()

    def shutdown(self):
        pass  # gracefully exit


if __name__ == '__main__':
    CVService()  # parse params
