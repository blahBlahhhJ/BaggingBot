#! /usr/bin/python3
from typing import List
import rospy
from transport.msg import Target
from transport.srv import CamInfo, Image, ObjectIdent, ObjectIdentRequest, ObjectIdentResponse, SawyerCtrl, SawyerCtrlRequest, SawyerCtrlResponse, TurtleCtrl, TurtleCtrlRequest, TurtleCtrlResponse

# Main demo logic
# -> bootstrap everything
# -> listen for /transport_exec_errors for printout
# loop:
# -> request to precess a frame with CV ident
# -> pass along the result to sawyer node, selecting a block randomly, unless there's no block left
# -> (If there's no block left, enter this ending sequence)
#   -> select the bag
#   -> drive turtle bot to landing area
#   -> select the bot
#   -> select ot-of-range (sawyer back to idle)
#   -> end
# -> loop
# Note: turtle bot control has been deprecated
# Note: append hardcoded table object into Pick command to sawyer, like:
# pick.objects.append(Target(ident="table", dimension=[x, y, z], pose=Pose(...)))


class TransportDemo:
    def __init__(self):
        rospy.init_node("demo_node")
        # more services
        try:
            self.cv_service_name = rospy.get_param("~cv_service_name")
            self.sawyer_controller_name = rospy.get_param(
                "~sawyer_controller_name")
            # more params?
        except KeyError as e:
            print(f"[Main] Invalid Params: {str(e)}")
            exit()
        rospy.wait_for_service(self.cv_service_name)
        self.cv_service = rospy.ServiceProxy(self.cv_service_name, ObjectIdent)
        self.cv_refresh()
        rospy.wait_for_service(self.sawyer_controller_name)
        self.sawyer_controller = rospy.ServiceProxy(
            self.sawyer_controller_name, SawyerCtrl)

    def cv_refresh(self):

        self.cv_service_last_res: ObjectIdentResponse = self.cv_service.call(
            ObjectIdentRequest(frame=None))  # TODO
        if self.cv_service_last_res.info != "":
            print(f"[CV] {self.cv_service_last_res.info}")

    def main_sequence(self):
        input(f"Press [Enter] to start demo...")
        while True:
            input('[cv refresh]')
            self.cv_refresh()
            candidates: List[Target] = self.cv_service_last_res.objects
            block_sel = [obj.ident == "block" for obj in candidates]
            if sum(block_sel) == 0:
                break
            sawyer_command: SawyerCtrlResponse = self.sawyer_controller.call(
                SawyerCtrlRequest(select=block_sel.index(True), objects=candidates))
            print(
                f"[Block] {'Success' if sawyer_command.success else 'Fail'}: {sawyer_command.info}")

        self.cv_refresh()
        sawyer_bag_command: SawyerCtrlResponse = self.sawyer_controller.call(
            SawyerCtrlRequest(
                select=[
                    obj.ident == "bag" for obj in self.cv_service_last_res.objects].index(True),
                objects=self.cv_service_last_res.objects
            )
        )
        print(
            f"[Bag] {'Success' if sawyer_bag_command.success else 'Fail'}: {sawyer_bag_command.info}")
        # self.cv_refresh()
        """ turtle_staging_command: TurtleCtrlResponse = self.turtle_controller.call(
            TurtleCtrlRequest(None))
        print(
            f"[Turtle in] {'Success' if turtle_staging_command.success else 'Fail'}: {turtle_staging_command.info}")
        self.cv_refresh()
        sawyer_bot_command: SawyerCtrlResponse = self.sawyer_controller.call(
            SawyerCtrlRequest(
                select=[obj.ident == "bot" for obj in self.cv_service_last_res.objects].index(
                    True),
                objects=self.cv_service_last_res.objects
            )
        )
        print(
            f"[Sawyer -> Bot] {'Success' if sawyer_bot_command.success else 'Fail'}: {sawyer_bot_command.info}")
        turtle_away_command: TurtleCtrlResponse = self.turtle_controller.call(
            TurtleCtrlRequest(None))
        print(
            f"[Turtle away] {'Success' if turtle_away_command.success else 'Fail'}: {turtle_away_command.info}") """
        sawyer_idle_command: SawyerCtrlResponse = self.sawyer_controller.call(
            SawyerCtrlRequest(select=1, objects=[])
        )
        print(
            f"{'Success' if sawyer_idle_command.success else 'Fail'}: {sawyer_idle_command.info}")
        if input("End of demo, loop? ('y')") == 'y':
            return self.main_sequence()
        else:
            return self

    def shutdown():
        pass


if __name__ == '__main__':
    TransportDemo().main_sequence().shutdown()
