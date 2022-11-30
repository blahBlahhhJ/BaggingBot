#! /usr/bin/env python
import argparse

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION


def f(side):
    limb = intera_interface.Limb(side)
    joints = limb.joint_names()
    angles = []
    angles = {}
    for j in joints:
        angles[j] = float(input(j + ': '))
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        limb.set_joint_positions(angles)
        r.sleep()
    



def main():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    f(valid_limbs[0])
    print("Done.")


if __name__ == '__main__':
    main()