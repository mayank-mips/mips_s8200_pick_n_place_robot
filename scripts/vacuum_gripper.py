#!/usr/bin/env python3
"""Vacuum gripper control.

The actual attach/detach physics is handled by gazebo_grasp_plugin
(libgazebo_grasp_fix.so, declared in suction_tip.xacro), which grips any
object the suction tip is in contact with and releases it when contact is
lost. This node provides a clean ROS service to signal grasp intent and, if
you prefer explicit control, can be extended to call Gazebo's attach/detach
services directly.

Provides:
  /vacuum/grasp  (std_srvs/SetBool)  True = enable suction, False = release
"""
import rospy
from std_srvs.srv import SetBool, SetBoolResponse


class VacuumGripper:
    def __init__(self):
        rospy.init_node('vacuum_gripper')
        self.engaged = False
        self.srv = rospy.Service('/vacuum/grasp', SetBool, self.handle)
        rospy.loginfo('vacuum_gripper ready. Call /vacuum/grasp (SetBool).')

    def handle(self, req):
        self.engaged = req.data
        # With gazebo_grasp_plugin, attachment is automatic on contact. Here we
        # just track intent and report. To force attach/detach, call the
        # Gazebo link-attacher services instead (see README).
        state = 'ENGAGED (suction on)' if req.data else 'RELEASED'
        rospy.loginfo('Vacuum %s', state)
        return SetBoolResponse(success=True, message=state)


if __name__ == '__main__':
    try:
        VacuumGripper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
