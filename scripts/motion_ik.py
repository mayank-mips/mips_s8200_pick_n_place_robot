#!/usr/bin/env python3
"""Inverse-kinematics motion service.

Exposes a simple ROS service-like interface (via a topic + MoveIt) to move the
arm's end effector (suction_tip) to a target pose in the base frame.

Two backends are provided:
  1. MoveIt (preferred): full planning with collision awareness.
  2. KDL fallback: lightweight numerical IK publishing straight to the
     joint position controllers. Enable with ~use_moveit:=false.

The FSM calls move_to_pose(PoseStamped) -> bool.
"""
import rospy
from geometry_msgs.msg import PoseStamped

USE_MOVEIT = True
try:
    import moveit_commander  # noqa
except Exception:
    USE_MOVEIT = False


class MoveItBackend:
    def __init__(self):
        import moveit_commander
        import sys
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = rospy.get_param('~move_group', 'manipulator')
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_pose_reference_frame(
            rospy.get_param('~base_frame', 'base_link'))
        self.group.set_end_effector_link(
            rospy.get_param('~eef_link', 'suction_tip'))
        self.group.set_planning_time(5.0)
        self.group.set_max_velocity_scaling_factor(0.3)
        rospy.loginfo('MoveIt backend ready (group=%s).', group_name)

    def move_to_pose(self, pose_stamped):
        self.group.set_pose_target(pose_stamped.pose)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return bool(ok)


class KDLBackend:
    """Minimal KDL IK fallback. Requires PyKDL + urdf kinematics.

    This is intentionally compact; for production use prefer MoveIt or trac_ik.
    """
    def __init__(self):
        from std_msgs.msg import Float64
        self.Float64 = Float64
        self.ns = rospy.get_param('~namespace', '/fanuc_gazebo')
        self.pubs = [
            rospy.Publisher('%s/joint%d_position_controller/command'
                            % (self.ns, i + 1), Float64, queue_size=1)
            for i in range(6)]
        rospy.logwarn('Using KDL fallback backend. Implement solve_ik() for '
                      'your robot or switch to MoveIt.')

    def solve_ik(self, pose_stamped):
        # TODO: plug in a real IK solver (PyKDL ChainIkSolverPos_LMA or
        # trac_ik_python). Returns a list of 6 joint angles or None.
        raise NotImplementedError(
            'KDL IK not implemented. Use MoveIt (default) or add a solver.')

    def move_to_pose(self, pose_stamped):
        angles = self.solve_ik(pose_stamped)
        if angles is None:
            return False
        for pub, a in zip(self.pubs, angles):
            pub.publish(self.Float64(a))
        rospy.sleep(2.0)
        return True


class MotionNode:
    def __init__(self):
        rospy.init_node('motion_ik')
        use_moveit = rospy.get_param('~use_moveit', USE_MOVEIT)
        self.backend = MoveItBackend() if use_moveit else KDLBackend()
        # Simple request/response over topics for the FSM.
        self.result_pub = rospy.Publisher('/motion_result', PoseStamped,
                                          queue_size=1)
        rospy.Subscriber('/motion_goal', PoseStamped, self.goal_cb,
                         queue_size=1)
        rospy.loginfo('motion_ik ready.')

    def goal_cb(self, pose):
        ok = self.backend.move_to_pose(pose)
        rospy.loginfo('motion_ik: move %s',
                      'succeeded' if ok else 'FAILED')
        # Echo the goal back on success as a lightweight ack.
        if ok:
            self.result_pub.publish(pose)


if __name__ == '__main__':
    try:
        MotionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
