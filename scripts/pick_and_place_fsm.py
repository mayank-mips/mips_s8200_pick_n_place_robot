#!/usr/bin/env python3
"""Pick-and-place finite state machine.

States:
  WAIT_DETECTION -> APPROACH_PICK -> DESCEND_PICK -> GRASP -> LIFT ->
  APPROACH_DROP -> DESCEND_DROP -> RELEASE -> RETREAT -> DONE

Sends Cartesian goals on /motion_goal (handled by motion_ik.py) and toggles
suction via the /vacuum/grasp service. Pick pose comes from the detector; the
drop pose and offsets are configurable in config/pick_and_place.yaml.
"""
import copy
import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool


class PickPlaceFSM:
    def __init__(self):
        rospy.init_node('pick_and_place_fsm')
        p = rospy.get_param('pick_and_place')
        self.base_frame = p['base_frame']
        self.approach_height = p['approach_height']
        self.grasp_depth = p['grasp_depth']
        self.drop = p['drop_pose']
        self.tool = p['tool_orientation']
        self.settle = p['settle_time']
        self.grasp_time = p['grasp_time']

        self.object_pose = None
        rospy.Subscriber(p['object_topic'], PoseStamped, self.obj_cb,
                         queue_size=1)
        self.goal_pub = rospy.Publisher('/motion_goal', PoseStamped,
                                        queue_size=1)
        self.motion_done = False
        rospy.Subscriber('/motion_result', PoseStamped, self.result_cb,
                         queue_size=1)

        rospy.loginfo('Waiting for vacuum service...')
        rospy.wait_for_service('/vacuum/grasp')
        self.vacuum = rospy.ServiceProxy('/vacuum/grasp', SetBool)
        rospy.loginfo('pick_and_place_fsm ready.')

    def obj_cb(self, msg):
        if self.object_pose is None:
            self.object_pose = msg

    def result_cb(self, _):
        self.motion_done = True

    def _make_pose(self, x, y, z):
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = self.tool['x']
        ps.pose.orientation.y = self.tool['y']
        ps.pose.orientation.z = self.tool['z']
        ps.pose.orientation.w = self.tool['w']
        return ps

    def _move(self, pose, label):
        rospy.loginfo('[FSM] moving to %s: (%.3f, %.3f, %.3f)', label,
                      pose.pose.position.x, pose.pose.position.y,
                      pose.pose.position.z)
        self.motion_done = False
        self.goal_pub.publish(pose)
        # Wait for motion ack (with timeout)
        t0 = rospy.Time.now()
        while not self.motion_done and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > 30.0:
                rospy.logwarn('[FSM] motion timed out for %s', label)
                break
            rospy.sleep(0.1)
        rospy.sleep(self.settle)

    def run(self):
        rate = rospy.Rate(10)
        rospy.loginfo('[FSM] WAIT_DETECTION')
        while self.object_pose is None and not rospy.is_shutdown():
            rate.sleep()
        if rospy.is_shutdown():
            return

        obj = self.object_pose.pose.position
        rospy.loginfo('[FSM] object located at (%.3f, %.3f, %.3f)',
                      obj.x, obj.y, obj.z)

        # APPROACH_PICK
        self._move(self._make_pose(obj.x, obj.y, obj.z + self.approach_height),
                   'APPROACH_PICK')
        # DESCEND_PICK
        self._move(self._make_pose(obj.x, obj.y, obj.z + 0.02),
                   'DESCEND_PICK')
        # GRASP
        rospy.loginfo('[FSM] GRASP (suction on)')
        self.vacuum(True)
        rospy.sleep(self.grasp_time)
        # LIFT
        self._move(self._make_pose(obj.x, obj.y, obj.z + self.approach_height),
                   'LIFT')
        # APPROACH_DROP
        self._move(self._make_pose(self.drop['x'], self.drop['y'],
                                   self.drop['z'] + self.approach_height),
                   'APPROACH_DROP')
        # DESCEND_DROP
        self._move(self._make_pose(self.drop['x'], self.drop['y'],
                                   self.drop['z'] + 0.03), 'DESCEND_DROP')
        # RELEASE
        rospy.loginfo('[FSM] RELEASE (suction off)')
        self.vacuum(False)
        rospy.sleep(self.grasp_time)
        # RETREAT
        self._move(self._make_pose(self.drop['x'], self.drop['y'],
                                   self.drop['z'] + self.approach_height),
                   'RETREAT')
        rospy.loginfo('[FSM] DONE')


if __name__ == '__main__':
    try:
        PickPlaceFSM().run()
    except rospy.ROSInterruptException:
        pass
