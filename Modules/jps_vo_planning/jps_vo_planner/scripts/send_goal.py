#!/usr/bin/env python3
"""Publish the goal of a scenario variant once, on the topic the RViz 3D Nav Goal tool uses.

  rosrun jps_vo_planner send_goal.py _scenario:=$(rospack find jps_vo_planner)/config/scenarios/scenario_static_only.yaml \
         _variant:=S2_north
  rosrun jps_vo_planner send_goal.py _goal:="[0.0, 14.0, 1.5]"
"""
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dyn_obstacles import load_scenario  # noqa: E402


def main():
    rospy.init_node('send_goal', anonymous=True)
    topic = rospy.get_param('~topic', '/uav1/prometheus/motion_planning/goal')
    goal = rospy.get_param('~goal', None)
    if goal is None:
        cfg = load_scenario(rospy.get_param('~scenario'), rospy.get_param('~variant', ''))
        goal = cfg['uav']['goal']
    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=True)
    t_end = rospy.get_time() + 5.0
    while pub.get_num_connections() == 0 and rospy.get_time() < t_end and not rospy.is_shutdown():
        rospy.sleep(0.05)
    if pub.get_num_connections() == 0:
        rospy.logerr('no subscriber on %s (is the planner running?)', topic)
        sys.exit(1)
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world'
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [float(v) for v in goal]
    msg.pose.orientation.w = 1.0
    pub.publish(msg)
    rospy.loginfo('goal (%.2f, %.2f, %.2f) published on %s at t0 = %.3f', *goal, topic, msg.header.stamp.to_sec())
    rospy.sleep(0.5)


if __name__ == '__main__':
    main()
