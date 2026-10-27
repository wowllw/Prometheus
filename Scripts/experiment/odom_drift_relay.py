#!/usr/bin/env python3
"""Odometry drift relay for the DASC experiments (new, independent helper).

Subscribes to the true odometry, adds a seeded random-walk position drift and writes a matching
position covariance into pose.covariance, then republishes it for the planner and its grid map.
The controller keeps using the true odometry, so the drift shows up as a real map/pose mismatch.

Drift levels (sigma_q = random-walk intensity, reported covariance = (sigma0^2 + sigma_q^2 t) I, capped):
  0 : pass-through, message unchanged (baseline)
  1 : sigma_q = 0.02 m/sqrt(s)
  2 : sigma_q = 0.05 m/sqrt(s)
"""
import math
import random

import rospy
from nav_msgs.msg import Odometry

LEVELS = {0: 0.0, 1: 0.02, 2: 0.05}
SIGMA0 = 0.01        # [m] reported floor when drift is on
VAR_CAP = 0.30 ** 2  # [m^2] reported covariance cap


class Relay(object):
    def __init__(self):
        self.level = int(rospy.get_param("~drift_level", 0))
        self.sigma_q = LEVELS.get(self.level, 0.0)
        self.rng = random.Random(int(rospy.get_param("~seed", 0)))
        self.pub = rospy.Publisher(rospy.get_param("~output", "/uav1/exp/odom_drift"), Odometry, queue_size=10)
        self.sub = rospy.Subscriber(rospy.get_param("~input", "/uav1/mavros/local_position/odom"), Odometry,
                                    self.cb, queue_size=10, tcp_nodelay=True)
        self.offset = [0.0, 0.0, 0.0]
        self.t0 = None
        self.t_last = None
        rospy.loginfo("[drift_relay] level=%d sigma_q=%.3f", self.level, self.sigma_q)

    def cb(self, msg):
        if self.sigma_q <= 0.0:
            self.pub.publish(msg)  # level 0: unchanged
            return
        t = msg.header.stamp.to_sec()
        if self.t0 is None:
            self.t0 = self.t_last = t
        dt = max(0.0, min(t - self.t_last, 1.0))
        self.t_last = t
        # random walk only while airborne-relevant motion is possible (always on; z drift halved)
        s = self.sigma_q * math.sqrt(dt)
        self.offset[0] += self.rng.gauss(0.0, s)
        self.offset[1] += self.rng.gauss(0.0, s)
        self.offset[2] += self.rng.gauss(0.0, 0.5 * s)
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.pose.pose.position.x += self.offset[0]
        out.pose.pose.position.y += self.offset[1]
        out.pose.pose.position.z += self.offset[2]
        out.twist = msg.twist
        var = min(SIGMA0 ** 2 + self.sigma_q ** 2 * (t - self.t0), VAR_CAP)
        cov = [0.0] * 36
        cov[0] = cov[7] = var
        cov[14] = 0.25 * var
        for i in (21, 28, 35):
            cov[i] = 1e-4
        out.pose.covariance = cov
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("odom_drift_relay")
    Relay()
    rospy.spin()
