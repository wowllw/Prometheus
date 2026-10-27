#!/usr/bin/env python3
"""Mission monitor for one experiment run (new, independent helper).

Arms the vehicle and enters COMMAND_CONTROL (same messages as Scripts/simulation/NO_RC/arm_and_command.sh),
waits for the take-off hover, publishes the scenario goal to the planner, then watches the true odometry
until the goal is reached, the vehicle crashes, or the timeout expires. Writes result.json.
"""
import argparse
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from prometheus_msgs.msg import UAVSetup, UAVState
from std_msgs.msg import String

SCENARIOS = {  # goal points in planning_easy.world (cylinder field), z inside the planner's height band
    0: (10.0, 0.0, 1.5),
    1: (8.0, -6.0, 1.5),
    2: (-6.0, -6.0, 1.5),
}


class Monitor(object):
    def __init__(self, args):
        self.args = args
        self.pos = None
        self.state = None
        self.path_len = 0.0
        self.last_pos = None
        rospy.Subscriber("/uav1/mavros/local_position/odom", Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber("/uav1/prometheus/state", UAVState, self.state_cb, queue_size=5)
        self.setup_pub = rospy.Publisher("/uav1/prometheus/setup", UAVSetup, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher("/uav1/prometheus/motion_planning/goal", PoseStamped, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/exp/monitor_status", String, queue_size=10, latch=True)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        cur = (p.x, p.y, p.z)
        if self.last_pos is not None and self.airborne_since is not None:
            self.path_len += math.sqrt(sum((a - b) ** 2 for a, b in zip(cur, self.last_pos)))
        self.last_pos = cur
        self.pos = cur

    airborne_since = None

    def state_cb(self, msg):
        self.state = msg

    def status(self, text):
        rospy.loginfo("[exp_monitor] %s", text)
        self.status_pub.publish(String(data=text))

    def setup(self, cmd, control_state):
        m = UAVSetup()
        m.cmd = cmd
        m.arming = True
        m.control_state = control_state
        self.setup_pub.publish(m)

    def wait(self, cond, timeout):
        t_end = time.time() + timeout
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() < t_end:
            if cond():
                return True
            r.sleep()
        return False

    def run(self):
        a = self.args
        res = {"scenario": a.scenario, "goal": SCENARIOS[a.scenario], "outcome": "error", "t_wall_start": time.time()}
        if not self.wait(lambda: self.pos is not None and self.state is not None, a.ready_timeout):
            res["outcome"] = "no_state"
            return res
        self.status("arming")
        self.setup(0, "")                      # arm (cmd 0)
        time.sleep(3.0)
        self.status("command_control")
        self.setup(3, "COMMAND_CONTROL")      # enter COMMAND_CONTROL (cmd 3) -> Init_Pos_Hover take-off
        if not self.wait(lambda: self.pos[2] > a.takeoff_z, a.takeoff_timeout):
            res["outcome"] = "takeoff_failed"
            res["max_z"] = self.pos[2]
            return res
        time.sleep(2.0)                       # settle in hover
        self.airborne_since = time.time()
        res["takeoff_pos"] = self.pos
        g = SCENARIOS[a.scenario]
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = g
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        t_goal = time.time()
        res["t_goal"] = t_goal
        self.status("goal_sent %.2f %.2f %.2f" % g)

        def dist():
            return math.sqrt(sum((p - q) ** 2 for p, q in zip(self.pos, g)))

        outcome = "timeout"
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() - t_goal < a.mission_timeout:
            if dist() < a.goal_tol:
                outcome = "reached"
                break
            if self.pos[2] < a.crash_z:
                outcome = "crash"
                break
            r.sleep()
        res["outcome"] = outcome
        res["flight_time"] = time.time() - t_goal
        res["final_dist"] = dist()
        res["path_length"] = self.path_len
        res["t_wall_end"] = time.time()
        self.status("done %s" % outcome)
        return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ready_timeout", type=float, default=120.0)
    ap.add_argument("--takeoff_timeout", type=float, default=60.0)
    ap.add_argument("--takeoff_z", type=float, default=0.8)
    ap.add_argument("--mission_timeout", type=float, default=120.0)
    ap.add_argument("--goal_tol", type=float, default=0.5)
    ap.add_argument("--crash_z", type=float, default=0.25)
    args, _ = ap.parse_known_args()
    rospy.init_node("exp_monitor", anonymous=True)
    res = Monitor(args).run()
    with open(args.out, "w") as f:
        json.dump(res, f, indent=2)
    print(json.dumps(res))


if __name__ == "__main__":
    main()
