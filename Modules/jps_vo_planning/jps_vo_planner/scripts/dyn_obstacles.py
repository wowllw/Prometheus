#!/usr/bin/env python3
"""Constant-velocity dynamic obstacles for the JPS+VO experiments (Subtask 3, deliverable 2).

Spawns sphere models named <prefix><id> (default prefix dyn_obs_, the prefix the planner filters in
/gazebo/model_states) and moves them at constant velocity by publishing /gazebo/set_model_state at `rate` Hz.
Pose AND twist are set every tick, so /gazebo/model_states reports the true velocity v_o (D2: constant, measured).
The spheres have visuals, so the D435i sees them; gravity is off.

Motion of obstacle i (t = wall time since its start trigger, use_sim_time=false like the rest of the stack):
  once     : p(t) = p0 + v * min(t, T), velocity v while t < T, then holds p0 + v T with zero velocity
  pingpong : triangle wave between p0 and p0 + v T (velocity reverses every T seconds)
Start trigger per obstacle: immediately, or when the UAV comes within trigger_radius of trigger_point
(reproducible encounter geometry independent of takeoff timing).

Usage:
  rosrun jps_vo_planner dyn_obstacles.py _scenario:=$(rospack find jps_vo_planner)/config/scenarios/crossing.yaml \
         _seed:=3 _log_dir:=$LOG_DIR
"""
import math
import os
import random
import time

import rospy
import yaml
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <gravity>0</gravity>
      <inertial><mass>1.0</mass></inertial>
      <collision name='collision'><geometry><sphere><radius>{r}</radius></sphere></geometry></collision>
      <visual name='visual'>
        <geometry><sphere><radius>{r}</radius></sphere></geometry>
        <material><ambient>0.9 0.3 0.1 1</ambient><diffuse>0.9 0.3 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class Obstacle:
    def __init__(self, idx, cfg, prefix, rng):
        self.name = cfg.get('name', '%s%d' % (prefix, idx))
        if not self.name.startswith(prefix):
            raise ValueError('obstacle name %s must start with prefix %s' % (self.name, prefix))
        self.r = float(cfg['radius'])
        jp = float(cfg.get('jitter_pos', 0.0))
        js = float(cfg.get('jitter_speed', 0.0))
        self.p0 = [float(x) + rng.uniform(-jp, jp) for x in cfg['start']]
        v = [float(x) for x in cfg['velocity']]
        speed = math.sqrt(sum(x * x for x in v))
        scale = (speed + rng.uniform(-js, js)) / speed if speed > 0 else 1.0
        self.v = [x * scale for x in v]
        self.T = float(cfg['duration'])
        self.mode = cfg.get('mode', 'once')
        self.delay = float(cfg.get('start_delay', 0.0))
        self.trigger_point = cfg.get('trigger_point')
        self.trigger_radius = float(cfg.get('trigger_radius', 0.0))
        self.t_start = None  # wall time when motion starts

    def state(self, now):
        """position and velocity at wall time now"""
        if self.t_start is None or now < self.t_start:
            return list(self.p0), [0.0, 0.0, 0.0]
        t = now - self.t_start
        if self.mode == 'pingpong':
            k = int(t // self.T)
            tau = t - k * self.T
            if k % 2 == 0:
                return [p + v * tau for p, v in zip(self.p0, self.v)], list(self.v)
            return [p + v * (self.T - tau) for p, v in zip(self.p0, self.v)], [-v for v in self.v]
        if t < self.T:
            return [p + v * t for p, v in zip(self.p0, self.v)], list(self.v)
        return [p + v * self.T for p, v in zip(self.p0, self.v)], [0.0, 0.0, 0.0]


def load_scenario(path, variant=''):
    """Flat file (top-level `obstacles`) or experiment file with `variants: {name: {...}}`; a variant's keys
    override the top-level ones."""
    cfg = yaml.safe_load(open(path))
    if 'variants' in cfg:
        if variant not in cfg['variants']:
            raise ValueError('%s: choose _variant from %s' % (path, list(cfg['variants'])))
        merged = {k: v for k, v in cfg.items() if k != 'variants'}
        for k, v in cfg['variants'][variant].items():
            if isinstance(v, dict) and isinstance(merged.get(k), dict):
                merged[k] = dict(merged[k], **v)  # one level deep, e.g. uav: {goal} keeps uav.spawn
            else:
                merged[k] = v
        cfg = merged
    cfg.setdefault('obstacles', [])
    return cfg


def main():
    rospy.init_node('dyn_obstacles')
    scenario = rospy.get_param('~scenario')
    seed = int(rospy.get_param('~seed', 0))
    log_dir = rospy.get_param('~log_dir', '')
    rate_hz = float(rospy.get_param('~rate', 50.0))
    odom_topic = rospy.get_param('~odom_topic', '/uav1/mavros/local_position/odom')

    cfg = load_scenario(scenario, rospy.get_param('~variant', ''))
    prefix = cfg.get('prefix', 'dyn_obs_')
    rng = random.Random(seed)
    obs = [Obstacle(i, c, prefix, rng) for i, c in enumerate(cfg['obstacles'])]

    uav = {'p': None}
    rospy.Subscriber(odom_topic, Odometry,
                     lambda m: uav.__setitem__('p', (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)),
                     queue_size=1)
    state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=max(1, len(obs) * 2))
    marker_pub = rospy.Publisher('/jps_vo/dyn_obstacles_markers', MarkerArray, queue_size=1)

    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=30.0)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    for o in obs:
        try:
            delete(o.name)  # leftover from a previous run
        except rospy.ServiceException:
            pass
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = o.p0
        pose.orientation.w = 1.0
        res = spawn(o.name, SDF.format(name=o.name, r=o.r), '', pose, 'world')
        rospy.loginfo('spawn %s r=%.2f p0=(%.2f, %.2f, %.2f) v=(%.2f, %.2f, %.2f) |v|=%.2f T=%.1fs mode=%s: %s',
                      o.name, o.r, *o.p0, *o.v, math.sqrt(sum(x * x for x in o.v)), o.T, o.mode, res.status_message)

    t_launch = time.time()
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)
        with open(os.path.join(log_dir, 'dyn_obstacles_seed%d.yaml' % seed), 'w') as f:
            yaml.safe_dump({'scenario': scenario, 'variant': rospy.get_param('~variant', ''), 'seed': seed,
                            'wall_time_spawn': t_launch,
                            'obstacles': [{'name': o.name, 'radius': o.r, 'start': o.p0, 'velocity': o.v,
                                           'duration': o.T, 'mode': o.mode} for o in obs]}, f)

    def shutdown():
        for o in obs:
            try:
                delete(o.name)
            except Exception:  # gazebo may already be gone
                pass
    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        now = time.time()
        markers = MarkerArray()
        for i, o in enumerate(obs):
            if o.t_start is None:
                if o.trigger_point is None:
                    o.t_start = t_launch + o.delay
                elif uav['p'] is not None and math.dist(uav['p'], o.trigger_point) <= o.trigger_radius:
                    o.t_start = now + o.delay
                    rospy.loginfo('%s triggered at UAV (%.2f, %.2f, %.2f), starts in %.1fs', o.name, *uav['p'], o.delay)
            p, v = o.state(now)
            ms = ModelState()
            ms.model_name = o.name
            ms.reference_frame = 'world'
            ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = p
            ms.pose.orientation.w = 1.0
            ms.twist.linear.x, ms.twist.linear.y, ms.twist.linear.z = v
            state_pub.publish(ms)

            mk = Marker()
            mk.header.frame_id = 'world'
            mk.header.stamp = rospy.Time.now()
            mk.ns, mk.id, mk.type, mk.action = 'dyn_obs', i, Marker.SPHERE, Marker.ADD
            mk.pose = ms.pose
            mk.scale.x = mk.scale.y = mk.scale.z = 2 * o.r
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.9, 0.3, 0.1, 0.6
            markers.markers.append(mk)
        marker_pub.publish(markers)
        rate.sleep()


if __name__ == '__main__':
    main()
