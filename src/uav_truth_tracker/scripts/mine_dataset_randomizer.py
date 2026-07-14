#!/usr/bin/env python3
"""Spawn randomized landmines and visually similar hard negatives in Gazebo."""

import math
import random
import rospy
from gazebo_msgs.srv import DeleteModel, SetLightProperties, SpawnModel
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA


MINE_TEMPLATE = """<?xml version='1.0'?>
<sdf version='1.6'><model name='{name}'><static>true</static><link name='body'>
<collision name='disc'><pose>0 0 0.0125 0 0 0</pose><geometry><cylinder><radius>{radius}</radius><length>0.025</length></cylinder></geometry></collision>
<visual name='disc'><pose>0 0 0.0125 0 0 0</pose><geometry><cylinder><radius>{radius}</radius><length>0.025</length></cylinder></geometry><material><ambient>{dr} {dg} {db} 1</ambient><diffuse>{dr} {dg} {db} 1</diffuse></material></visual>
<collision name='detonator'><pose>0 0 0.055 0 0 0</pose><geometry><box><size>0.04 0.04 0.06</size></box></geometry></collision>
<visual name='detonator'><pose>0 0 0.055 0 0 0</pose><geometry><box><size>0.04 0.04 0.06</size></box></geometry><material><ambient>{br} {bg} {bb} 1</ambient><diffuse>{br} {bg} {bb} 1</diffuse></material></visual>
</link></model></sdf>"""

DISTRACTOR_TEMPLATE = """<?xml version='1.0'?>
<sdf version='1.6'><model name='{name}'><static>true</static><link name='body'>
<collision name='collision'><pose>0 0 {half} 0 0 0</pose><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></collision>
<visual name='visual'><pose>0 0 {half} 0 0 0</pose><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry><material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>
</link></model></sdf>"""


class Randomizer:
    def __init__(self):
        self.seed = int(rospy.get_param("~seed", 42))
        self.rng = random.Random(self.seed)
        self.mine_count = int(rospy.get_param("~mine_count", 10))
        self.distractor_count = int(rospy.get_param("~distractor_count", 12))
        self.extent = float(rospy.get_param("~extent", 12.0))
        self.min_spacing = float(rospy.get_param("~min_spacing", 1.5))
        self.refresh_period = float(rospy.get_param("~refresh_period", 75.0))
        self.hard_distractor_ratio = min(max(float(rospy.get_param("~hard_distractor_ratio", 0.25)), 0.0), 1.0)
        self.names = []
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        self.spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.light = rospy.ServiceProxy("/gazebo/set_light_properties", SetLightProperties)
        self.palette = [(0.65,0.05,0.04), (0.20,0.26,0.05), (0.28,0.18,0.08),
                        (0.18,0.18,0.16), (0.38,0.12,0.06), (0.08,0.22,0.12)]
        self.cycle = 0
        rospy.set_param("/mine_dataset/randomizer_seed", self.seed)
        rospy.set_param("/mine_dataset/randomizer_busy", False)
        rospy.set_param("/mine_dataset/randomizer_last_change_time", -1.0)
        self.regenerate()
        if self.refresh_period > 0:
            rospy.Timer(rospy.Duration(self.refresh_period), lambda _: self.regenerate())

    def positions(self, total):
        out = []
        attempts = 0
        while len(out) < total and attempts < total * 100:
            attempts += 1
            p = (self.rng.uniform(-self.extent, self.extent),
                 self.rng.uniform(-self.extent, self.extent))
            if math.hypot(*p) < 1.5:
                continue
            if all(math.hypot(p[0]-q[0], p[1]-q[1]) >= self.min_spacing for q in out):
                out.append(p)
        return out

    def regenerate(self):
        """Regenerate a complete scene behind a collector-visible barrier."""
        if rospy.is_shutdown():
            return
        rospy.set_param("/mine_dataset/randomizer_busy", True)
        try:
            self._regenerate_scene()
        finally:
            if not rospy.is_shutdown():
                rospy.set_param("/mine_dataset/randomizer_last_change_time", rospy.Time.now().to_sec())
                rospy.set_param("/mine_dataset/randomizer_busy", False)

    def _regenerate_scene(self):
        for name in self.names:
            try: self.delete(name)
            except (rospy.ServiceException, rospy.ROSInterruptException): return
        self.names = []
        brightness = self.rng.uniform(0.45, 1.0)
        direction = Vector3(self.rng.uniform(-0.7, 0.7), self.rng.uniform(-0.7, 0.7), -1.0)
        norm = math.sqrt(direction.x**2 + direction.y**2 + direction.z**2)
        direction.x /= norm; direction.y /= norm; direction.z /= norm
        light_pose = Pose(); light_pose.position.z = 10.0; light_pose.orientation.w = 1.0
        try:
            self.light("sun", True, ColorRGBA(brightness, brightness, brightness, 1.0),
                       ColorRGBA(0.2, 0.2, 0.2, 1.0), 1.0, 0.0, 0.0,
                       direction, light_pose)
        except (rospy.ServiceException, rospy.ROSInterruptException) as exc:
            if rospy.is_shutdown(): return
            rospy.logwarn_throttle(5.0, "[MineRandomizer] light randomization unavailable: %s", exc)
        positions = self.positions(self.mine_count + self.distractor_count)
        for i, (x, y) in enumerate(positions[:self.mine_count]):
            name = f"landmine_aug_{self.cycle:03d}_{i:02d}"
            color = self.rng.choice(self.palette)
            block = self.rng.choice([(0.85,0.72,0.02), (0.55,0.48,0.08), (0.22,0.20,0.12)])
            # Keep geometry equal to the labelled canonical mesh. Appearance,
            # pose and context are randomized without corrupting mask truth.
            radius = 0.068
            sdf = MINE_TEMPLATE.format(name=name, radius=radius, dr=color[0], dg=color[1], db=color[2],
                                       br=block[0], bg=block[1], bb=block[2])
            pose = Pose(); pose.position.x=x; pose.position.y=y; pose.position.z=self.rng.uniform(-0.012, 0.0)
            yaw = self.rng.uniform(-math.pi, math.pi)
            pose.orientation.z = math.sin(yaw/2.0); pose.orientation.w = math.cos(yaw/2.0)
            try:
                self.spawn(name, sdf, "", pose, "world"); self.names.append(name)
            except (rospy.ServiceException, rospy.ROSInterruptException) as exc:
                if rospy.is_shutdown(): return
                rospy.logwarn("spawn %s failed: %s", name, exc)
        for i, (x, y) in enumerate(positions[self.mine_count:]):
            hard = self.rng.random() < self.hard_distractor_ratio
            name = f"distractor_disc_{'hard' if hard else 'easy'}_{self.cycle:03d}_{i:02d}"
            if hard:
                # Mine-like but still solvable: similar disc, no detonator.
                color = self.rng.choice(self.palette)
                radius = self.rng.uniform(0.052, 0.086)
                height = self.rng.uniform(0.018, 0.035)
            else:
                # Most negatives must remain visibly distinguishable at 3--4 m.
                color = self.rng.choice(self.palette + [(0.7,0.7,0.7), (0.05,0.05,0.05)])
                radius = self.rng.uniform(0.032, 0.050) if self.rng.random() < 0.5 else self.rng.uniform(0.095, 0.135)
                height = self.rng.uniform(0.015, 0.065)
            sdf = DISTRACTOR_TEMPLATE.format(name=name, radius=radius,
                                             height=height, half=height/2.0,
                                             r=color[0], g=color[1], b=color[2])
            pose = Pose(); pose.position.x=x; pose.position.y=y; pose.orientation.w=1.0
            try:
                self.spawn(name, sdf, "", pose, "world"); self.names.append(name)
            except (rospy.ServiceException, rospy.ROSInterruptException):
                if rospy.is_shutdown(): return
        rospy.set_param("/mine_dataset/domain_cycle", self.cycle)
        rospy.set_param("/mine_dataset/light_brightness", brightness)
        rospy.set_param("/mine_dataset/scene_id", f"seed_{self.seed}_cycle_{self.cycle}")
        rospy.logwarn("[MineRandomizer] cycle=%d mines=%d distractors=%d seed=%d",
                      self.cycle, self.mine_count, self.distractor_count, self.seed)
        self.cycle += 1


if __name__ == "__main__":
    rospy.init_node("mine_dataset_randomizer")
    Randomizer()
    rospy.spin()
