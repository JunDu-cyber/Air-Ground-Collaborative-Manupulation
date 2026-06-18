#!/usr/bin/env python3
"""Publish a prefixed UGV robot_description, joint states, and root TF for RViz."""

import math
import os
import subprocess
import xml.etree.ElementTree as ET

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler


def prefixed(name, prefix):
    if not name or not prefix or name.startswith(prefix):
        return name
    return prefix + name


def prefix_attr(element, attr_name, prefix):
    value = element.get(attr_name)
    if value:
        element.set(attr_name, prefixed(value, prefix))


class UgvRobotModelHelper(object):
    def __init__(self):
        default_urdf = os.path.expanduser(
            "~/Air-Ground-Collaborative-Manupulation/src/"
            "husky_ur5_moveit_config/config/gazebo_husky_ur5.urdf"
        )
        self.urdf_file = os.path.expanduser(
            rospy.get_param("~urdf_file", default_urdf)
        )
        self.link_prefix = rospy.get_param("~link_prefix", "ugv_")
        self.description_param = rospy.get_param(
            "~robot_description_param", "/robot_description"
        )
        self.override_visual_materials = self.bool_param("~override_visual_materials", True)
        self.start_robot_state_publisher = self.bool_param(
            "~start_robot_state_publisher", False
        )
        self.robot_state_publisher_bin = rospy.get_param(
            "~robot_state_publisher_bin",
            "/opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher",
        )
        self.robot_state_publisher_proc = None
        self.root_frame = rospy.get_param("~root_frame", "map")
        self.base_link = prefixed(rospy.get_param("~base_link", "base_link"), self.link_prefix)
        self.publish_rate = max(float(rospy.get_param("~publish_rate", 10.0)), 0.1)

        self.x = float(rospy.get_param("~x", 2.5))
        self.y = float(rospy.get_param("~y", -18.0))
        self.z = float(rospy.get_param("~z", 0.25))
        self.yaw = float(rospy.get_param("~yaw", 0.0))

        self.joint_names = []
        self.joint_positions = []
        urdf = self.load_prefixed_urdf()
        rospy.set_param(self.description_param, urdf)

        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=5)
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        self.publish_root_tf()
        if self.start_robot_state_publisher:
            self.start_rsp()
            rospy.on_shutdown(self.stop_rsp)

        rospy.loginfo(
            "[UgvRobotModelHelper] robot_description=%s urdf=%s prefix=%s "
            "root_tf=%s->%s pose=(%.2f, %.2f, %.2f, yaw %.2f) joints=%d "
            "override_materials=%s start_rsp=%s",
            self.description_param,
            self.urdf_file,
            self.link_prefix,
            self.root_frame,
            self.base_link,
            self.x,
            self.y,
            self.z,
            self.yaw,
            len(self.joint_names),
            self.override_visual_materials,
            self.start_robot_state_publisher,
        )

    def bool_param(self, name, default=False):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def load_prefixed_urdf(self):
        if not os.path.isfile(self.urdf_file):
            raise RuntimeError("UGV URDF does not exist: %s" % self.urdf_file)

        tree = ET.parse(self.urdf_file)
        root = tree.getroot()
        root.set("name", prefixed(root.get("name", "husky_ur5"), self.link_prefix))

        for link in root.findall(".//link"):
            prefix_attr(link, "name", self.link_prefix)

        for joint in root.findall(".//joint"):
            prefix_attr(joint, "name", self.link_prefix)
            parent = joint.find("parent")
            if parent is not None and parent.get("link"):
                parent.set("link", prefixed(parent.get("link"), self.link_prefix))
            child = joint.find("child")
            if child is not None and child.get("link"):
                child.set("link", prefixed(child.get("link"), self.link_prefix))

        for gazebo in root.findall(".//gazebo"):
            prefix_attr(gazebo, "reference", self.link_prefix)

        for mimic in root.findall(".//mimic"):
            prefix_attr(mimic, "joint", self.link_prefix)

        if self.override_visual_materials:
            self.apply_visual_materials(root)

        self.collect_joint_defaults(root)
        return ET.tostring(root, encoding="unicode")

    def ensure_material(self, root, name, rgba):
        for material in root.findall("material"):
            if material.get("name") == name:
                color = material.find("color")
                if color is None:
                    color = ET.SubElement(material, "color")
                color.set("rgba", rgba)
                return
        material = ET.SubElement(root, "material")
        material.set("name", name)
        color = ET.SubElement(material, "color")
        color.set("rgba", rgba)

    def material_for_link(self, link_name):
        name = link_name.lower()
        if "wheel" in name:
            return ("ugv_black_rubber", "0.02 0.02 0.02 1.0")
        if "finger" in name or "knuckle" in name or "robotiq" in name or "gripper" in name:
            return ("ugv_gripper_dark", "0.08 0.09 0.10 1.0")
        if "ur5" in name or "shoulder" in name or "upper_arm" in name or "forearm" in name or "wrist" in name:
            return ("ugv_ur5_blue_grey", "0.46 0.58 0.68 1.0")
        if "base_link" in name or "inertial" in name:
            return ("ugv_husky_body", "0.12 0.16 0.18 1.0")
        return ("ugv_medium_grey", "0.42 0.45 0.46 1.0")

    def apply_visual_materials(self, root):
        material_defs = {}
        for link in root.findall(".//link"):
            link_name = link.get("name", "")
            material_name, rgba = self.material_for_link(link_name)
            material_defs[material_name] = rgba
            for visual in link.findall("visual"):
                material = visual.find("material")
                if material is None:
                    material = ET.SubElement(visual, "material")
                else:
                    for child in list(material):
                        material.remove(child)
                material.set("name", material_name)
                color = ET.SubElement(material, "color")
                color.set("rgba", rgba)

        for material_name, rgba in material_defs.items():
            self.ensure_material(root, material_name, rgba)

    def start_rsp(self):
        if not os.path.isfile(self.robot_state_publisher_bin):
            rospy.logerr(
                "[UgvRobotModelHelper] robot_state_publisher executable not found: %s",
                self.robot_state_publisher_bin,
            )
            return
        self.robot_state_publisher_proc = subprocess.Popen(
            [self.robot_state_publisher_bin]
        )
        rospy.loginfo(
            "[UgvRobotModelHelper] started robot_state_publisher pid=%d",
            self.robot_state_publisher_proc.pid,
        )

    def stop_rsp(self):
        proc = self.robot_state_publisher_proc
        if proc is None or proc.poll() is not None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    def collect_joint_defaults(self, root):
        defaults = {
            "ur5_shoulder_pan_joint": 0.0,
            "ur5_shoulder_lift_joint": -1.9,
            "ur5_elbow_joint": 0.7,
            "ur5_wrist_1_joint": 1.8,
            "ur5_wrist_2_joint": 1.57,
            "ur5_wrist_3_joint": 0.0,
            "finger_joint": 0.0,
        }
        movable_types = set(["continuous", "revolute", "prismatic"])
        names = []
        positions = []
        for joint in root.findall(".//joint"):
            if joint.get("type") not in movable_types:
                continue
            name = joint.get("name")
            if not name or name in names:
                continue
            pos = 0.0
            for suffix, value in defaults.items():
                if name.endswith(suffix):
                    pos = value
                    break
            names.append(name)
            positions.append(pos)
        self.joint_names = names
        self.joint_positions = positions

    def publish_root_tf(self):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.root_frame
        msg.child_frame_id = self.base_link
        msg.transform.translation.x = self.x
        msg.transform.translation.y = self.y
        msg.transform.translation.z = self.z
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.static_tf.sendTransform(msg)

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = list(self.joint_names)
            msg.position = list(self.joint_positions)
            self.joint_pub.publish(msg)
            rate.sleep()


def main():
    rospy.init_node("ugv_robot_model_helper")
    helper = UgvRobotModelHelper()
    helper.spin()


if __name__ == "__main__":
    main()
