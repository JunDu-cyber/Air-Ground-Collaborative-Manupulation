#!/usr/bin/env python3
import os
import rospy
import rospkg
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf.transformations as tf_trans


def create_polygon_marker(region, m_id):
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = "semantic_polygons"
    m.id = m_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    for pt in region['polygon']:
        p = Point(x=pt[0], y=pt[1], z=0.05)
        m.points.append(p)
    if region['polygon']:
        p = Point(x=region['polygon'][0][0], y=region['polygon'][0][1], z=0.05)
        m.points.append(p)
    m.scale.x = 0.05
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.color.a = 0.8
    m.pose.orientation.w = 1.0
    return m


def create_nav_arrow_marker(region, m_id):
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = "nav_goals"
    m.id = m_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x = region['x']
    m.pose.position.y = region['y']
    m.pose.position.z = 0.1
    q = tf_trans.quaternion_from_euler(0, 0, region['yaw'])
    m.pose.orientation.x = q[0]
    m.pose.orientation.y = q[1]
    m.pose.orientation.z = q[2]
    m.pose.orientation.w = q[3]
    m.scale.x = 0.5
    m.scale.y = 0.1
    m.scale.z = 0.1
    m.color.r = 1.0
    m.color.g = 0.2
    m.color.b = 0.2
    m.color.a = 0.9
    return m


def create_point_marker(region, m_id):
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = "nav_points"
    m.id = m_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = region['x']
    m.pose.position.y = region['y']
    m.pose.position.z = 0.05
    m.pose.orientation.w = 1.0
    m.scale.x = 0.15
    m.scale.y = 0.15
    m.scale.z = 0.15
    m.color.r = 1.0
    m.color.g = 0.65
    m.color.b = 0.0
    m.color.a = 1.0
    return m


def create_text_marker(region, m_id):
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = "semantic_names"
    m.id = m_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = region['x']
    m.pose.position.y = region['y']
    m.pose.position.z = 0.6
    m.pose.orientation.w = 1.0
    room = region.get('room', 'unknown')
    m.text = f"{region['name']}\n[{room}]\n({region['x']:.2f}, {region['y']:.2f})"
    m.scale.z = 0.20
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.color.a = 1.0
    return m


def build_marker_array(regions):
    marker_array = MarkerArray()

    # DELETEALL must have a valid frame_id or RViz logs a warning
    delete_m = Marker()
    delete_m.header.frame_id = "map"
    delete_m.header.stamp = rospy.Time.now()
    delete_m.action = Marker.DELETEALL
    marker_array.markers.append(delete_m)

    for m_id, region in enumerate(regions):
        marker_array.markers.append(create_polygon_marker(region, m_id))
        marker_array.markers.append(create_nav_arrow_marker(region, m_id))
        marker_array.markers.append(create_point_marker(region, m_id))
        marker_array.markers.append(create_text_marker(region, m_id))

    return marker_array


def main():
    rospy.init_node('visualize_semantic_map', anonymous=True)
    pub = rospy.Publisher('/semantic_markers', MarkerArray, queue_size=1, latch=True)

    # yaml_path can be overridden via ROS param
    default_path = os.path.join(
        rospkg.RosPack().get_path('mobile_manipulator'),
        'config', 'semantic_map.yaml')
    yaml_path = rospy.get_param('~yaml_path', default_path)

    rospy.loginfo(f"Semantic map visualizer watching: {yaml_path}")

    rate = rospy.Rate(1)
    last_mtime = 0

    while not rospy.is_shutdown():
        if not os.path.exists(yaml_path):
            rate.sleep()
            continue

        mtime = os.path.getmtime(yaml_path)
        if mtime == last_mtime:
            rate.sleep()
            continue

        last_mtime = mtime
        rospy.loginfo("Reloading semantic_map.yaml …")
        try:
            with open(yaml_path) as f:
                data = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load {yaml_path}: {e}")
            rate.sleep()
            continue

        if not data or 'regions' not in data:
            rospy.logwarn("semantic_map.yaml has no 'regions' key — nothing to publish.")
            rate.sleep()
            continue

        pub.publish(build_marker_array(data['regions']))
        rospy.loginfo(f"Published {len(data['regions'])} semantic regions.")
        rate.sleep()


if __name__ == '__main__':
    main()
