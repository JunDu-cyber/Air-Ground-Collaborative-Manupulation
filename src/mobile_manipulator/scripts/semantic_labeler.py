#!/usr/bin/env python3

import rospy
import yaml
import os
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class SemanticLabeler:
    def __init__(self):
        rospy.init_node('semantic_labeler')

        self.output_path = rospy.get_param('~output',
            os.path.expanduser(
                '~/catkin_ws/src/mobile_manipulator/config/semantic_map.yaml'))

        self.regions = []
        self.current_polygon = []
        self.min_points = rospy.get_param('~min_points', 3)

        # load existing regions if file exists
        if os.path.exists(self.output_path):
            with open(self.output_path, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'regions' in data:
                    self.regions = data['regions']
                    rospy.loginfo(
                        f"Loaded {len(self.regions)} existing regions")

        # subscriber — use RViz Publish Point tool
        self.point_sub = rospy.Subscriber(
            '/clicked_point',
            PointStamped,
            self.point_callback)

        # publisher for live visualization
        self.marker_pub = rospy.Publisher(
            '/semantic_map/markers',
            MarkerArray,
            latch=True,
            queue_size=1)

        self.publish_all_markers()

        rospy.loginfo("=" * 50)
        rospy.loginfo("Semantic Labeler ready!")
        rospy.loginfo("In RViz: select 'Publish Point' tool (keyboard: p)")
        rospy.loginfo("Click points to define a region polygon")
        rospy.loginfo(f"Minimum {self.min_points} points per region")
        rospy.loginfo("Double-click last point to close polygon")
        rospy.loginfo("=" * 50)

        rospy.spin()

    def point_callback(self, msg):
        x = round(msg.point.x, 3)
        y = round(msg.point.y, 3)

        # check if double-click (within 0.15m of last point = close polygon)
        if self.current_polygon:
            last = self.current_polygon[-1]
            dist = ((x - last[0])**2 + (y - last[1])**2) ** 0.5
            if dist < 0.15:
                self.close_polygon()
                return

        self.current_polygon.append([x, y])
        rospy.loginfo(
            f"Point {len(self.current_polygon)} added: ({x}, {y})")

        if len(self.current_polygon) >= self.min_points:
            rospy.loginfo(
                "  → click near first point to close, "
                "or keep adding points")

        # show in-progress polygon
        self.publish_all_markers()

    def close_polygon(self):
        if len(self.current_polygon) < self.min_points:
            rospy.logwarn(
                f"Need at least {self.min_points} points, "
                f"only have {len(self.current_polygon)}")
            return

        rospy.loginfo("Polygon closed!")
        rospy.loginfo("-" * 40)

        # compute centroid as default nav goal
        cx = round(sum(p[0] for p in self.current_polygon)
                   / len(self.current_polygon), 3)
        cy = round(sum(p[1] for p in self.current_polygon)
                   / len(self.current_polygon), 3)

        # prompt for region name
        rospy.loginfo(f"Centroid (nav goal): ({cx}, {cy})")
        name = input("Enter region name (e.g. desk, sofa, kitchen): ").strip()
        if not name:
            rospy.logwarn("Empty name, discarding region")
            self.current_polygon = []
            return

        # prompt for nav goal override
        rospy.loginfo(
            f"Default nav goal is centroid ({cx}, {cy})")
        override = input(
            "Override nav goal? Enter 'x y yaw' or press Enter to keep: "
        ).strip()

        yaw = 0.0
        if override:
            parts = override.split()
            if len(parts) >= 2:
                cx = float(parts[0])
                cy = float(parts[1])
            if len(parts) >= 3:
                yaw = float(parts[2])

        region = {
            'name': name,
            'x': cx,
            'y': cy,
            'yaw': yaw,
            'polygon': self.current_polygon
        }

        self.regions.append(region)
        self.current_polygon = []

        self.save_yaml()
        self.publish_all_markers()

        rospy.loginfo(f"Region '{name}' saved!")
        rospy.loginfo(f"Total regions: {len(self.regions)}")
        rospy.loginfo("-" * 40)
        rospy.loginfo("Ready for next region — click points in RViz")

    def save_yaml(self):
        data = {'regions': self.regions}
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        with open(self.output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        rospy.loginfo(f"Saved to {self.output_path}")

    def publish_all_markers(self):
        marker_array = MarkerArray()
        colors = [
            [0.9, 0.3, 0.3],
            [0.3, 0.9, 0.3],
            [0.3, 0.3, 0.9],
            [0.9, 0.9, 0.3],
            [0.9, 0.3, 0.9],
            [0.3, 0.9, 0.9],
        ]

        # draw saved regions
        for i, region in enumerate(self.regions):
            color = colors[i % len(colors)]
            self.add_region_markers(
                marker_array, region, i * 3, color, alpha=0.8)

        # draw in-progress polygon in white
        if self.current_polygon:
            self.add_progress_markers(
                marker_array,
                self.current_polygon,
                len(self.regions) * 3)

        self.marker_pub.publish(marker_array)

    def add_region_markers(self, arr, region, base_id, color, alpha=0.8):
        # polygon outline
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = 'regions'
        m.id = base_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.06
        m.color = ColorRGBA(color[0], color[1], color[2], alpha)
        m.lifetime = rospy.Duration(0)
        for pt in region['polygon']:
            m.points.append(Point(x=pt[0], y=pt[1], z=0.01))
        # close polygon
        m.points.append(
            Point(x=region['polygon'][0][0],
                  y=region['polygon'][0][1], z=0.01))
        arr.markers.append(m)

        # text label
        t = Marker()
        t.header.frame_id = 'map'
        t.header.stamp = rospy.Time.now()
        t.ns = 'labels'
        t.id = base_id + 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = region['x']
        t.pose.position.y = region['y']
        t.pose.position.z = 0.6
        t.scale.z = 0.35
        t.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        t.text = region['name']
        t.lifetime = rospy.Duration(0)
        arr.markers.append(t)

        # nav goal sphere
        s = Marker()
        s.header.frame_id = 'map'
        s.header.stamp = rospy.Time.now()
        s.ns = 'goals'
        s.id = base_id + 2
        s.type = Marker.SPHERE
        s.action = Marker.ADD
        s.pose.position.x = region['x']
        s.pose.position.y = region['y']
        s.pose.position.z = 0.05
        s.scale.x = s.scale.y = s.scale.z = 0.25
        s.color = ColorRGBA(color[0], color[1], color[2], 1.0)
        s.lifetime = rospy.Duration(0)
        arr.markers.append(s)

    def add_progress_markers(self, arr, points, base_id):
        # white dots for each clicked point
        for i, pt in enumerate(points):
            d = Marker()
            d.header.frame_id = 'map'
            d.header.stamp = rospy.Time.now()
            d.ns = 'progress_dots'
            d.id = base_id + i
            d.type = Marker.SPHERE
            d.action = Marker.ADD
            d.pose.position.x = pt[0]
            d.pose.position.y = pt[1]
            d.pose.position.z = 0.05
            d.scale.x = d.scale.y = d.scale.z = 0.15
            d.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            d.lifetime = rospy.Duration(0)
            arr.markers.append(d)

        # white line connecting in-progress points
        if len(points) >= 2:
            l = Marker()
            l.header.frame_id = 'map'
            l.header.stamp = rospy.Time.now()
            l.ns = 'progress_line'
            l.id = base_id + 100
            l.type = Marker.LINE_STRIP
            l.action = Marker.ADD
            l.scale.x = 0.04
            l.color = ColorRGBA(1.0, 1.0, 1.0, 0.6)
            l.lifetime = rospy.Duration(0)
            for pt in points:
                l.points.append(Point(x=pt[0], y=pt[1], z=0.01))
            arr.markers.append(l)

if __name__ == '__main__':
    SemanticLabeler()