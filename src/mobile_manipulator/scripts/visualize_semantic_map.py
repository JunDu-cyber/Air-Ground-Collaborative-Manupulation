#!/usr/bin/env python3
import rospy
import yaml
import os
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
    # Draw polygon border a bit above ground to avoid z-fighting with the map
    for pt in region['polygon']:
        p = Point()
        p.x = pt[0]
        p.y = pt[1]
        p.z = 0.05
        m.points.append(p)
    # Close the boundary
    if len(region['polygon']) > 0:
        p = Point()
        p.x = region['polygon'][0][0]
        p.y = region['polygon'][0][1]
        p.z = 0.05
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
    
    # Arrow tail position (the coordinate where Husky base_link should reach)
    m.pose.position.x = region['x']
    m.pose.position.y = region['y']
    m.pose.position.z = 0.1
    
    # Convert yaw to quaternion
    q = tf_trans.quaternion_from_euler(0, 0, region['yaw'])
    m.pose.orientation.x = q[0]
    m.pose.orientation.y = q[1]
    m.pose.orientation.z = q[2]
    m.pose.orientation.w = q[3]
    
    m.scale.x = 0.5  # arrow length
    m.scale.y = 0.1  # arrow shaft width
    m.scale.z = 0.1  # arrow head width
    
    m.color.r = 1.0
    m.color.g = 0.2
    m.color.b = 0.2
    m.color.a = 0.9
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
    m.pose.position.z = 0.6 # Float above nav arrow
    m.pose.orientation.w = 1.0
    
    m.text = region['name']
    m.scale.z = 0.25 # text size
    
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.color.a = 1.0
    return m

def main():
    rospy.init_node('visualize_semantic_map', anonymous=True)
    pub = rospy.Publisher('/semantic_markers', MarkerArray, queue_size=1, latch=True)
    
    yaml_path = '/home/jun/learning_ws/src/mobile_manipulator/config/semantic_map.yaml'
    
    rate = rospy.Rate(1)
    last_mtime = 0
    
    rospy.loginfo("Starting Semantic Map Visualizer.")
    rospy.loginfo(f"Listening to file changes on: {yaml_path}")
    rospy.loginfo("Please open RViz and add a 'MarkerArray' display subscribing to topic: /semantic_markers")

    while not rospy.is_shutdown():
        # Live reload logic: only publish if file has changed
        if not os.path.exists(yaml_path):
            rate.sleep()
            continue
            
        mtime = os.path.getmtime(yaml_path)
        if mtime != last_mtime:
            last_mtime = mtime
            rospy.loginfo("Detected update to semantic_map.yaml. Publishing updated markers to RViz...")
            try:
                with open(yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
            except Exception as e:
                rospy.logerr(f"Failed to load yaml: {e}")
                continue
                
            if not data or 'regions' not in data:
                continue
                
            marker_array = MarkerArray()
            
            # Start by issuing a delete-all to clear out old arrows/boxes
            delete_m = Marker()
            delete_m.action = Marker.DELETEALL
            marker_array.markers.append(delete_m)

            m_id = 0
            for region in data['regions']:
                marker_array.markers.append(create_polygon_marker(region, m_id))
                marker_array.markers.append(create_nav_arrow_marker(region, m_id))
                marker_array.markers.append(create_text_marker(region, m_id))
                m_id += 1
                
            pub.publish(marker_array)
            
        rate.sleep()

if __name__ == '__main__':
    main()
