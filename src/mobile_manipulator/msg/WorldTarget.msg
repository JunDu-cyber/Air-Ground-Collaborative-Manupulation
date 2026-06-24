# A single 3-D target point detected by the UAV camera during terrain scanning.
# Reserved interface: the (future) UAV vision detector publishes ONE WorldTarget
# per detection on /detected_targets (streamed). ugv_target_tour collects them,
# transforms point.header.frame_id -> odom, orders them, and drives the UGV.
#
# Vision recognition carries a class + confidence, but UGV navigation uses ONLY
# the position (orientation is ignored).
string                     class_name
int32                      class_id
float32                    confidence
geometry_msgs/PointStamped point        # 3-D position + its own frame_id
