#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""px4_bridge: EGO-Planner → MAVROS OFFBOARD + 深度相机近障紧急悬停

纯传感器模式: 信任 EGO-Planner 避障路径，深度相机仅作最后防线。
"""

import math
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ── 参数 ───────────────────────────────────────────────────────────
TAKEOFF_Z = 2.5               # 起飞初始悬停高度 (m, LiDAR 需要足够高度覆盖障碍物)
MIN_CMD_Z = 0.8               # 无天花板，但拒绝低于地面安全高度的 planner z
PLANNER_ENABLE_Z = 1.5        # 起飞到足够高度后才允许 planner 接管
CMD_TIMEOUT = rospy.Duration(1.5)
DEPTH_TOPIC = "/depth/image_dilated"
REQUIRE_DEPTH_BEFORE_TAKEOFF = False
DEPTH_WAIT_TIMEOUT = 20.0
OFFBOARD_ARM_RETRY_PERIOD = 1.0
OFFBOARD_ARM_WARN_TIMEOUT = 20.0

# 深度相机近障紧急悬停
DEPTH_BLOCK_DIST = 1.35       # 前方近障时只拦截继续前冲
DEPTH_HARD_STOP_DIST = 0.65   # 正中心极近障碍时无条件悬停
DEPTH_BLOCK_ROWS = (0.25, 0.75)
DEPTH_BLOCK_COLS = (0.32, 0.68)
DEPTH_HARD_ROWS = (0.35, 0.65)
DEPTH_HARD_COLS = (0.42, 0.58)
# 是否让 bridge 用深度做反射式拦截/强制悬停。默认关：交给全局规划器+EGO 规划避障，
# bridge 不再插手(否则它一看近障就强行悬停，跟规划器抢控制 → 乱飞/反复悬停)。
ENABLE_DEPTH_INTERCEPT = False

# 状态
current_state = State()
current_pos = None
current_yaw = None
last_cmd_time = None
cmd_pose = PoseStamped()
block_depth_min = None
hard_depth_min = None
has_depth = False
planner_active = False


def state_cb(msg):
    global current_state
    current_state = msg


def pos_cb(msg):
    global current_pos, current_yaw
    current_pos = msg.pose.position
    q = msg.pose.orientation
    _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])


def robust_depth_min(depth, row_range, col_range):
    h, w = depth.shape
    r0 = int(h * row_range[0]); r1 = int(h * row_range[1])
    c0 = int(w * col_range[0]); c1 = int(w * col_range[1])
    roi = depth[r0:r1, c0:c1]
    valid = roi[np.isfinite(roi) & (roi > 0.05)]
    if valid.size == 0:
        return None
    return float(np.percentile(valid, 5.0))


def depth_cb(msg):
    global block_depth_min, hard_depth_min, has_depth
    if msg.height == 0 or msg.width == 0:
        return
    if msg.encoding == "16UC1":
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
    elif msg.encoding == "32FC1":
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    else:
        return
    block_depth_min = robust_depth_min(depth, DEPTH_BLOCK_ROWS, DEPTH_BLOCK_COLS)
    hard_depth_min = robust_depth_min(depth, DEPTH_HARD_ROWS, DEPTH_HARD_COLS)
    has_depth = True


def cmd_cb(msg):
    """接收 EGO-Planner 指令，深度相机近障紧急悬停"""
    global cmd_pose, last_cmd_time, planner_active

    if current_pos is not None and current_pos.z < PLANNER_ENABLE_Z:
        return
    planner_active = True

    safe_x = msg.position.x
    safe_y = msg.position.y
    safe_z = max(msg.position.z, MIN_CMD_Z)

    if ENABLE_DEPTH_INTERCEPT and current_pos is not None:
        dx = safe_x - current_pos.x
        dy = safe_y - current_pos.y

        # 深度相机紧急悬停: 前方近障时拒绝前冲
        if has_depth and hard_depth_min is not None and hard_depth_min < DEPTH_HARD_STOP_DIST:
            safe_x = current_pos.x
            safe_y = current_pos.y
            safe_z = current_pos.z
            rospy.logerr_throttle(0.5,
                "[Bridge] 正中心极近障 %.2fm -> 强制悬停", hard_depth_min)
        elif (has_depth and block_depth_min is not None
                and block_depth_min < DEPTH_BLOCK_DIST
                and current_yaw is not None):
            forward = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
            lateral = abs(-dy * math.cos(current_yaw) + dx * math.sin(current_yaw))
            if forward > 0.12 and lateral < 0.35:
                safe_x = current_pos.x
                safe_y = current_pos.y
                safe_z = current_pos.z
                rospy.logwarn_throttle(0.5,
                    "[Bridge] 前方近障 %.2fm -> 拦截前冲，允许横向绕行", block_depth_min)

    cmd_pose.pose.position.x = safe_x
    cmd_pose.pose.position.y = safe_y
    cmd_pose.pose.position.z = safe_z
    q = quaternion_from_euler(0, 0, msg.yaw)
    cmd_pose.pose.orientation.x = q[0]
    cmd_pose.pose.orientation.y = q[1]
    cmd_pose.pose.orientation.z = q[2]
    cmd_pose.pose.orientation.w = q[3]
    last_cmd_time = rospy.Time.now()


def publish_pose(pub, pose):
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pub.publish(pose)


def enter_offboard_and_arm(pub, set_mode, arm, rate):
    """Keep streaming setpoints and retry until PX4 confirms OFFBOARD + armed."""
    retry_period = rospy.Duration(
        rospy.get_param("~offboard_arm_retry_period", OFFBOARD_ARM_RETRY_PERIOD)
    )
    warn_timeout = rospy.Duration(
        rospy.get_param("~offboard_arm_warn_timeout", OFFBOARD_ARM_WARN_TIMEOUT)
    )
    last_request = rospy.Time(0)
    start_time = rospy.Time.now()
    warned_timeout = False

    rospy.loginfo("[Bridge] 请求 OFFBOARD + 解锁，直到 MAVROS 状态确认成功...")
    while not rospy.is_shutdown() and (
        current_state.mode != "OFFBOARD" or not current_state.armed
    ):
        publish_pose(pub, cmd_pose)
        now = rospy.Time.now()

        if last_request == rospy.Time(0) or now - last_request >= retry_period:
            if current_state.mode != "OFFBOARD":
                try:
                    mode_resp = set_mode(custom_mode="OFFBOARD")
                    # mode_sent 只是“指令已送达”的回执，不代表已切换；
                    # current_mode 此刻仍是切换前的状态(状态回调尚未刷新)，仅供参考，
                    # 真正确认见循环退出后的 "OFFBOARD + armed confirmed"。
                    rospy.loginfo_throttle(
                        2.0,
                        "[Bridge] set_mode OFFBOARD 已发送(回执 mode_sent=%s)，等待状态切换…(当前 %s)",
                        getattr(mode_resp, "mode_sent", None),
                        current_state.mode,
                    )
                except rospy.ServiceException as exc:
                    rospy.logerr("[Bridge] set_mode OFFBOARD failed: %s", exc)

            if not current_state.armed:
                try:
                    arm_resp = arm(True)
                    # success/result 是解锁请求的回执(result=0 为已接受)；
                    # current_armed 此刻可能仍为旧值，确认见循环退出日志。
                    rospy.loginfo_throttle(
                        2.0,
                        "[Bridge] arm 已发送(success=%s result=%s)，等待解锁确认…(当前 armed=%s)",
                        getattr(arm_resp, "success", None),
                        getattr(arm_resp, "result", None),
                        current_state.armed,
                    )
                except rospy.ServiceException as exc:
                    rospy.logerr("[Bridge] arm failed: %s", exc)

            last_request = now

        if (
            not warned_timeout
            and warn_timeout.to_sec() > 0.0
            and now - start_time > warn_timeout
        ):
            warned_timeout = True
            rospy.logerr(
                "[Bridge] 等待 OFFBOARD/armed 超过 %.1fs，仍继续重试。"
                "请检查 /mavros/statustext/recv 或 PX4 console 的 preflight/arming 拒绝原因。",
                warn_timeout.to_sec(),
            )

        rate.sleep()

    rospy.loginfo("[Bridge] OFFBOARD + armed confirmed, 起飞!")


def main():
    global current_pos, last_cmd_time

    rospy.init_node('px4_ego_bridge')

    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/planning/pos_cmd", PositionCommand, cmd_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pos_cb)
    depth_topic = rospy.get_param("~depth_topic", DEPTH_TOPIC)
    require_depth = rospy.get_param("~require_depth_before_takeoff", REQUIRE_DEPTH_BEFORE_TAKEOFF)
    depth_wait_timeout = rospy.get_param("~depth_wait_timeout", DEPTH_WAIT_TIMEOUT)
    global ENABLE_DEPTH_INTERCEPT
    ENABLE_DEPTH_INTERCEPT = bool(rospy.get_param("~enable_depth_intercept", ENABLE_DEPTH_INTERCEPT))
    rospy.loginfo("[Bridge] 深度反射拦截=%s (False=不插手，全交给全局+EGO 规划器)", ENABLE_DEPTH_INTERCEPT)
    rospy.Subscriber(depth_topic, Image, depth_cb)

    pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
    rate = rospy.Rate(50)

    # 等待 MAVROS
    rospy.loginfo("[Bridge] 等待 MAVROS 连接...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    # 等待位置
    rospy.loginfo("[Bridge] 等待本地位姿...")
    while not rospy.is_shutdown() and current_pos is None:
        rate.sleep()

    if require_depth:
        rospy.loginfo("[Bridge] 等待深度图 %s ...", depth_topic)
        deadline = time.time() + depth_wait_timeout
        while not rospy.is_shutdown() and not has_depth and time.time() < deadline:
            rate.sleep()
        if not has_depth:
            rospy.logerr("[Bridge] 没收到深度图 %s，拒绝解锁起飞，避免无避障盲飞", depth_topic)
            return
    elif not has_depth:
        rospy.logwarn(
            "[Bridge] 未强制等待深度图 %s；先解锁起飞，深度图到达后继续用于近障急停",
            depth_topic)

    takeoff_z = TAKEOFF_Z
    rospy.loginfo("[Bridge] 起飞高度=%.1fm", takeoff_z)

    cmd_pose.pose.position.x = current_pos.x
    cmd_pose.pose.position.y = current_pos.y
    cmd_pose.pose.position.z = takeoff_z
    cmd_pose.pose.orientation.w = 1.0

    for _ in range(100):
        publish_pose(pub, cmd_pose)
        rate.sleep()

    # 解锁 + OFFBOARD
    try:
        rospy.wait_for_service("/mavros/set_mode", timeout=3.0)
        rospy.wait_for_service("/mavros/cmd/arming", timeout=3.0)
        set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("[Bridge] 解锁服务不可用: %s" % e)
        return

    enter_offboard_and_arm(pub, set_mode, arm, rate)

    last_cmd_time = rospy.Time.now()

    while not rospy.is_shutdown():
        # planner 超时 → 原地悬停
        if (rospy.Time.now() - last_cmd_time) > CMD_TIMEOUT and current_pos is not None:
            cmd_pose.pose.position.x = current_pos.x
            cmd_pose.pose.position.y = current_pos.y
            cmd_pose.pose.position.z = current_pos.z if planner_active else TAKEOFF_Z
            cmd_pose.pose.orientation.w = 1.0

        publish_pose(pub, cmd_pose)
        rate.sleep()


if __name__ == '__main__':
    main()
