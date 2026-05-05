#!/usr/bin/env python3
"""
Flask web interface for the Husky UR5 mobile manipulator.

Architecture:
  - Main thread  : rospy.spin() — keeps ROS alive
  - Worker thread: processes commands from Flask via a queue (blocking nav/arm calls run here)
  - Flask thread : serves HTTP; puts commands on queue, waits for results
"""

import os
import sys
import math
import queue
import threading
import yaml
import rospy
from flask import Flask, jsonify, request, render_template
from std_msgs.msg import Bool

# Import RobotAgent from the sibling script
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from agent import RobotAgent

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PATH   = os.path.expanduser("~/learning_ws/src/mobile_manipulator/config/semantic_map.yaml")

app    = Flask(__name__, template_folder=os.path.join(SCRIPT_DIR, 'templates'))
_agent = None
_cmd_q = queue.Queue()
_res_q = queue.Queue()
_history      = []          # [{role, text}, ...]
_history_lock = threading.Lock()

# ── E-stop state ────────────────────────────────────────────────────────────
_estop_active  = False
_estop_pub     = None          # rospy.Publisher, set in main()
_estop_lock    = threading.Lock()



# ── Routes ─────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/command', methods=['POST'])
def api_command():
    cmd = (request.get_json(force=True).get('command') or '').strip()
    if not cmd:
        return jsonify(error='Empty command'), 400

    with _history_lock:
        _history.append(dict(role='user', text=cmd))

    _cmd_q.put(cmd)
    try:
        result = _res_q.get(timeout=120)
    except queue.Empty:
        return jsonify(error='Agent timed out after 120 s'), 504

    with _history_lock:
        _history.append(dict(role='robot', text=result))
    return jsonify(response=result)


@app.route('/api/map')
def api_map():
    if _agent is None:
        return jsonify(error='Agent not ready'), 503
    locs = [
        {'name': n, 'x': d['x'], 'y': d['y'], 'yaw': d['yaw_rad'], 'room': d['room']}
        for n, d in _agent.semantic_map.items()
    ]
    pose  = _agent.mc.get_current_pose()
    robot = {'x': pose[0], 'y': pose[1], 'yaw': pose[2]} if pose else None
    return jsonify(locations=locs, robot=robot)


@app.route('/api/save_pose', methods=['POST'])
def api_save_pose():
    if _agent is None:
        return jsonify(error='Agent not ready'), 503

    data = request.get_json(force=True)
    name = (data.get('name') or '').strip().lower().replace(' ', '_')
    room = (data.get('room') or 'unknown').strip().lower().replace(' ', '_')
    if not name:
        return jsonify(error='Location name required'), 400

    pose = _agent.mc.get_current_pose()
    if pose is None:
        return jsonify(error='Cannot read current robot pose from TF'), 503

    x, y, yaw = pose
    _agent.semantic_map[name] = {'x': x, 'y': y, 'yaw_rad': yaw, 'room': room}
    _agent.room_groups.setdefault(room, [])
    if name not in _agent.room_groups[room]:
        _agent.room_groups[room].append(name)

    _write_yaml(MAP_PATH, _agent.semantic_map)
    rospy.loginfo(f"[MAP] Saved '{name}' ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°) → {room}")
    return jsonify(success=True, name=name, x=x, y=y, yaw=yaw)


@app.route('/api/rooms')
def api_rooms():
    return jsonify(sorted(_agent.room_groups) if _agent else [])


@app.route('/api/history')
def api_history():
    with _history_lock:
        return jsonify(history=list(_history))


@app.route('/api/estop', methods=['POST'])
def api_estop():
    global _estop_active
    data = request.get_json(force=True) or {}
    with _estop_lock:
        # Explicit 'active' field, or toggle if omitted
        _estop_active = data.get('active', not _estop_active)
        state = _estop_active
    if _estop_pub:
        _estop_pub.publish(Bool(data=state))
    rospy.logwarn(f"[UI] E-stop {'ACTIVATED' if state else 'RELEASED'} via web UI")
    return jsonify(active=state)



# ── YAML persistence ────────────────────────────────────────────────────────

def _write_yaml(path: str, semantic_map: dict):
    """Merge updated coordinates into the YAML file, preserving polygon data."""
    try:
        with open(path) as f:
            doc = yaml.safe_load(f) or {'regions': []}
    except Exception:
        doc = {'regions': []}

    by_name = {r['name'].lower(): r for r in doc.get('regions', [])}
    regions = []
    for name, d in semantic_map.items():
        entry = dict(by_name.get(name, {'name': name, 'polygon': []}))
        entry.update(
            name=name, room=d['room'],
            x=round(float(d['x']), 4),
            y=round(float(d['y']), 4),
            yaw=round(float(d['yaw_rad']), 4),
        )
        regions.append(entry)

    with open(path, 'w') as f:
        yaml.dump({'regions': regions}, f, default_flow_style=False, sort_keys=False)


# ── Background threads ──────────────────────────────────────────────────────

def _agent_worker():
    """Processes one command at a time from the queue so ROS calls stay serial."""
    while not rospy.is_shutdown():
        try:
            cmd = _cmd_q.get(timeout=1.0)
        except queue.Empty:
            continue
        try:
            result = _agent.give_command(cmd)
        except Exception as e:
            result = f'[Error] {e}'
        _res_q.put(result)



# ── Entry point ─────────────────────────────────────────────────────────────

def main():
    global _agent, _estop_pub
    rospy.init_node('robot_flask_ui', anonymous=False)
    _agent = RobotAgent()

    _estop_pub = rospy.Publisher('/e_stop', Bool, queue_size=1, latch=True)

    threading.Thread(target=_agent_worker, daemon=True).start()
    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False),
        daemon=True,
    ).start()

    rospy.loginfo('[Flask UI] Running at http://0.0.0.0:5000')
    rospy.spin()


if __name__ == '__main__':
    main()
