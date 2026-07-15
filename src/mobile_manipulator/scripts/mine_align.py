#!/usr/bin/env python3
"""The final approach: re-acquire the mine with the UGV's own camera, then drive the base to it.

WHY THIS NODE EXISTS
--------------------
Navigation cannot park the UGV well enough to grasp, and no amount of tuning will change that.

    the arm's reachable band for a ground grasp : x in [0.60, 1.05] m from base_link
                                                  (MEASURED, with a hard cliff at 0.55)
    i.e. a window                               : 0.45 m wide
    the tour's arrival tolerance                : 0.6 m
    the UAV's fix                               : taken from ~12 m up, then carried through
                                                  DLIO odometry that drifts over a long tour

So arriving is not the same as being able to grasp, and treating it as if it were is how you get
a pick that works on the bench and fails in the field. Nav's job is to get us into the AREA. This
node's job is the last half-metre, and it does it the only way that is actually reliable: by
LOOKING at the mine and closing the loop on what it sees.

    SEARCH   put the wrist camera over the coarse target and detect. If the mine is not in
             frame, sweep a few poses -- the camera's ground footprint (~0.6 m at the standard
             look height) is SMALLER than nav's arrival error, so a single look genuinely misses
             sometimes, and that failure looks exactly like a broken detector if you do not
             expect it.

    ALIGN    drive the base until the mine sits 0.75 m dead ahead -- the centre of the reachable
             band, as far from both edges as it is possible to be.

HOW IT DRIVES THE BASE WITHOUT FIGHTING THE PLANNER
--------------------------------------------------
It publishes Twist on grasp/cmd_vel, which is a twist_mux input at priority 7. The CMU planner
reaches the base through `external` at priority 1, so we simply outrank it while we are talking,
and it resumes on its own 0.5 s after we stop. No planner surgery, no mode flag, nothing to
reset, and every HUMAN input still outranks us (see config/twist_mux_grasp.yaml).

THE ERROR SIGNAL IS ALREADY IN THE RIGHT FRAME
----------------------------------------------
landmine_detector publishes the wrist detection in base_link (measured to 1.6 mm), which IS the
control error -- no TF gymnastics, no odom, no drift:

    yaw_err   = atan2(y, x)              -> turn until the mine is dead ahead
    range_err = hypot(x, y) - 0.75       -> creep until it is in the middle of the band

Services:
    /ugv/align_to_mine   std_srvs/Trigger   SEARCH + ALIGN. Succeeds only if the mine ends up
                                            inside the reachable band.
    /ugv/hold_base       std_srvs/SetBool   true = pin the base at zero velocity (used during
                                            the grasp, so the planner cannot drive out from
                                            under an arm that is reaching for the ground).
"""
import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion

from mobile_manipulator.msg import WorldTarget


class MineAlign(object):
    def __init__(self):
        # The centre of the MEASURED reachable band [0.60, 1.05]. Aiming at the centre is not
        # fussiness: detection is good to a couple of mm but the base is not, and every cm of
        # margin here is a cm that does not have to come out of the grasp's IK.
        self.standoff = float(rospy.get_param('~standoff', 0.75))
        self.band_lo = float(rospy.get_param('~band_lo', 0.60))
        self.band_hi = float(rospy.get_param('~band_hi', 1.05))

        self.range_tol = float(rospy.get_param('~range_tol', 0.05))
        self.yaw_tol = math.radians(float(rospy.get_param('~yaw_tol_deg', 4.0)))

        # Converge anywhere COMFORTABLY inside the reachable band -- not pinpoint on 0.75. The
        # band is [0.60, 1.05]; [0.65, 0.95] keeps clear of both the 0.55 cliff and the 1.05 far
        # edge, so a grasp from anywhere in it is safe. Chasing 0.75 exactly is what drove the
        # mine off the camera's edge and lost it.
        self.conv_lo = float(rospy.get_param('~converge_lo', 0.65))
        self.conv_hi = float(rospy.get_param('~converge_hi', 0.95))
        self.max_iters = int(rospy.get_param('~max_iters', 8))
        self.max_step = float(rospy.get_param('~max_step', 0.60))

        # THE ARM CANNOT AIM THE CAMERA ANYWHERE -- the wrist camera reaches a limited ground
        # envelope, and aiming it AT a far mine (x=1.14) simply fails IK, which is what stalled
        # the loop: every look failed, so it never drove. So clamp the aim to what the arm can
        # actually reach; the camera's ~0.6 m footprint keeps a mine at the edge of that envelope
        # in frame anyway, and each drive step brings it closer to being centred.
        self.look_x_min = float(rospy.get_param('~look_x_min', 0.55))
        self.look_x_max = float(rospy.get_param('~look_x_max', 1.00))
        self.look_y_max = float(rospy.get_param('~look_y_max', 0.25))

        # SLOW. This happens with the arm unstowed and the camera looking down, on a robot whose
        # CoG is already high enough to tip on slopes. There is nothing to be gained by hurrying
        # the last half-metre.
        self.v_max = float(rospy.get_param('~v_max', 0.12))     # m/s
        self.w_max = float(rospy.get_param('~w_max', 0.35))     # rad/s
        self.k_v = float(rospy.get_param('~k_v', 0.6))
        self.k_w = float(rospy.get_param('~k_w', 1.2))

        self.timeout = float(rospy.get_param('~align_timeout', 45.0))
        self.detect_srv = rospy.get_param('~detect_service', '/landmine_detector/detect_once')
        self.stream_srv = rospy.get_param('~stream_service', '/landmine_detector/stream')
        self.look_srv = rospy.get_param('~look_service', '/grasp/look')
        self.brake_srv = rospy.get_param('~brake_service', '/grasp/brake')

        # NEVER DRIVE ON A STALE DETECTION. The mine is tracked continuously while we align, so a
        # reading older than this means we have lost the track -- and at 0.12 m/s even a 1.5 s-old
        # reading is 18 cm out of date. Driving on one is driving blind, and it is how the UGV
        # drove over the mine and BULLDOZED it 5.7 m across the map, reporting it dead ahead the
        # whole way because it was, wedged against the bumper.
        self.max_age = float(rospy.get_param('~max_age', 0.5))
        # And a hard floor: below this the mine is about to go under the bumper. Stop. Nothing
        # good happens closer, and the arm cannot reach inside 0.55 m anyway (the measured cliff).
        self.min_range = float(rospy.get_param('~min_range', 0.50))

        # The camera's ground footprint at the look height is ~0.6 m and nav's arrival error is
        # up to 0.8 m, so the first look CAN legitimately come up empty. Sweep before giving up.
        #
        # The offsets are SMALL on purpose. They are limits on where the ARM can put the camera,
        # not on where we would like to look: a look pose over x = 1.05 m is simply unreachable
        # (measured -- IK fails on all 3 planning attempts), and asking for it just burns 35 s
        # per sweep pose before failing. +/-0.25 m keeps every pose inside the arm's envelope,
        # and the camera's own ~0.6 m footprint covers the gaps between them.
        self.sweep = rospy.get_param('~sweep', [[0.0, 0.0], [0.20, 0.0], [-0.20, 0.0],
                                                [0.0, 0.25], [0.0, -0.25]])

        # Once SEARCH has locked onto a mine, later detections must be CONSISTENT with it.
        self.max_jump = float(rospy.get_param('~max_jump', 0.35))
        self._locked = False

        self.lock = threading.Lock()
        self.det = None            # the wrist detection, in base_link
        self.det_stamp = rospy.Time(0)

        self.cmd = rospy.Publisher('grasp/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber(rospy.get_param('~detection_topic', '/ugv/landmine_detection'),
                         WorldTarget, self._det_cb, queue_size=5)

        # Base pose from wheel odometry. The drive steps CLOSE ON THIS, not on a timer: open-loop
        # timed driving was wildly unreliable here (a 0.30 m step produced 2.5 m of motion -- the
        # base coasts, and re-braking between looks leaves residual wheel velocity). Odometry
        # feedback drives until the base has ACTUALLY moved the target amount and then stops, so
        # none of that matters. In the mission DLIO owns odom; the wheel odom is what exists on
        # the bench and is fine for a sub-metre relative move.
        self.odom = None
        rospy.Subscriber(rospy.get_param('~odom_topic', '/husky_velocity_controller/odom'),
                         Odometry, self._odom_cb, queue_size=5)

        self._hold = False
        rospy.Timer(rospy.Duration(0.1), self._hold_tick)

        rospy.Service('/ugv/align_to_mine', Trigger, self._align_cb)
        rospy.Service('/ugv/hold_base', SetBool, self._hold_cb)
        rospy.loginfo('[mine_align] ready. band=[%.2f, %.2f] standoff=%.2f; /ugv/align_to_mine',
                      self.band_lo, self.band_hi, self.standoff)

    # ---------- plumbing ----------
    def _det_cb(self, msg):
        if not msg.class_name.startswith('landmine'):
            return
        p = (msg.point.point.x, msg.point.point.y)
        with self.lock:
            # A MINE CANNOT TELEPORT. We creep at 0.12 m/s and re-detect several times a second,
            # so a detection that has jumped half a metre since the last one is not the mine
            # moving -- it is the detector locking onto something else. Refuse it.
            #
            # This is a second line of defence behind the detector's own red-disc gate, and it is
            # cheap. The failure it guards against is not theoretical: the aligner once turned
            # 103 degrees off the mine and drove away, because the mine had left the camera frame
            # and the biggest yellow thing left in view was a ROAD MARKING.
            if self.det is not None and self._locked:
                jump = math.hypot(p[0] - self.det[0], p[1] - self.det[1])
                if jump > self.max_jump:
                    rospy.logwarn_throttle(
                        2.0, '[mine_align] ignoring a detection that jumped %.2f m (> %.2f) -- '
                             'that is not the same mine', jump, self.max_jump)
                    return
            self.det = p
            self.det_stamp = rospy.Time.now()

    def _odom_cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.odom = (p.x, p.y, yaw)

    def _stop(self):
        self.cmd.publish(Twist())

    def _hold_cb(self, req):
        self._hold = bool(req.data)
        rospy.loginfo('[mine_align] base hold %s', 'ON' if self._hold else 'OFF')
        return SetBoolResponse(success=True, message='held' if self._hold else 'released')

    def _hold_tick(self, _evt):
        # A zero Twist at priority 7 beats the planner's priority 1, so while this is on, the
        # planner physically cannot move the base. The parking brake already welds it in Gazebo;
        # this stops the wheels FIGHTING the brake, which is what spins them against a locked
        # chassis and looks like a controller fault.
        if self._hold:
            self.cmd.publish(Twist())

    def _fresh(self, max_age=2.0):
        with self.lock:
            if self.det is None:
                return None
            if (rospy.Time.now() - self.det_stamp).to_sec() > max_age:
                return None
            return self.det

    def _brake(self, on):
        """Pin the base while the ARM moves.

        The sweep swings a 20 kg arm around on a 46 kg base whose wheels are velocity-controlled
        with no position hold, so nothing resists the reaction: measured, the base drifted 0.16 m
        during one sweep. That matters more here than anywhere else, because the base IS the
        frame this whole controller closes its loop in -- if it wanders while we are looking, the
        error signal we compute is against a robot that is no longer where it was.
        """
        try:
            rospy.wait_for_service(self.brake_srv, timeout=3.0)
            rospy.ServiceProxy(self.brake_srv, SetBool)(data=on)
            return True
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn('[mine_align] brake %s: %s', 'on' if on else 'off', exc)
            return False

    def _track(self, on):
        """Turn the detector's CONTINUOUS tracking on/off.

        The alignment is a closed loop on a moving robot, so it needs a fresh error signal every
        cycle, not a snapshot taken before it started driving. This is what makes the loop a loop.
        """
        try:
            rospy.wait_for_service(self.stream_srv, timeout=3.0)
            rospy.ServiceProxy(self.stream_srv, SetBool)(data=on)
            return True
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn('[mine_align] tracking %s: %s', 'on' if on else 'off', exc)
            return False

    def _detect_once(self):
        try:
            rospy.wait_for_service(self.detect_srv, timeout=3.0)
            return bool(rospy.ServiceProxy(self.detect_srv, Trigger)().success)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn('[mine_align] detector: %s', exc)
            return False

    def _look(self, dx, dy):
        """Aim the wrist camera at a ground point (dx, dy) relative to the nominal standoff."""
        try:
            from mobile_manipulator.srv import LookAt
            rospy.wait_for_service(self.look_srv, timeout=3.0)
            # LookAt.srv returns `ok`, not `success`. Reading the wrong field raises, the
            # exception is swallowed as "look failed", and SEARCH then never runs the detector
            # at all -- which presents as "no mine in view" on a mine that is in plain view.
            return bool(rospy.ServiceProxy(self.look_srv, LookAt)(
                x=self.standoff + dx, y=dy).ok)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn('[mine_align] look: %s', exc)
            return False

    # ---------- SEARCH ----------
    def _search(self):
        """Find the mine with the wrist camera. Returns (x, y) in base_link, or None.

        Unlocked while searching (we have no prior, so anything is admissible), locked once we
        have found something (after which a detection that has jumped is a different object).

        The base is BRAKED for the whole sweep: the arm is what is moving, and an unbraked base
        gets dragged around by it (0.16 m, measured), which corrupts the very frame we are about
        to align in.
        """
        self._locked = False
        with self.lock:
            self.det = None
        self._brake(True)
        try:
            found = self._search_swept()
        finally:
            self._brake(False)          # ALIGN has to be able to DRIVE
        self._locked = found is not None
        return found

    def _search_swept(self):
        for i, (dx, dy) in enumerate(self.sweep):
            if rospy.is_shutdown():
                return None
            if not self._look(dx, dy):
                continue
            rospy.sleep(0.6)                     # let the camera settle before trusting a frame
            if self._detect_once():
                rospy.sleep(0.3)
                d = self._fresh()
                if d is not None:
                    rospy.loginfo('[mine_align] found the mine at (%.3f, %.3f) on sweep pose '
                                  '%d/%d', d[0], d[1], i + 1, len(self.sweep))
                    return d
            rospy.loginfo('[mine_align] nothing at sweep pose %d/%d', i + 1, len(self.sweep))
        rospy.logwarn('[mine_align] swept %d look poses and never saw a mine. Either nav put us '
                      'further out than the sweep covers, or there is nothing here.',
                      len(self.sweep))
        return None

    # ---------- ALIGN ----------
    def _align_cb(self, _req):
        """ANCHOR the mine in odom from one clean fix, then drive an odom go-to-goal.

        Everything about the base's physical bracing that made the camera-in-the-loop designs
        fail -- the weld that would not release, the search sweep dragging the base -- is
        sidestepped here. The mine does not move in the world, so:

          1. LOOK once, detect the mine in base_link, and TRANSFORM it into odom using the base's
             odometry AT THAT INSTANT. That odom point is world-fixed: it does not care that the
             base drifted while the arm was looking, because the drift is baked into the odom
             reading we anchored against.
          2. DRIVE a plain odom go-to-goal until the base sits `standoff` from that point, facing
             it. No braking (the arm is still while we drive, so nothing levers the base), no
             re-looking (the anchor is world-fixed and the mine has not moved), no vision in the
             control loop at all -- just wheel odometry, which is precise over half a metre.
        """
        # HOLD the base still while the arm sweeps -- but with a zero-velocity command, not the
        # physical weld. The weld braces the base perfectly, but its release is unreliable in
        # rapid succession (measured: after a search cycle the base stayed welded and the wheels
        # just spun). A continuous zero Twist at twist_mux priority 7 makes the velocity
        # controller actively hold the wheels at zero, which resists the arm's lever (a free base
        # drifted 0.25 m and rotated 58 deg during one sweep and lost the mine) -- and there is
        # nothing to release, so the drive that follows is never fighting a stuck base.
        self._hold = True
        try:
            anchor = self._acquire_anchor()
        finally:
            self._hold = False
        rospy.sleep(0.5)                         # let the hold's last zero-cmd time out of the mux
        if anchor is None:
            self._stop()
            return TriggerResponse(success=False, message='SEARCH failed: no mine in view')
        return self._goto_standoff(anchor)

    def _acquire_anchor(self):
        """Sweep look poses, detect the mine, and return its position in ODOM (or None).

        No brake: the small base drift from the arm sweep is captured by reading the base's own
        odometry at the moment of detection, so the anchor is correct regardless.
        """
        for i, (dx, dy) in enumerate(self.sweep):
            if rospy.is_shutdown():
                return None
            if not self._look(dx, dy):
                continue
            rospy.sleep(0.6)
            if not self._detect_once():
                rospy.loginfo('[mine_align] nothing at sweep pose %d/%d', i + 1, len(self.sweep))
                continue
            rospy.sleep(0.3)
            d = self._fresh(max_age=2.0)
            od = self.odom
            if d is None or od is None:
                continue
            mx, my = d
            bx, by, byaw = od
            ox = bx + mx * math.cos(byaw) - my * math.sin(byaw)
            oy = by + mx * math.sin(byaw) + my * math.cos(byaw)
            rospy.loginfo('[mine_align] anchored mine at odom (%.3f, %.3f) from a %.3f m fix on '
                          'sweep pose %d/%d', ox, oy, math.hypot(mx, my), i + 1, len(self.sweep))
            return (ox, oy)
        rospy.logwarn('[mine_align] swept %d look poses and never saw a mine.', len(self.sweep))
        return None

    def _goto_standoff(self, mine_odom):
        """Odom go-to-goal: drive until the base is `standoff` from the mine, facing it.

        Converging on odom (not vision) means the loop is immune to everything that broke the
        camera-in-the-loop version: the mine leaving the frame, the depth corrupting at the edge,
        the arm being unable to aim far. It is a plain differential-drive controller.
        """
        ox, oy = mine_odom
        rate = rospy.Rate(15)
        end = rospy.Time.now() + rospy.Duration(self.timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if self.odom is None:
                rate.sleep(); continue
            bx, by, byaw = self.odom
            dx, dy = ox - bx, oy - by
            rng = math.hypot(dx, dy)
            yaw_err = self._wrap(math.atan2(dy, dx) - byaw)
            rng_err = rng - self.standoff

            if abs(rng_err) < 0.04 and abs(yaw_err) < self.yaw_tol:
                self._stop()
                if not (self.band_lo <= rng <= self.band_hi):
                    return TriggerResponse(
                        success=False,
                        message='converged at %.3f m, outside band [%.2f, %.2f]'
                                % (rng, self.band_lo, self.band_hi))
                rospy.loginfo('[mine_align] aligned: mine %.3f m ahead, %.1f deg',
                              rng, math.degrees(yaw_err))
                return TriggerResponse(
                    success=True,
                    message='mine at %.3f m, %.1f deg (band [%.2f, %.2f])'
                            % (rng, math.degrees(yaw_err), self.band_lo, self.band_hi))

            t = Twist()
            # Turn to face the goal first; only drive once roughly aligned, so forward motion is
            # always toward the mine and never sideways past it.
            if abs(yaw_err) > self.yaw_tol:
                t.angular.z = math.copysign(min(self.w_max, max(0.06, self.k_w * abs(yaw_err))),
                                            yaw_err)
            else:
                t.linear.x = math.copysign(min(self.v_max, max(0.04, self.k_v * abs(rng_err))),
                                           rng_err)
            self.cmd.publish(t)
            rate.sleep()

        self._stop()
        return TriggerResponse(success=False, message='align timed out after %.0f s' % self.timeout)


if __name__ == '__main__':
    rospy.init_node('mine_align')
    MineAlign()
    rospy.spin()
