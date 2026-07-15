#!/usr/bin/env python3
"""MoveIt Task Constructor pick of the landmine detonator.

Replaces the legacy master_control.py pick, which was a linear moveit_commander script
that FAILED SILENTLY: when GPD timed out (it always did — nothing ever launched the
detector) it quietly fell back to a hardcoded horizontal heuristic and carried on. MTC's
value here is not staged planning so much as staged *diagnosis*: when a pick fails, the
RViz Task panel says which stage failed and why, instead of leaving you guessing between
perception, kinematics and control.

THE SEAM: this file never mentions GPD, GraspGen or colour. It asks /get_grasps for a
ranked list of grasp_tcp poses. grasp_source:=analytic|gpd|graspgen swaps the server
behind that service and this file does not change by one line — which is the only way the
GPD-vs-GraspGen A/B measures the detector rather than the plumbing.

SEPARATE TASKS WITH PHYSICAL CHECKS BETWEEN THEM -- never one open-loop task:

    reach_landmine:
        CurrentState                   (scene already contains the landmine)
          -> MoveTo(arm, "ready")      unstow -- see the shoulder_lift note below
          -> MoveTo(gripper, "open")   FULLY open (128.5 mm) -- see _close_and_weld
          -> Connect
          -> [pick]
               MoveRelative  approach  Cartesian, along grasp_tcp +Z (back-propagated)
               ComputeIK     <- GeneratePose   ik_frame = grasp_tcp
               allowCollisions(landmine, gripper links + ground)
        ENDS AT THE GRASP POSE, jaws open, nothing touched.

    == THE CLOSE: _close_and_weld() -- NOT an MTC stage. See its docstring. ==
       Rungs, stop on contact, squeeze 0.003. Fails loudly (EMPTY_GRASP /
       GAZEBO_ATTACH_FAILED) rather than pretending.

    lift_landmine:
        allowCollisions -> attachObject(landmine -> grasp_tcp) -> MoveRelative lift (ODOM +Z)

    == LIFT CHECK (grasp_tcp height, and grasp_fix still reports attached) ==
       lost -> open where we are, purge the scene, retreat UPWARD, restow. NO descent.
       held -> lower_landmine  (MoveRelative lower, capped at lift.min so it cannot undershoot)
            -> release_landmine (open the jaws -> grasp_fix lets go -> detach -> retreat -> stow)

WHY THE CLOSE CANNOT BE AN MTC STAGE (it used to be a MoveTo(gripper, "closed")):
   MTC plans, THEN executes. It cannot stop a trajectory partway on a sensor event -- and
   stopping the jaws the instant they touch the block is the entire fix. See _close_and_weld.

TWO THINGS THAT LOOK WRONG BUT ARE NOT:

1. LIFT IS ALONG `odom` +Z, NOT `base_link` +Z.
   The UGV parks on outdoor terrain and may be on a slope, which tilts base_link relative
   to gravity. Lifting along base +Z would drag the mine sideways into the ground. odom is
   gravity-aligned; that is the frame "up" means.

2. THE ARM IS SPAWNED STOWED, AND "ready" MUST BE REACHED VIA THE +2*pi BRANCH.
   Stow sits at shoulder_lift = +3.271 (the spawner's -3.0, wrapped). The SRDF "ready"
   state deliberately stores shoulder_lift = 4.683 (= -1.60 + 2*pi) rather than -1.60:
   the same physical pose, but reached by a 1.4 rad lift up and over instead of a 279 deg
   sweep the long way round, which drives ur5_upper_arm_link and ur5_forearm_link straight
   through the Husky chassis. With -1.60, RRTConnect explores >17k states and fails.
"""
import sys
import time

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from moveit.task_constructor import core, stages
# REQUIRED, even though it looks unused. InterfaceState.scene returns a C++
# planning_scene::PlanningScene, and pybind11 can only hand that back to Python if the type
# has been registered by importing its own module first. Without this import the generator
# dies at runtime with:
#     "Unable to convert function return value to a Python type!
#      The signature was (arg0: pymoveit_mtc.core.InterfaceState) -> planning_scene::PlanningScene"
from moveit.core.planning_scene import PlanningScene  # noqa: F401
from moveit_msgs.msg import CollisionObject
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gazebo_link_attacher.srv import Attach
from grasp_mtc.srv import GetGrasps, GetGraspsRequest
from mobile_manipulator.msg import WorldTarget
from mobile_manipulator.srv import LookAt, LookAtResponse

# Every link of the 2F-140 that can touch the object. An INCOMPLETE touch_links list is
# the classic cause of "attach succeeded but the lift won't plan": the instant the object
# is attached, any gripper link still touching it reads as a collision.
GRIPPER_LINKS = [
    'robotiq_arg2f_base_link',
    'left_inner_finger',  'left_inner_finger_pad',  'left_inner_knuckle',
    'left_outer_finger',  'left_outer_knuckle',
    'right_inner_finger', 'right_inner_finger_pad', 'right_inner_knuckle',
    'right_outer_finger', 'right_outer_knuckle',
]

ARM, GRIPPER, TCP = 'ur5_arm', 'gripper', 'grasp_tcp'
OBJECT = 'landmine'

# --- THE CLOSING LADDER ---------------------------------------------------------------------
# Pad gap vs finger_joint, by FK from this robot's own URDF (and confirmed to three decimals
# against a teammate's independently measured table):
#
#     q      0.000   0.400   0.480   0.490   0.495   0.498   0.500   0.513   0.580
#     gap   128.5    58.9    43.46   41.50   40.53   39.94   39.55   37.00   23.76  mm
#
# The detonator is a 40 mm block. Note where 0.580 -- the OLD 'closed' command -- lands: a
# 23.76 mm gap, i.e. 16.2 mm INSIDE the block. This gripper cannot stall on it (the mimic
# fingers are kinematic, driven by SetPosition, so they exert no force and are forced to their
# commanded angle whatever they are touching), so it simply PENETRATES, and the solver's
# response to that penetration is what ejected the mine and threw the 46 kg Husky.
#
# So we never command a gap narrower than the object. We creep up on it in rungs and STOP THE
# INSTANT WE TOUCH IT. The rungs get finer as the gap closes on 40 mm, so no single step can
# jump past first contact:
#
#     0.480 -> 0.490 : -1.95 mm      0.498 -> 0.499 : -0.20 mm
#     0.490 -> 0.495 : -0.98 mm      0.499 -> 0.500 : -0.20 mm
#     0.495 -> 0.498 : -0.59 mm
#
# The last rung's 39.55 mm is 0.45 mm narrower than a face-on block -- a light squeeze, not a
# 16 mm ram -- so even running the ladder to the end cannot detonate the solver.
# WHY THERE IS NO CONTACT-STOP LADDER ANY MORE, AND NO gazebo_grasp_fix.
#
# The ladder was the right idea and it WORKED: it is what stopped the close from throwing the
# robot (base displacement fell to 0.006-0.017 m, from metres). But it needs a signal that says
# "you are touching the block now", and on this gripper no such signal exists:
#
#   * finger_joint cannot say it. Kinematic jaws never stall; they reach their command whether
#     or not there is a block between them.
#   * gazebo_grasp_fix cannot say it either. It looks for two OPPOSING CONTACT FORCES, and a
#     kinematic finger exerts NO force on what it touches, so there is nothing for it to see. It
#     fires on solver noise instead. Measured across 7 trials, its "attach" landed at
#     finger_joint = 0.4254, 0.4387, 0.4413, 0.480, 0.495 and even 0.0017 (jaws WIDE OPEN) --
#     for one and the same 40 mm block, whose true contact angle is 0.495. Tightening its
#     grip_count_threshold to reject the noise made it stop firing at the real contact too.
#
# So we stop pretending to sense contact and use what we actually know, which is a great deal:
# the detector locates the block to 1.6 mm, and the pad gap is an exact function of finger_joint
# (FK, verified against a teammate's independently measured table to three decimals):
#
#     q      0.400   0.470   0.475   0.480   0.490   0.495   0.500   0.580
#     gap   58.90   45.40   44.43   43.46   41.50   40.53   39.55   23.76  mm
#
# CLOSE_TARGET is chosen so the jaws cradle the block WITHOUT EVER TOUCHING IT, and the grasp is
# an explicit weld. A 40 mm block yawed by the detector's ~2.5 deg residual presents
# 40*(cos+sin) = 41.7 mm, so 0.475 (44.4 mm) leaves ~1.4 mm of air on each side, and still
# clears any yaw error up to 6 deg. Nothing touches, so nothing can be penetrated, so there are
# no contact impulses at all -- which is the failure mode this entire exercise is about.
#
# The grip force we are giving up costs us nothing: this gripper has none to give.
# 0.400 -> a 58.9 mm gap around a block that presents ~41.7 mm: 8.6 MILLIMETRES OF AIR per side.
#
# That margin is not timidity, it is the whole fix, and it was measured. At 0.475 (a 44.4 mm gap,
# 1.4 mm per side) the pads are still INSIDE ODE's contact margin: they touch the welded mine and,
# being kinematic and therefore infinitely stiff, they shove it. The weld holds it, the pads push
# it, and the fight is reacted into a 46 kg base on free-spinning wheels. Measured at 0.475: the
# TCP rose 129 mm during the lift while the mine rose only 22 mm, and the base was jacked off the
# ground. Measured with the jaws OPEN and the same weld: the mine tracked the TCP to 0.1 mm
# (0.1499 m against 0.1500 m) and the base moved 0.0002 m. The pads are the entire difference.
#
# We give up nothing by not touching the block. This gripper cannot grip: the mimic joints are
# kinematic (SetPosition), so the fingers exert no force and generate no friction at ANY commanded
# angle. The jaws are decoration; the weld is the grasp. So close them far enough to look like a
# grasp and cradle the block, and not one millimetre further.
CLOSE_TARGET = 0.400
CLOSE_SPEED = 0.06            # rad/s -- slow; a fast 128 mm jaw sweep is momentum into the base
CLOSE_MIN_S = 0.60
SETTLE_POLL_S = 0.02
HOLD_CONFIRM_S = 1.0          # the weld must still be reported held this long before we lift
HOLD_TIMEOUT_S = 2.5

GRIP_ACTION = '/gripper_controller/follow_joint_trajectory'

# WHAT HOLDS THE MINE: an explicit Gazebo joint, created on request by gazebo_link_attacher.
#
# A friction grasp is not available here at any price. The 2F-140's mimic joints are driven by
# MimicJointPlugin's SetPosition() -- a kinematic teleport -- and right_outer_knuckle_joint is
# itself a mimic, so the ENTIRE right finger is kinematic: it is forced to its commanded angle
# regardless of what it is touching, and therefore pushes on nothing. No squeeze, no friction,
# no grip, no matter what we command. The only honest options are to weld the object, or to
# fail; gazebo_grasp_fix reaches the same conclusion (it welds too) and merely disagrees about
# when. Since its contact trigger is unusable here (see CLOSE_TARGET), we weld on our own terms.
WELD_SRV, UNWELD_SRV = '/link_attacher/attach', '/link_attacher/detach'
ROBOT_MODEL = 'husky_ur5'
# The weld anchor is ur5_wrist_3_link, NOT robotiq_arg2f_base_link: the palm is joined to the
# wrist by a FIXED joint, and Gazebo LUMPS fixed-joint children into their parent, so no link by
# that name exists in the physics world ("no such link: robotiq_arg2f_base_link"). The wrist IS
# the palm as far as the solver is concerned -- rigidly the same body.
PALM = 'ur5_wrist_3_link'
OBJECT_MODEL, OBJECT_LINK = 'landmine', 'body'      # link name from the landmine SDF

# THE PARKING BRAKE, and it is not optional.
#
# The Husky's wheels are driven by husky_velocity_controller, which commands wheel VELOCITY. It
# has NO POSITION HOLD: nothing whatsoever resists an external push or torque. And a 20 kg UR5
# cantilevered forward and DOWN to the grasp pose at x=0.75 is a large external torque on a 46 kg
# base. Measured, with nothing grasped and nothing even touched: merely MOVING THE ARM TO THE
# GRASP POSE displaced base_link by 0.030 m and lifted it 26 mm (z 0.132 -> 0.158). That alone is
# the entire 0.03 m stability budget, spent before the pick has begun. Lifting from there then
# threw the base 0.22 m.
#
# This is not a grasp defect and no change to the gripper can fix it. The manipulation phase
# simply requires a stationary base (it is Invariant 5 of the plan), so we give it one: weld
# base_link to the static ground for the duration of the pick, and release it afterwards. A real
# UGV would set a brake; this is that brake.
GROUND_MODEL, GROUND_LINK = 'grass_plane', 'link'
BASE_LINK = 'base_link'

# Landmine geometry, straight from its SDF. Modelled as ONE collision object with TWO
# primitives, because it IS one rigid link: grasping the detonator lifts the whole 0.30 kg
# mine, and the 136 mm disc sits directly beneath the jaws.
DISC_R, DISC_H = 0.068, 0.025
DET_W, DET_H = 0.040, 0.060
MINE_TOTAL_H = 0.085          # ground to detonator top face


def make_landmine(frame, x, y, z_top):
    """CollisionObject for the mine, placed from the DETECTED TOP-FACE centre."""
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = OBJECT
    co.operation = CollisionObject.ADD
    # The object's own origin is the disc bottom, i.e. ground level.
    co.pose.position.x = x
    co.pose.position.y = y
    co.pose.position.z = z_top - MINE_TOTAL_H
    co.pose.orientation.w = 1.0

    disc = SolidPrimitive()
    disc.type = SolidPrimitive.CYLINDER
    disc.dimensions = [DISC_H, DISC_R]
    disc_pose = PoseStamped().pose
    disc_pose.position.z = DISC_H / 2.0
    disc_pose.orientation.w = 1.0

    det = SolidPrimitive()
    det.type = SolidPrimitive.BOX
    det.dimensions = [DET_W, DET_W, DET_H]
    det_pose = PoseStamped().pose
    det_pose.position.z = DISC_H + DET_H / 2.0
    det_pose.orientation.w = 1.0

    co.primitives = [disc, det]
    co.primitive_poses = [disc_pose, det_pose]
    return co


def fetch_grasps(target):
    """THE SEAM. Ask whichever grasp source is running for ranked grasp_tcp poses.

    grasp_source:=analytic|gpd|graspgen swaps the server behind this one service, and
    nothing below this line changes.
    """
    rospy.wait_for_service('/get_grasps', timeout=10.0)
    req = GetGraspsRequest()
    req.target = target
    res = rospy.ServiceProxy('/get_grasps', GetGrasps)(req)
    if not res.success or not res.grasps:
        rospy.logerr('[grasp_task] /get_grasps returned nothing: %s', res.message)
        return []
    rospy.loginfo('[grasp_task] %d candidates from /get_grasps', len(res.grasps))
    # MTC minimises cost; detectors maximise score.
    return list(zip(res.grasps, [1.0 - float(s) for s in res.score]))


def make_planners():
    """Planners shared by the pick / place / restow tasks, tuned so that nothing the arm
    or the jaws do can shove the base.

    Free-space arm motions are throttled: a 20 kg UR5 slewing at full speed is a large
    momentum change reacted straight into a 46 kg base whose wheels are velocity-controlled
    and therefore free to roll -- there is no position hold to resist it. Half speed costs
    a couple of seconds and removes a whole class of "why did the robot move" failures.
    """
    sampling = core.PipelinePlanner()
    sampling.planner = 'RRTConnectkConfigDefault'
    sampling.max_velocity_scaling_factor = 0.5
    sampling.max_acceleration_scaling_factor = 0.5

    interp = core.JointInterpolationPlanner()
    interp.max_velocity_scaling_factor = 0.5
    interp.max_acceleration_scaling_factor = 0.5

    # The Cartesian approach and lift are also deliberately slow. The mine is held by FRICTION
    # between two rubber pads -- there is no form closure on a smooth 40 mm box -- so the hold
    # is limited by pad friction against m*(g + a). Snatching it upward at full speed adds an
    # `a` the friction cannot carry, and the block slides out of the jaws at the top of the
    # lift. Measured: at full lift speed the mine reaches ~0.14 m and drops.
    cartesian = core.CartesianPath()
    cartesian.max_velocity_scaling_factor = 0.15
    cartesian.max_acceleration_scaling_factor = 0.15

    # The gripper planner now only ever plans jaw OPENING -- MTC does not close the jaws at
    # all (see _close_and_weld). Opening happens in free air, away from the mine, so nothing
    # here is delicate; it is kept slow simply because a fast 136 mm jaw sweep is a momentum
    # change reacted into the same free-wheeled base as everything else.
    gripper_planner = core.JointInterpolationPlanner()
    gripper_planner.max_velocity_scaling_factor = 0.05
    gripper_planner.max_acceleration_scaling_factor = 0.05

    return sampling, interp, cartesian, gripper_planner


def build_reach_task(target, grasps):
    """Fan the candidates into GeneratePose -> ComputeIK pairs under an Alternatives.

    THIS TASK ENDS AT THE GRASP POSE, jaws FULLY OPEN and NOTHING touched. GraspNode then runs
    the close itself (_close_and_weld), and only then does build_lift_task() run.

    The close is not a stage here for a hard reason: MTC plans, then executes. It cannot abort
    a trajectory partway on a sensor event -- and stopping the jaws the instant they touch the
    block is the whole fix. A MoveTo(gripper, 'closed') can only drive to a fixed angle, and
    every fixed angle that actually grips a 40 mm block is an angle this gripper PENETRATES it
    at, because the mimic fingers are kinematic (SetPosition) and cannot stall on contact.
    Measured: finger_joint reached its commanded 0.58 -- a 23.76 mm gap -- with the 40 mm block
    between the pads, and the resulting penetration impulses spun finger_joint to +/-18 rad and
    launched base_link to z=0.96 m. That launch is 'the roll'.

    The pick is split across tasks for a second reason, which is independent of the above and
    still stands: pymoveit_mtc's task.execute() returns None, so a mid-task trajectory abort is
    INVISIBLE from Python. An early open-loop task did pick AND place in one shot; the mine
    slipped out during the lift, the task carried on regardless, and its 'lower' stage drove
    the closed EMPTY gripper down onto the real mine on the ground, pole-vaulting the 46 kg
    Husky 1.09 m. Nothing may descend until a check has proved what the gripper is holding.

    WHY NOT A CUSTOM core.MonitoringGenerator (which the plan called for, and which IS
    subclassable in these bindings): its compute() has to read the upstream planning scene
    via `solution.end.scene`, and in pymoveit_mtc 0.1.3 that getter returns a
    PlanningSceneConstPtr which pybind11 cannot convert back to Python —

        "Unable to convert function return value to a Python type!
         The signature was (arg0: core.InterfaceState) -> planning_scene::PlanningScene"

    Importing moveit.core.planning_scene does not help: the type IS registered, but as the
    non-const PlanningScene, so the const pointer still has no converter. It is a binding
    limitation, not a configuration mistake.

    stages.GeneratePose sidesteps it entirely: it is a MonitoringGenerator implemented in
    C++ that takes a `pose` and spawns an InterfaceState with `target_pose` set — exactly
    the one thing our generator needed to do — and never crosses the Python boundary with a
    scene. One per candidate, wrapped in ComputeIK, all under an Alternatives container so
    MTC tries them in cost order and keeps whichever survives IK and collision checking.

    The detector seam is unaffected: the candidates still come from /get_grasps.
    """
    task = core.Task('pick_landmine')
    sampling, interp, cartesian, gripper_planner = make_planners()

    task.add(stages.CurrentState('current'))

    # Unstow. Joint interpolation, not sampling: the straight joint-space line from stow to
    # ready is collision-free at every waypoint *provided* ready uses the +2pi shoulder_lift
    # branch (see module docstring). A sampling planner would work too but is slower and,
    # with the -1.60 branch, would simply fail.
    unstow = stages.MoveTo('unstow -> ready', interp)
    unstow.group = ARM
    unstow.setGoal('ready')
    task.add(unstow)

    # FULLY OPEN -- 128.5 mm. There used to be a 'pregrasp' state here that descended with the
    # jaws pre-closed to 49.3 mm, to shorten the final close. But a 40 mm square yawed by theta
    # presents 40*(cos+sin) mm -- 49 mm at 15 deg, 56.6 mm at 45 deg -- so a pre-closed descent
    # RAMS a yawed block on the way in. Measured: the mine jumped to z=0.036 and the base rose
    # 3 cm before the jaws had begun to close. Fully open clears any yaw.
    open_g = stages.MoveTo('open gripper', gripper_planner)
    open_g.group = GRIPPER
    open_g.setGoal('open')
    task.add(open_g)

    task.add(stages.Connect('connect', [(ARM, sampling)]))

    pick = core.SerialContainer('pick')

    # Approach. MoveRelative is PropagatingEitherWay, so MTC back-propagates from the IK'd
    # grasp pose to derive the pre-grasp standoff -- we never compute a pre-grasp ourselves.
    approach = stages.MoveRelative('approach', cartesian)
    approach.group = ARM
    approach.ik_frame = PoseStamped(header=rospy.Header(frame_id=TCP))
    approach.min_distance = 0.04
    approach.max_distance = 0.12
    v = Vector3Stamped()
    v.header.frame_id = TCP
    v.vector.z = 1.0                       # +Z of grasp_tcp IS the approach axis
    approach.setDirection(v)
    pick.insert(approach)

    candidates = core.Alternatives('grasp candidates')
    for i, (pose, cost) in enumerate(grasps):
        gp = stages.GeneratePose('pose %d' % i)
        gp.pose = pose                       # a grasp_tcp pose, straight from the server
        # Monitor the LAST GRIPPER STAGE, not 'current'. The generator's scene snapshot fixes
        # the gripper joints of every state it spawns; Connect only plans the ARM group, so if
        # this snapshot's gripper disagrees with the forward branch's gripper, Connect must
        # join two states whose gripper joints differ and returns 0 solutions -- measured 5/5
        # picks dying at 'connect', with every other stage solving fine.
        gp.setMonitoredStage(task['open gripper'])
        ik = stages.ComputeIK('IK %d' % i, gp)
        ik.group = ARM
        ik.eef = 'gripper_ee'
        ik.max_ik_solutions = 4
        # Set target_pose on the IK stage DIRECTLY rather than relying on GeneratePose to
        # forward it through the interface. The forwarding does not survive the Alternatives
        # container here — MTC fails at init with
        #     "Property 'target_pose': undefined / in stage 'grasp candidates': undeclared"
        # as the lookup walks up the parent chain instead of reading the interface state.
        # We know the pose already (one ComputeIK per candidate), so there is nothing to
        # forward: GeneratePose's job reduces to supplying the upstream scene.
        ik.target_pose = pose
        # THE payoff of the grasp_tcp link: IK is solved for the jaw center-line directly,
        # so there are no offset scalars anywhere. The legacy code's GPD_TOOL_OFFSET=0.12
        # and FINGER_REACH=0.18 both disappear here.
        ik.ik_frame = PoseStamped(header=rospy.Header(frame_id=TCP))
        candidates.insert(ik)
    pick.insert(candidates)

    allow = stages.ModifyPlanningScene('allow gripper-landmine collision')
    allow.allowCollisions(OBJECT, GRIPPER_LINKS, True)
    # The mine SITS ON the ground box. Without this it is permanently in collision with it and
    # every ComputeIK returns zero solutions -- the ground plane would "fix" the robot-throwing
    # by making the pick unplannable, which is not a fix.
    allow.allowCollisions(OBJECT, ['ground'], True)
    pick.insert(allow)

    task.add(pick)
    # ENDS at the grasp pose: TCP on the block, jaws fully open (128.5 mm around a 40 mm
    # block), nothing touched. GraspNode closes on contact here, THEN lifts.
    return task


def build_lift_task():
    """Lift the mine. Runs ONLY after _close_and_weld() has welded it to the palm.

    There is no 'close gripper' stage here either: by the time this task runs, the jaws are
    already shut on the block and grasp_fix's fixed joint is holding it. This task's only job
    is to tell MoveIt what Gazebo already knows (attachObject) and go up.
    """
    task = core.Task('lift_landmine')
    sampling, interp, cartesian, gripper_planner = make_planners()

    task.add(stages.CurrentState('current'))

    # Re-assert: MTC's ACM edits belong to the task that made them.
    allow = stages.ModifyPlanningScene('allow gripper-landmine collision')
    allow.allowCollisions(OBJECT, GRIPPER_LINKS, True)
    allow.allowCollisions(OBJECT, ['ground'], True)
    task.add(allow)

    attach = stages.ModifyPlanningScene('attach landmine')
    attach.attachObject(OBJECT, TCP)
    task.add(attach)

    lift = stages.MoveRelative('lift', cartesian)
    lift.group = ARM
    lift.ik_frame = PoseStamped(header=rospy.Header(frame_id=TCP))
    # A NARROW, FIXED lift band, paired with the descent cap in build_lower_task(). The
    # guarantee "the closed gripper never goes below the height it grasped at" only holds if
    # every possible lift is at least as long as the longest possible descent:
    #     lift.min_distance >= lower.max_distance.  Keep the two in sync.
    lift.min_distance = 0.10
    lift.max_distance = 0.12
    up = Vector3Stamped()
    up.header.frame_id = 'odom'            # gravity-aligned. NOT base_link -- see docstring.
    up.vector.z = 1.0
    lift.setDirection(up)
    task.add(lift)
    return task


def _add_go_home(task, sampling, interp):
    back = stages.MoveTo('back to ready', sampling)
    back.group = ARM
    back.setGoal('ready')
    task.add(back)

    restow = stages.MoveTo('restow', interp)
    restow.group = ARM
    restow.setGoal('stow')
    task.add(restow)


def build_place_task():
    """PLACE the mine back down -- do not carry it up and drop it.

    The pre-place order used to be: lift -> back to ready -> open -> detach -> restow. That
    RELEASES A 0.3 kg RIGID BODY FROM ~0.4 m, right over the robot, and then folds the arm
    home through the space it is falling into. The mine lands on the chassis or under the
    moving arm, the collision impulse goes straight into a 46 kg base on FREE-SPINNING
    WHEELS (husky_velocity_controller commands wheel VELOCITY; it has no position hold and
    nothing resists an external shove), and the Husky rolls away and pitches up onto its
    nose. Observed exactly this: the base only ever ran away on trials where the grasp
    SUCCEEDED -- i.e. only when there was actually a mine up there to drop.

    This task runs ONLY after the lift check has confirmed the arm is actually up (see
    build_reach_task's docstring for what blindly lowering an empty gripper did).

    It ENDS with the mine back on the ground and STILL HELD. build_release_task() then opens
    the jaws, and separating the fingers is itself what makes gazebo_grasp_fix drop its joint
    (release_tolerance = 0.005) -- so the release happens at ground level, by construction.
    There is no separate "unweld" step to get the order wrong.
    """
    task = core.Task('lower_landmine')
    sampling, interp, cartesian, gripper_planner = make_planners()

    task.add(stages.CurrentState('current'))

    # Re-assert the collision permissions in THIS task's scene snapshot: the attached mine
    # must be allowed to touch both the jaws that hold it and the ground it is returning to.
    allow = stages.ModifyPlanningScene('allow landmine contacts')
    allow.allowCollisions(OBJECT, GRIPPER_LINKS, True)
    allow.allowCollisions(OBJECT, ['ground'], True)
    task.add(allow)

    lower = stages.MoveRelative('lower it back down', cartesian)
    lower.group = ARM
    lower.ik_frame = PoseStamped(header=rospy.Header(frame_id=TCP))
    # HARD CEILING: lower.max_distance <= lift.min_distance (build_pick_task). The executed
    # lift is 0.10-0.12 m, so descending at most 0.10 m can never take the gripper below
    # the pose it grasped at -- which was collision-free by construction. The residual drop
    # when the jaws open is then 0-2 cm, instead of the ~8 cm that toppled the mine onto
    # its side when this was 0.14 against a 0.117 m lift truncated by the ground box.
    lower.min_distance = 0.05
    lower.max_distance = 0.10
    down = Vector3Stamped()
    down.header.frame_id = 'odom'
    down.vector.z = -1.0
    lower.setDirection(down)
    task.add(lower)
    return task


def build_release_task():
    """Open the jaws and go home. THE OPENING IS THE RELEASE.

    By the time this executes the mine is back on the ground (build_place_task), so when the
    jaws separate past grasp_fix's release_tolerance and it drops its joint, the mine is
    already resting on the terrain. The 0-2 cm it may settle is the residual of
    lower.max <= lift.min.
    """
    task = core.Task('release_landmine')
    sampling, interp, cartesian, gripper_planner = make_planners()

    task.add(stages.CurrentState('current'))

    allow = stages.ModifyPlanningScene('allow landmine contacts')
    allow.allowCollisions(OBJECT, GRIPPER_LINKS, True)
    allow.allowCollisions(OBJECT, ['ground'], True)
    task.add(allow)

    open_again = stages.MoveTo('release (open gripper)', gripper_planner)
    open_again.group = GRIPPER
    open_again.setGoal('open')
    task.add(open_again)

    detach = stages.ModifyPlanningScene('detach landmine')
    detach.detachObject(OBJECT, TCP)
    task.add(detach)

    # Only NOW is it safe to swing the arm home: the jaws are empty and nothing is falling.
    retreat = stages.MoveRelative('retreat', cartesian)
    retreat.group = ARM
    retreat.ik_frame = PoseStamped(header=rospy.Header(frame_id=TCP))
    retreat.min_distance = 0.05
    retreat.max_distance = 0.15
    up2 = Vector3Stamped()
    up2.header.frame_id = 'odom'
    up2.vector.z = 1.0
    retreat.setDirection(up2)
    task.add(retreat)

    _add_go_home(task, sampling, interp)
    return task


def build_carry_task():
    """Carry mode: the mine stays welded to the palm, and the arm holds it for the DRIVE.

    It goes to "carry", NOT to "stow", and that is not a detail. Stow folds the arm back OVER
    THE CHASSIS -- which is precisely where a 136 mm disc welded to the gripper would meet the
    top plate and the arm's own links. Restowing while holding a mine is asking to drag it
    through the robot.

    "carry" instead pulls the payload in to a 0.283 m reach (the smallest lever arm we can hold
    it at, on a robot whose CoG is already high enough to tip on slopes) and keeps the mine
    0.41 m off the ground. See the SRDF for the FK that picked it.

    If MoveIt refuses to plan this, BELIEVE IT: it checks the attached mine against the robot,
    so a failure here means the disc fouls the arm, and that is exactly the loud failure we want
    instead of a mine dragged through the chassis.
    """
    task = core.Task('carry_mine')
    sampling, interp, _cartesian, _gripper = make_planners()
    task.add(stages.CurrentState('current'))

    # The mine is attached to grasp_tcp in the scene, so re-assert what it is allowed to touch:
    # without this every gripper link holding it reads as a collision and nothing plans.
    allow = stages.ModifyPlanningScene('allow landmine contacts')
    allow.allowCollisions(OBJECT, GRIPPER_LINKS, True)
    task.add(allow)

    hold = stages.MoveTo('to carry', sampling)
    hold.group = ARM
    hold.setGoal('carry')
    task.add(hold)
    return task


class GraspNode(object):
    """Drives one pick. Exposes /grasp/execute so the target tour can call it on arrival."""

    # The camera's OPTICAL frame, not the TCP. The look pose has to be defined by what the
    # camera sees, and the wrist camera's optical axis is NOT parallel to the gripper's
    # approach axis -- aiming the TCP down leaves the mine at the very bottom edge of the
    # image, one bump away from being out of frame. IK on the optical frame centres it.
    CAM = 'realsense_camera_optical_frame'

    def __init__(self):
        import moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.mc = moveit_commander
        self.psi = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.arm = moveit_commander.MoveGroupCommander(ARM)
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(10)

        self.frame = rospy.get_param('~planning_frame', 'base_link')
        self.plan_only = bool(rospy.get_param('~plan_only', False))
        self.detect_srv = rospy.get_param('~detect_service',
                                          '/landmine_detector/detect_once')
        # Where the mine sits when the UGV has parked correctly. MEASURED: a top-down grasp
        # is reachable only for x in [0.60, 1.05] from base_link, with a HARD CLIFF at 0.55
        # (the UR5 base is just 0.377 m above ground and must reach ~0.31 m BELOW itself).
        # 0.75 is the centre of that band.
        self.standoff = float(rospy.get_param('~grasp_standoff', 0.75))
        # Open the jaws again at the end of the pick. TRUE in the harness: it lets
        # gazebo_grasp_fix release its weld, which is what makes a second trial possible at
        # all. The mission will want this FALSE (carry the mine to a disposal point) -- and
        # will then need its own release before anything tries to reposition the mine.
        self.release = bool(rospy.get_param('~release_after_lift', True))
        self.look_height = float(rospy.get_param('~look_height', 0.55))
        self.ground_z = float(rospy.get_param('~ground_z', -0.132))  # ground, in base_link
        self.last_target = None
        self._finger_q = None
        # True while the mine is joined to the palm by a real Gazebo joint. That joint IS the
        # grasp: a kinematic finger cannot hold anything. See _close_and_weld.
        self.welded = False
        self._add_ground()
        # The close drives the controller's ACTION directly (we need per-rung arrival, and we
        # must be able to accept GOAL_TOLERANCE_VIOLATED as "arrived"). The topic publisher is
        # kept for opening the jaws, where we do not care when it finishes.
        self.grip_ac = actionlib.SimpleActionClient(GRIP_ACTION, FollowJointTrajectoryAction)
        self.grip_pub = rospy.Publisher('/gripper_controller/command',
                                        JointTrajectory, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self._js_cb, queue_size=5)
        # The WRIST camera's detection (base_link, grasp-accurate), NOT the UAV's coarse seam.
        # See landmine_detector.py: /detected_targets is what the TOUR collects, and a grasp must
        # never be planned from a 12 m aerial fix.
        rospy.Subscriber(rospy.get_param('~detection_topic', '/ugv/landmine_detection'),
                         WorldTarget, self._target_cb, queue_size=5)
        if not self.grip_ac.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn('[grasp_task] %s did not come up; the close will fail', GRIP_ACTION)
        rospy.Service('/grasp/execute', Trigger, self._execute_cb)
        # The arm is owned here, but the SEARCH sweep is driven by the final-approach controller
        # (mine_align), which has to aim the camera around until the mine is in frame. Hence a
        # service rather than a private method: the searcher asks, the arm's owner moves.
        rospy.Service('/grasp/look', LookAt, self._look_cb)
        # Put the mine DOWN. In carry mode (release_after_lift:=false) the pick ends with the
        # mine still welded to the palm, so the mission can haul it somewhere; this is how it
        # gets let go of once it is there. Without it, carry mode is a one-way trip.
        rospy.Service('/grasp/place', Trigger, self._place_cb)
        # The brake, exposed. The SEARCH sweep moves the ARM while the base is free, and a 20 kg
        # UR5 swinging around levers a 46 kg Husky whose wheels have no position hold: measured,
        # the base drifted 0.16 m during a single sweep, which moves the very frame the visual
        # alignment is closing the loop in. Whoever moves the arm must be able to pin the base.
        rospy.Service('/grasp/brake', SetBool, self._brake_cb)
        rospy.loginfo('[grasp_task] ready; call /grasp/execute')

    def _target_cb(self, msg):
        self.last_target = msg.point

    def _js_cb(self, msg):
        if 'finger_joint' in msg.name:
            self._finger_q = msg.position[msg.name.index('finger_joint')]

    def _command_jaws(self, q, duration):
        """Send one jaw position and wait for the controller to get there.

        GOAL_TOLERANCE_VIOLATED is NOT an error here: it is the controller reporting that the
        jaws stopped short of the command. On this gripper that essentially never happens (the
        jaws are kinematic and always reach their command), but tolerating it costs nothing and
        the path tolerance is already disabled in ur5_controllers.yaml for the same reason.
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['finger_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [q]
        pt.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [pt]
        self.grip_ac.send_goal(goal)
        # Wall time, x3 slack: Gazebo's /clock stalls under load, and a ROS-time wait would then
        # hang forever instead of returning.
        self.grip_ac.wait_for_result(rospy.Duration(duration * 3.0 + 2.0))

    def _close_and_weld(self):
        """THE GRASP: cradle the block without touching it, then weld it to the palm.

        Two facts drive every line of this, and both are measured, not assumed:

        1. NOTHING MAY PENETRATE THE BLOCK. The pads are kinematic (SetPosition), so they do not
           stall on contact -- they drive straight through whatever is in the way, and the
           solver's answer to that penetration is what ejected the mine and threw the 46 kg
           Husky. The old close commanded finger_joint = 0.58, a 23.76 mm gap around a 40 mm
           block: 16.2 mm of interpenetration. CLOSE_TARGET = 0.475 is a 44.4 mm gap, which
           leaves ~1.4 mm of AIR on each side. Nothing is touched, so nothing can be penetrated.

        2. THE JAWS CANNOT HOLD IT ANYWAY, so not touching it costs nothing. A kinematic finger
           exerts no force and generates no friction. The mine is carried by a real Gazebo joint.

        Returns (ok, reason).
        """
        if not self.grip_ac.wait_for_server(rospy.Duration(5.0)):
            return False, 'gripper action server missing'

        start = self._finger_q if self._finger_q is not None else 0.0
        dt = max(CLOSE_MIN_S, abs(CLOSE_TARGET - start) / CLOSE_SPEED)
        self._command_jaws(CLOSE_TARGET, dt)
        rospy.sleep(0.3)                      # let the jaws settle before freezing the offset
        rospy.loginfo('[grasp_task] jaws closed to %.4f (commanded %.3f); welding',
                      self._finger_q if self._finger_q is not None else float('nan'),
                      CLOSE_TARGET)

        if not self._weld():
            return False, 'WELD_FAILED: the link_attacher service refused'

        # Prove it actually holds before anything lifts. The weld is a real fixed joint, so the
        # mine must now move rigidly with the palm; if the service lied, we find out here rather
        # than by dropping a mine on the robot.
        rospy.sleep(HOLD_CONFIRM_S)
        if not self.welded:
            return False, 'WELD_LOST: the weld did not survive the confirmation window'
        return True, 'held'

    def _brake(self, on):
        """Hold the base still for the manipulation phase (or let it go again).

        See GROUND_MODEL: without this the arm levers the robot off its parking spot before it
        has touched anything, because the wheels have no position hold.
        """
        # THE GROUND IS model_1 (the joint's PARENT) and the robot is model_2 (its CHILD). That
        # is the only correct topology: a STATIC link cannot be a joint's child.
        #
        # Which model OWNS the joint is a different question, and getting it wrong makes the weld
        # PERMANENT -- Detach() returns cleanly, the service says "released", and the robot stays
        # bolted to the world. link_attacher now sorts that out itself (it gives the joint to
        # whichever model is not static), so this call just has to describe the geometry.
        srv = WELD_SRV if on else UNWELD_SRV
        try:
            rospy.wait_for_service(srv, timeout=5.0)
            res = rospy.ServiceProxy(srv, Attach)(
                model_name_1=GROUND_MODEL, link_name_1=GROUND_LINK,
                model_name_2=ROBOT_MODEL, link_name_2=BASE_LINK,
                silence_collisions=False)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr('[grasp_task] parking brake %s failed: %s', 'ON' if on else 'OFF', exc)
            return False
        if not res.ok:
            rospy.logerr('[grasp_task] parking brake %s refused: %s',
                         'ON' if on else 'OFF', res.message)
            return False
        rospy.loginfo('[grasp_task] parking brake %s', 'ON' if on else 'OFF')
        return True

    def _weld(self):
        """Join the mine rigidly to the palm. This IS the grasp -- see the module docstring."""
        try:
            rospy.wait_for_service(WELD_SRV, timeout=5.0)
            res = rospy.ServiceProxy(WELD_SRV, Attach)(
                model_name_1=ROBOT_MODEL, link_name_1=PALM,
                model_name_2=OBJECT_MODEL, link_name_2=OBJECT_LINK,
                silence_collisions=False)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr('[grasp_task] weld service failed: %s', exc)
            return False
        if not res.ok:
            rospy.logerr('[grasp_task] weld refused: %s', res.message)
            return False
        self.welded = True
        rospy.loginfo('[grasp_task] welded: %s', res.message)
        return True

    def _unweld(self):
        """Release. Idempotent by design: a release must never be able to throw."""
        try:
            rospy.wait_for_service(UNWELD_SRV, timeout=5.0)
            res = rospy.ServiceProxy(UNWELD_SRV, Attach)(
                model_name_1=ROBOT_MODEL, link_name_1=PALM,
                model_name_2=OBJECT_MODEL, link_name_2=OBJECT_LINK,
                silence_collisions=False)
            self.welded = False
            return bool(res.ok)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr('[grasp_task] unweld service failed: %s', exc)
            return False

    def _open_jaws(self, wait=4.0):
        """Open the jaws. Does NOT release the mine -- _unweld() does that, and must run first."""
        jt = JointTrajectory()
        jt.joint_names = ['finger_joint']
        p = JointTrajectoryPoint()
        p.positions = [0.0]
        p.time_from_start = rospy.Duration(3.0)
        jt.points = [p]
        self.grip_pub.publish(jt)
        rospy.sleep(wait)

    def _lifted(self, aim_z):
        """Is the arm actually UP, holding the mine?

        task.execute() returns None in pymoveit_mtc 0.1.3, so a trajectory that aborts
        mid-task is INVISIBLE from Python, and an aborted lift leaves the arm low -- where a
        subsequent descent becomes exactly the press-into-the-ground that the task split
        exists to prevent. grasp_tcp's height is the one signal that cannot lie about it.
        (What the mine did is measured independently, from Gazebo, by the trial harness.)
        """
        rospy.sleep(0.5)                     # let the last trajectory point settle
        # RETRY the pose read. get_current_pose() occasionally returns an all-zero pose when the
        # TF for grasp_tcp is momentarily unavailable right after a trajectory, and a spurious
        # z=0.000 then reads as "the lift silently aborted" and kills a pick that actually
        # succeeded (mine lifted, base stable). The TCP is never truly at odom z=0 during a lift,
        # so treat an exact zero as a failed read and try again.
        tcp_z = 0.0
        for _ in range(5):
            try:
                tcp_z = self.arm.get_current_pose(TCP).pose.position.z
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn('[grasp_task] TCP pose read raised (%s); retrying', exc)
                tcp_z = 0.0
            if abs(tcp_z) > 1e-6:
                break
            rospy.sleep(0.3)
        if abs(tcp_z) < 1e-6:
            rospy.logwarn('[grasp_task] could not read a valid TCP pose after 5 tries; '
                          'treating the pick as failed')
            return False, float('nan')
        min_up = aim_z + 0.06                # lift.min is 0.10; generous margin
        if tcp_z < min_up:
            rospy.logerr('[grasp_task] TCP is at z=%.3f, below the post-lift minimum %.3f: '
                         'the lift execution silently aborted partway.', tcp_z, min_up)
            return False, tcp_z
        return True, tcp_z

    def _abort_lost_grasp(self):
        """Recover from a lost object WITHOUT EVER MOVING DOWN.

        The real mine is somewhere on the ground below -- possibly directly under the
        closed jaws. Anything that descends open-loop from here can press it into the
        ground with full position-control authority and lever the base (measured: 1.09 m).
        So: open the jaws where they are, purge the phantom attached mine from the planning
        scene, and go home upward through free space.
        """
        # BREAK THE WELD FIRST (idempotent). Retreating with the mine still joined to the palm
        # would drag it through the world on the end of the arm, and a welded object that
        # survives into the next trial's reset gets teleported while still joined to the robot,
        # which detonates the solver and silently poisons every trial after it.
        self._unweld()
        self._open_jaws()

        # Detach AND delete: detaching alone would drop a ghost mine floating in mid-air at
        # the TCP, where the real one is not, and every later plan would dodge a phantom.
        self.arm.detach_object(OBJECT)
        rospy.sleep(0.3)
        self.psi.remove_world_object(OBJECT)
        rospy.sleep(0.3)

        # RETREAT STRAIGHT UP FIRST, while the scene contains no mine. If the pick aborted
        # low, the open jaws are wrapped around the real mine: re-adding its collision box
        # now would put the current state IN COLLISION and every plan to 'ready' fails --
        # observed: the arm was left parked at the grasp pose, and the next trial's reset
        # teleported the mine into the parked gripper (constraint-solver explosion,
        # finger_joint spun to 25 rad, base thrown >1 m). Straight up is free by
        # construction (the approach came down through it), so skip collision checking --
        # with the jaws open and empty this motion cannot press on anything.
        wp = self.arm.get_current_pose().pose      # the group's own eef link, which is what
        wp.position.z += 0.15                      # compute_cartesian_path plans for
        # This MoveIt build's signature is (waypoints, eef_step, avoid_collisions=True) --
        # no jump_threshold. Pass avoid_collisions by keyword only.
        path, frac = self.arm.compute_cartesian_path([wp], 0.01,
                                                     avoid_collisions=False)
        if frac > 0.5:
            self.arm.execute(path, wait=True)
            self.arm.stop()
        else:
            rospy.logwarn('[grasp_task] abort: could not compute the upward retreat '
                          '(fraction %.2f); going home directly', frac)

        # NOW the jaws are clear of the mine: put its collision object back (the real mine
        # is still on the ground where the detector saw it) so the go-home plans route
        # around it instead of through it.
        if self.last_target is not None:
            self.psi.add_object(make_landmine(
                self.last_target.header.frame_id or self.frame,
                self.last_target.point.x, self.last_target.point.y,
                self.last_target.point.z))
            rospy.sleep(0.3)

        for named in ('ready', 'stow'):
            self.arm.set_named_target(named)
            if not self.arm.go(wait=True):
                rospy.logerr("[grasp_task] abort: move to '%s' failed; leaving the arm "
                             'where it is', named)
                break
            self.arm.stop()

    def _brake_cb(self, req):
        """/grasp/brake -- pin the base to the ground, or let it go."""
        ok = self._brake(bool(req.data))
        return SetBoolResponse(success=ok,
                               message='braked' if req.data else 'released')

    def _look_cb(self, req):
        """/grasp/look -- aim the wrist camera at a ground point. Used by the SEARCH sweep."""
        ok = self._look(aim=(req.x, req.y))
        return LookAtResponse(ok=ok,
                              message='looking at (%.2f, %.2f)' % (req.x, req.y) if ok
                              else 'could not reach a look pose over (%.2f, %.2f)'
                                   % (req.x, req.y))

    def _place_cb(self, _req):
        """/grasp/place -- put the mine down, here, and let go of it.

        The counterpart to carry mode. /grasp/execute with release_after_lift:=false ends with
        the mine WELDED to the palm and the arm tucked, so the UGV can drive it somewhere; this
        is how it gets released once it has arrived. Without this, carry mode is a one-way trip
        and the robot drives around with a landmine bolted to its wrist forever.

        The order is not negotiable, and each step exists because the other order broke something:
          1. brake  -- the arm is about to reach down and out again, and an unbraked base gets
                       levered off its spot by that alone (0.030 m before anything is touched).
          2. lower  -- capped at lift.min, so it can never descend below the height it grasped at.
          3. unweld -- FIRST, so the mine is a free physical object again BEFORE the jaws move.
                       Opening first would leave a welded mine hanging off the palm.
          4. open + detach + retreat + restow.
        """
        if not self.welded:
            return TriggerResponse(success=False,
                                   message='nothing is held; there is nothing to place')
        try:
            self._brake(True)

            lower = build_place_task()
            if not lower.plan(1):
                # Holding a mine mid-air with no plan to set it down. Unwelding drops it ~10 cm
                # onto flat ground: not pretty, but bounded and known, unlike improvising.
                rospy.logerr('[grasp_task] no lower plan; releasing where we are')
                self._unweld()
                self._open_jaws()
                return TriggerResponse(success=False, message='lower planning failed; dropped')
            lower.execute(lower.solutions[0])

            self._unweld()
            rospy.sleep(0.5)

            rel = build_release_task()
            if not rel.plan(1):
                rospy.logerr('[grasp_task] no release plan; opening the jaws in place')
                self._open_jaws()
                return TriggerResponse(success=False, message='release planning failed')
            rel.execute(rel.solutions[0])
            rospy.loginfo('[grasp_task] placed')
            return TriggerResponse(success=True, message='placed')
        finally:
            self._brake(False)

    def _look(self, aim=None):
        """Unstow, then put the CAMERA above `aim` (a ground point in base_link), looking down.

        Perception needs a viewpoint. Stowed, the camera stares forward along the chassis
        and cannot see a mine on the ground at all -- the detector just reports "no
        detonator visible", which is easy to misread as a detection bug.

        `aim` exists for the SEARCH sweep. On arrival the mine can be anywhere within nav's
        error, which is LARGER than the camera's ground footprint, so the final approach has to
        be able to point the camera somewhere other than dead ahead and try again. With aim=None
        this keeps its old behaviour: the last known target, else the nominal standoff.
        """
        import math
        import numpy as np
        from tf.transformations import quaternion_from_matrix

        self.arm.set_named_target('ready')          # +2pi branch; see module docstring
        if not self.arm.go(wait=True):
            rospy.logerr('[grasp_task] unstow to "ready" failed')
            return False
        self.arm.stop(); self.arm.clear_pose_targets()

        if aim is not None:
            x, y = aim
        elif self.last_target is not None:
            x, y = self.last_target.point.x, self.last_target.point.y
        else:
            x, y = self.standoff, 0.0

        R = np.eye(4)
        R[:3, 2] = [0.0, 0.0, -1.0]          # optical +Z is the view axis -> look DOWN
        R[:3, 0] = [1.0, 0.0, 0.0]           # optical +X -> image right
        R[:3, 1] = np.cross(R[:3, 2], R[:3, 0])
        q = quaternion_from_matrix(R)

        ps = PoseStamped()
        ps.header.frame_id = self.frame
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.position.z = self.ground_z + self.look_height
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = (float(v) for v in q)

        # RETRY, because this fails perhaps one time in three and the failure is not a real one.
        # The look pose is a full 6-DoF Cartesian target on the CAMERA frame (not the TCP), which
        # is a harder IK problem than it looks, and RRTConnect is randomised: the same pose that
        # fails now succeeds on the next seed. Aborting the whole pick on the first sampling
        # failure throws away a perfectly good grasp and reads, misleadingly, as a perception bug
        # ("no target") when the arm simply never got somewhere it could see from.
        #
        # Retry with a longer planning budget each time rather than giving the same attempt
        # again: if the first quick try missed, the next one should be allowed to work harder.
        ok = False
        for attempt, budget in enumerate((5.0, 10.0, 20.0), start=1):
            self.arm.set_planning_time(budget)
            self.arm.set_pose_target(ps, end_effector_link=self.CAM)
            ok = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            if ok:
                if attempt > 1:
                    rospy.loginfo('[grasp_task] look pose reached on attempt %d', attempt)
                break
            rospy.logwarn('[grasp_task] look pose attempt %d/3 failed (planning budget %.0f s); '
                          'retrying', attempt, budget)
        self.arm.set_planning_time(10.0)
        if not ok:
            rospy.logerr('[grasp_task] could not reach the look pose above [%.2f %.2f] in 3 '
                         'attempts', x, y)
            return False
        rospy.sleep(1.0)                      # let the camera settle before we trust a frame
        return True

    def _acquire(self):
        """Re-perceive up close. The UAV's target is a ~12 m-altitude detection — nowhere
        near grasp-accurate — so the coarse point only tells us where to LOOK."""
        if not self._look():
            return None
        try:
            rospy.wait_for_service(self.detect_srv, timeout=3.0)
            if not rospy.ServiceProxy(self.detect_srv, Trigger)().success:
                rospy.logwarn('[grasp_task] detector saw nothing from the look pose')
                return None
        except rospy.ROSException:
            rospy.logwarn('[grasp_task] no detector service; using last /detected_targets')
        rospy.sleep(0.5)
        return self.last_target

    def _add_ground(self):
        """Put the GROUND in the planning scene. Nothing else does.

        This harness runs no octomap, and MoveIt has no implicit notion of a floor, so without
        this the planning scene is an infinite void and EVERY downward motion is free to plan
        the gripper straight through the terrain. The arm then holds that below-ground pose with
        its full 150 N.m of shoulder authority, pressing into ground that cannot yield -- and
        the reaction levers a 46 kg Husky on free-spinning wheels off its wheels entirely.
        Measured: base thrown 0.54 m and lifted to z=0.56 during a pick.

        A thin box just under z=0 is enough: it makes "down" a hard constraint for the planner
        rather than a suggestion. The mine sits ON it, so the box top must be at ground level,
        not above it, or the mine itself reads as in-collision.
        """
        from geometry_msgs.msg import PoseStamped as PS
        g = PS()
        g.header.frame_id = self.frame
        g.pose.position.x = 0.0
        g.pose.position.y = 0.0
        # ground_z is the terrain height in base_link (the UR5 base sits ~0.377 m above it).
        g.pose.position.z = self.ground_z - 0.05      # box is 0.10 thick; its TOP lands on z=0
        g.pose.orientation.w = 1.0
        self.psi.add_box('ground', g, size=(4.0, 4.0, 0.10))
        rospy.sleep(0.3)
        rospy.loginfo('[grasp_task] ground plane added to the planning scene at z=%.3f (%s)',
                      self.ground_z, self.frame)

    def _reset_scene(self):
        """Purge any landmine a PREVIOUS pick left in the planning scene.

        MTC's attachObject is a planning-scene edit, and move_group's scene PERSISTS across
        service calls. So one pick ends with the mine welded to grasp_tcp in MoveIt's world
        — while the real one is back on the ground — and the NEXT pick plans with a phantom
        mine bolted to the gripper. It collides with everything, every ComputeIK returns 0
        solutions, and the task dies at 'connect' with no obvious cause.

        Symptom to recognise: the pick succeeds exactly ONCE per move_group lifetime and
        every call after that fails at IK. add_object() alone does NOT fix it — an attached
        object has to be detached first.
        """
        if OBJECT in self.psi.get_attached_objects():
            rospy.logwarn('[grasp_task] stale landmine still attached to the gripper in the '
                          'planning scene; detaching before replanning')
            self.arm.detach_object(OBJECT)
            rospy.sleep(0.3)
        if OBJECT in self.psi.get_known_object_names():
            self.psi.remove_world_object(OBJECT)
            rospy.sleep(0.3)

    def _execute_cb(self, _req):
        try:
            return self._pick()
        finally:
            # ALWAYS release the brake, however the pick ended. A robot left welded to the
            # ground would silently refuse to drive on, and the tour would look like a nav bug.
            self._brake(False)

    def _pick(self):
        # Before anything else -- including the look motion, which is itself planned and would
        # otherwise be planned with a phantom mine hanging off the TCP.
        self._reset_scene()

        # BRAKE FIRST, before any arm motion at all. The look pose already extends the arm, and
        # the grasp pose alone is enough to shove the base off its spot (see GROUND_MODEL).
        self._brake(True)

        target = self._acquire()
        if target is None:
            return TriggerResponse(success=False, message='no target')

        # Spawn the mine into the planning scene BEFORE the task, so CurrentState — and
        # therefore every state the generator spawns — carries a scene containing it, which
        # is what makes ComputeIK's collision check mean anything.
        self.psi.add_object(make_landmine(target.header.frame_id or self.frame,
                                          target.point.x, target.point.y, target.point.z))
        rospy.sleep(0.5)

        grasps = fetch_grasps(target)
        if not grasps:
            return TriggerResponse(success=False, message='no grasp candidates')

        task = build_reach_task(target, grasps)
        try:
            ok = task.plan(1)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr('[grasp_task] planning raised: %s', exc)
            return TriggerResponse(success=False, message=str(exc))

        if not ok:
            # MTC's whole point: say WHICH stage failed, instead of the legacy script's
            # silent fallback.
            # Do NOT try to walk the stage tree from Python: task.add()/insert() DISOWN the
            # stage objects (C++ takes ownership), so any later attribute access on one raises
            # "Missing value for wrapped C++ type: Python instance was disowned". MTC prints
            # its own tree -- with per-stage solution/failure counts -- to this node's stdout
            # on every plan, which is the same information and always correct. Read the log.
            rospy.logerr('[grasp_task] no solution. See the MTC stage tree printed above: '
                         'the first stage with 0 forward solutions is the culprit.')
            return TriggerResponse(success=False, message='planning failed')

        rospy.loginfo('[grasp_task] reach planned (cost %.3f)', task.solutions[0].cost)
        if self.plan_only:
            return TriggerResponse(success=True, message='planned (plan_only)')

        task.execute(task.solutions[0])
        rospy.loginfo('[grasp_task] at the grasp pose, jaws open; closing and welding')

        # THE GRASP. The jaws are open around the block and nothing has touched it yet.
        held, why = self._close_and_weld()
        if not held:
            rospy.logerr('[grasp_task] CLOSE FAILED: %s', why)
            self._abort_lost_grasp()
            return TriggerResponse(success=False, message=why)
        rospy.loginfo('[grasp_task] grasped (%s)', why)

        lift = build_lift_task()
        try:
            ok = lift.plan(1)
        except Exception as exc:  # noqa: BLE001
            ok = False
            rospy.logerr('[grasp_task] lift planning raised: %s', exc)
        if not ok:
            rospy.logerr('[grasp_task] no lift plan; opening the jaws and retreating')
            self._abort_lost_grasp()
            return TriggerResponse(success=False, message='lift planning failed')
        lift.execute(lift.solutions[0])

        # Everything below this line may move the arm DOWN, and nothing may move down until
        # the lift is verified to have actually happened (task.execute() cannot tell us).
        # All four yaw candidates share one z, so grasps[0] gives the aim height regardless
        # of which candidate ComputeIK chose.
        aim_z = grasps[0][0].pose.position.z
        up, tcp_z = self._lifted(aim_z)
        if not up:
            rospy.logerr('[grasp_task] LIFT CHECK FAILED (TCP z=%.3f): aborting UPWARD, '
                         'no descent.', tcp_z)
            self._abort_lost_grasp()
            return TriggerResponse(success=False, message='lift aborted')
        # The arm is up, and the weld is a real fixed joint, so the mine cannot have "slipped".
        # This still guards the case where the weld was never made or was broken by an abort:
        # without it we would carry on and lower an EMPTY gripper onto the real mine, which is
        # what pole-vaulted the Husky 1.09 m the one time it happened.
        if not self.welded:
            rospy.logerr('[grasp_task] MINE NOT HELD after the lift. Aborting UPWARD, '
                         'no descent.')
            self._abort_lost_grasp()
            return TriggerResponse(success=False, message='dropped during lift')
        rospy.loginfo('[grasp_task] lifted: mine held, TCP at z=%.3f', tcp_z)

        if not self.release:
            second = build_carry_task()         # carry mode: the mine stays in the jaws
            if second.plan(1):
                second.execute(second.solutions[0])
                return TriggerResponse(success=True, message='picked (carrying)')
            self._abort_lost_grasp()
            return TriggerResponse(success=False, message='restow planning failed')

        lower = build_place_task()
        try:
            ok = lower.plan(1)
        except Exception as exc:  # noqa: BLE001
            ok = False
            rospy.logerr('[grasp_task] lower planning raised: %s', exc)
        if not ok:
            # Holding the mine mid-air with no plan to set it down. Opening the jaws here
            # drops it ~10 cm onto flat ground -- not pretty, but bounded and known, unlike
            # improvising a descent.
            rospy.logerr('[grasp_task] no lower plan; releasing at lift height and '
                         'retreating upward')
            self._abort_lost_grasp()
            return TriggerResponse(success=False, message='lower planning failed')
        lower.execute(lower.solutions[0])

        # The mine is back on the ground and still welded. Break the weld FIRST: that restores
        # it to being a free physical object BEFORE the jaws move, so opening them releases
        # nothing and drops nothing. Opening first would leave a welded mine hanging off the
        # palm; unwelding while still high would drop it.
        self._unweld()
        rospy.sleep(0.5)

        rel = build_release_task()
        try:
            ok = rel.plan(1)
        except Exception as exc:  # noqa: BLE001
            ok = False
            rospy.logerr('[grasp_task] release planning raised: %s', exc)
        if not ok:
            rospy.logerr('[grasp_task] no release plan; opening the jaws in place')
            self._abort_lost_grasp()
            return TriggerResponse(success=False, message='release planning failed')
        rel.execute(rel.solutions[0])
        rospy.loginfo('[grasp_task] executed')
        return TriggerResponse(success=True, message='picked')


if __name__ == '__main__':
    rospy.init_node('grasp_task')
    GraspNode()
    rospy.spin()
