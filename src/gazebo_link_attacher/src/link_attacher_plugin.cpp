/* Carry a Gazebo model rigidly with a robot link, on request ("weld" / "release").
 *
 * WHY THIS EXISTS
 * ---------------
 * The 2F-140 in this workspace cannot exert grip force in Gazebo. Every mimic joint is driven
 * by robotiq_gazebo's MimicJointPlugin with SetPosition() -- a KINEMATIC teleport -- and
 * right_outer_knuckle_joint is itself a mimic, so the whole right finger is kinematic.
 * gazebo_ros_control drives finger_joint in position mode, which is SetPosition() as well.
 * A kinematically-forced finger does not push on what it touches: it is driven to its
 * commanded angle regardless of contact, and simply penetrates the object.
 *
 * Measured consequences, all of which this plugin removes:
 *   1. gazebo_grasp_fix NEVER fires. It attaches only on two OPPOSING CONTACT FORCES, and a
 *      kinematic finger produces none. (Same failure is reported upstream for Robotiq 2F
 *      grippers: JenniferBuehler/gazebo-pkgs issues #9 and #52.)
 *   2. finger_joint reached its commanded 0.580 with a 40 mm block between the pads. A real
 *      grip must STALL short. What used to look like a "friction grasp" was the two pads
 *      interpenetrating the block symmetrically so the lateral solver forces cancelled -- a
 *      coin flip, not a grasp.
 *   3. The penetration impulses threw the 46 kg Husky: finger_joint spun to +/-18 rad and
 *      base_link was launched to z=0.96 m. THAT is "the roll" -- a contact-solver artefact,
 *      not a centre-of-mass effect.
 *
 * HOW THE WELD IS BUILT (and the two ways it was built wrong first)
 * -----------------------------------------------------------------
 * A revolute joint with zero limits, exactly as gazebo_grasp_plugin's own GazeboGraspGripper
 * builds it. The ONE thing that matters is the third argument to Joint::Load:
 *
 *     diff = object_link->WorldPose() - holder_link->WorldPose();   // the CURRENT offset
 *     joint->Load(holder_link, object_link, diff);
 *
 * Passing an identity pose there instead -- which is what gazebo_ros_link_attacher's README
 * example looks like it does -- makes the joint try to drag the object's origin onto the
 * holder's origin. Measured: the mine snapped upward the instant the joint was created, ended
 * up 0.13 m above the TCP it was grasped at, and the solver dragged the 46 kg robot with it
 * (base thrown 0.82 m). The constraint became a second source of the violence it was meant
 * to end.
 *
 * The other dead end was to skip joints and re-impose the grasp pose with SetWorldPose() from
 * a WorldUpdateBegin callback. That DEADLOCKS gzserver: SetWorldPose takes the world pose
 * mutex, which is not reentrant from inside the update loop, and the simulation simply stops
 * stepping (/clock goes silent, everything downstream hangs waiting for a robot state).
 */
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "gazebo_link_attacher/Attach.h"

namespace gazebo
{
class LinkAttacher : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr /*sdf*/) override
  {
    world_ = world;

    if (!ros::isInitialized())
    {
      int argc = 0;
      ros::init(argc, nullptr, "gazebo_link_attacher", ros::init_options::NoSigintHandler);
    }
    nh_.reset(new ros::NodeHandle("link_attacher"));

    ros::AdvertiseServiceOptions attach_opts =
        ros::AdvertiseServiceOptions::create<gazebo_link_attacher::Attach>(
            "attach", boost::bind(&LinkAttacher::Attach, this, _1, _2), ros::VoidPtr(), &queue_);
    ros::AdvertiseServiceOptions detach_opts =
        ros::AdvertiseServiceOptions::create<gazebo_link_attacher::Attach>(
            "detach", boost::bind(&LinkAttacher::Detach, this, _1, _2), ros::VoidPtr(), &queue_);
    attach_srv_ = nh_->advertiseService(attach_opts);
    detach_srv_ = nh_->advertiseService(detach_opts);

    queue_thread_ = std::thread(&LinkAttacher::QueueThread, this);
    ROS_INFO("link_attacher: ready (/link_attacher/attach, /link_attacher/detach)");
  }

  ~LinkAttacher() override
  {
    queue_.clear();
    queue_.disable();
    nh_->shutdown();
    if (queue_thread_.joinable())
      queue_thread_.join();
  }

private:
  struct Held
  {
    std::string m1, l1, m2, l2;
    physics::JointPtr joint;
  };

  bool Attach(gazebo_link_attacher::Attach::Request& req,
              gazebo_link_attacher::Attach::Response& res)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (Find(req) != held_.end())
    {
      res.ok = true;
      res.message = "already held";
      return true;
    }

    physics::ModelPtr m1 = world_->ModelByName(req.model_name_1);
    physics::ModelPtr m2 = world_->ModelByName(req.model_name_2);
    if (!m1 || !m2)
    {
      res.ok = false;
      res.message = "no such model: " + (m1 ? req.model_name_2 : req.model_name_1);
      return true;
    }
    physics::LinkPtr holder = m1->GetLink(req.link_name_1);
    physics::LinkPtr obj = m2->GetLink(req.link_name_2);
    if (!holder || !obj)
    {
      // NB fixed-joint children are LUMPED into their parent by Gazebo, so e.g.
      // robotiq_arg2f_base_link does not exist as a link -- ur5_wrist_3_link is that body.
      res.ok = false;
      res.message = "no such link: " + (holder ? req.link_name_2 : req.link_name_1);
      return true;
    }

    Held h;
    h.m1 = req.model_name_1; h.l1 = req.link_name_1;
    h.m2 = req.model_name_2; h.l2 = req.link_name_2;

    // THE grasp. `diff` is the whole trick: it is the object's pose RELATIVE TO THE HOLDER,
    // right now, so the joint locks the object exactly where it already is. Pass identity
    // here and the joint instead hauls the object's origin onto the holder's origin, taking
    // the robot with it.
    const ignition::math::Pose3d diff = obj->WorldPose() - holder->WorldPose();

    // Every argument here was learned the hard way; none of them is decoration.
    //
    // * CreateJoint takes the HOLDER'S MODEL. Created without one, the joint anchors the
    //   object to the WORLD instead: measured, the mine froze in mid-air at z=0.078 and the
    //   arm then winched the whole Husky up to it (base thrown 1.29 m, base_z 0.13 -> 0.59).
    // * `diff` is the object's CURRENT pose relative to the holder, which locks it where it
    //   already is. Identity instead makes the joint haul the object's origin onto the
    //   holder's origin -- the mine snapped upward on contact and the solver threw the robot.
    // * "fixed", not gazebo_grasp_plugin's zero-limit "revolute": that plugin predates the
    //   fixed joint, and on a runtime-created joint the limits silently fail to bind, leaving
    //   the object swinging on a free hinge (measured 80 mm of carry error).
    // * THE JOINT MUST BE OWNED BY A MODEL THAT IS NOT STATIC, and that is NOT always model_1.
    //
    //   The two things get conflated easily, because usually model_1 is both: the parent link of
    //   the joint, AND the model that owns it. For a gripper holding an object they are the same
    //   and everything works. For pinning the ROBOT TO THE GROUND they are not:
    //
    //     parent MUST be the ground  -- a static link cannot be a joint's CHILD.
    //     owner  MUST be the robot   -- a joint owned by a STATIC model can never be undone.
    //
    //   Own it with the static model and the weld is permanent: Detach() returns cleanly, the
    //   caller is told "released", and the robot stays bolted to the world forever. Measured, and
    //   it is vicious, because nothing anywhere reports an error: a fresh sim drives 0.509 m on a
    //   cmd_vel; the same sim after ONE brake cycle drives 0.000 m -- even when the command is
    //   published straight to husky_velocity_controller, bypassing every mux. It presents as a
    //   dead velocity controller and it is nothing of the kind.
    //
    //   So: keep model_1's link as the parent, and give the joint to whichever model can actually
    //   own it.
    physics::ModelPtr owner = m1->IsStatic() ? m2 : m1;
    if (m1->IsStatic() && m2->IsStatic())
    {
      res.ok = false;
      res.message = "both models are static; a joint between them could never be released";
      return true;
    }

    h.joint = world_->Physics()->CreateJoint("fixed", owner);
    h.joint->Attach(holder, obj);
    h.joint->Load(holder, obj, diff);
    h.joint->SetModel(owner);
    h.joint->Init();

    held_.push_back(h);
    res.ok = true;
    res.message = "held " + req.model_name_2;
    ROS_INFO_STREAM("link_attacher: welded " << req.model_name_2 << " to "
                                             << req.model_name_1 << "::" << req.link_name_1
                                             << " at offset " << diff.Pos());
    return true;
  }

  bool Detach(gazebo_link_attacher::Attach::Request& req,
              gazebo_link_attacher::Attach::Response& res)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = Find(req);
    if (it == held_.end())
    {
      res.ok = true;                 // idempotent on purpose: a release must never throw
      res.message = "not held";
      return true;
    }
    // Detach(), THEN TEAR THE JOINT DOWN. Detach() alone is not enough when the parent is the
    // static world: the ODE constraint survives it, and the "released" child stays welded in
    // place. Measured with a parking brake pinning the Husky to the terrain -- brake off, service
    // says "released", and the robot still drives 0.000 m on a direct velocity command, forever.
    //
    // Fini() is what actually dismantles the constraint. It must come AFTER Detach(): tearing
    // down a joint that is still attached to two live bodies mid-step is a segfault.
    //
    // The husk is still parked rather than freed. Gazebo has no safe runtime joint DELETION, and
    // dropping the last reference to one is its own crash; an inert JointPtr costs a pointer.
    it->joint->Detach();
    it->joint->Fini();
    graveyard_.push_back(it->joint);
    held_.erase(it);
    res.ok = true;
    res.message = "released";
    ROS_INFO_STREAM("link_attacher: released " << req.model_name_2);
    return true;
  }

  std::vector<Held>::iterator Find(const gazebo_link_attacher::Attach::Request& req)
  {
    for (auto it = held_.begin(); it != held_.end(); ++it)
      if (it->m1 == req.model_name_1 && it->l1 == req.link_name_1 &&
          it->m2 == req.model_name_2 && it->l2 == req.link_name_2)
        return it;
    return held_.end();
  }

  void QueueThread()
  {
    ros::Rate r(50);
    while (nh_->ok())
    {
      queue_.callAvailable(ros::WallDuration(0.0));
      r.sleep();
    }
  }

  physics::WorldPtr world_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  std::thread queue_thread_;
  ros::ServiceServer attach_srv_, detach_srv_;
  std::vector<Held> held_;
  std::vector<physics::JointPtr> graveyard_;   // detached joints, never freed (see Detach)
  std::mutex mtx_;
};

GZ_REGISTER_WORLD_PLUGIN(LinkAttacher)
}  // namespace gazebo
