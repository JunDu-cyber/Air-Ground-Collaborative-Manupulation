/*********************************************************************
 * Software License Agreement (BSD License)
 *  Copyright (c) 2014, Konstantinos Chatzilygeroudis
 *  Copyright (c) 2016, CRI Lab at Nanyang Technological University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <robotiq_gazebo/mimic_joint_plugin.h>

namespace gazebo
{

MimicJointPlugin::MimicJointPlugin()
{
  kill_sim = false;

  joint_.reset();
  mimic_joint_.reset();
}

MimicJointPlugin::~MimicJointPlugin()
{
  this->updateConnection.reset();

  kill_sim = true;
}

void MimicJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  ros::NodeHandle model_nh;
  model_ = _parent;
  world_ = model_->GetWorld();

  // Error message if the model couldn't be found
  if (!model_)
  {
    ROS_ERROR("Parent model is NULL! MimicJointPlugin could not be loaded.");
    return;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_ERROR("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  // Check for robot namespace
  robot_namespace_ = "/";
  if(_sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  // Check for joint element
  if(!_sdf->HasElement("joint"))
  {
    ROS_ERROR("No joint element present. MimicJointPlugin could not be loaded.");
    return;
  }
  
  joint_name_ = _sdf->GetElement("joint")->Get<std::string>();

  // Check for mimicJoint element
  if(!_sdf->HasElement("mimicJoint"))
  {
    ROS_ERROR("No mimicJoint element present. MimicJointPlugin could not be loaded.");
    return;
  }
  
  mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();

  has_pid_ = false;
  // Check if PID controller wanted
  if(_sdf->HasElement("hasPID"))
  {
    has_pid_ = true;

    // Gains come from SDF elements <p>/<i>/<d>, NOT from the ROS parameter server.
    //
    // Upstream read them from /gazebo_ros_control/pid_gains/<mimicJoint>. That namespace is
    // unusable here: on this Noetic + gazebo_ros_control build, the mere PRESENCE of any
    // /gazebo_ros_control/pid_gains/* parameter makes gzserver abort at startup with
    // std::bad_array_new_template ("std::bad_array_new_length"), whatever joint it names.
    // Verified by bisection: with those params gzserver dies every time; without them it
    // starts every time. gazebo_ros_control probes that namespace for every position-mode
    // joint it owns, so we cannot populate it just for the mimics without tripping it.
    //
    // Reading the gains straight from the SDF keeps them next to the joint they tune and
    // sidesteps the whole thing. Defaults match the tuned values in the 2F-140 macro.
    double p = 20.0, i = 0.5, d = 1.0;
    if(_sdf->HasElement("p")) p = _sdf->GetElement("p")->Get<double>();
    if(_sdf->HasElement("i")) i = _sdf->GetElement("i")->Get<double>();
    if(_sdf->HasElement("d")) d = _sdf->GetElement("d")->Get<double>();

    pid_ = control_toolbox::Pid(p,i,d);
    ROS_INFO("MimicJointPlugin: %s follows %s with PID (%.2f, %.2f, %.2f), maxEffort clamp applied",
             mimic_joint_name_.c_str(), joint_name_.c_str(), p, i, d);
  }

  // Check for multiplier element
  multiplier_ = 1.0;
  if(_sdf->HasElement("multiplier"))
    multiplier_ = _sdf->GetElement("multiplier")->Get<double>();

  // Check for offset element
  offset_ = 0.0;
  if (_sdf->HasElement("offset"))
    offset_ = _sdf->GetElement("offset")->Get<double>();

  // Check for sensitiveness element
  sensitiveness_ = 0.0;
  if (_sdf->HasElement("sensitiveness"))
    sensitiveness_ = _sdf->GetElement("sensitiveness")->Get<double>();

  // Check for max effort
  max_effort_ = 1.0;
  if (_sdf->HasElement("maxEffort"))
  {
    max_effort_ = _sdf->GetElement("maxEffort")->Get<double>();
  }

  // Get pointers to joints
  joint_ = model_->GetJoint(joint_name_);
  if(!joint_)
  {
    ROS_ERROR("No joint named %s. MimicJointPlugin could not be loaded.", joint_name_.c_str());
    return;
  }
  mimic_joint_ = model_->GetJoint(mimic_joint_name_);
  if(!mimic_joint_)
  {
    ROS_ERROR("No (mimic) joint named %s. MimicJointPlugin could not be loaded.", mimic_joint_name_.c_str());
    return;
  }
  
  // Set max effort
  if(!has_pid_)
  {
    #if GAZEBO_MAJOR_VERSION > 2
      mimic_joint_->SetParam("fmax", 0, max_effort_);
    #else
      mimic_joint_->SetMaxForce(0, max_effort_);
    #endif
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MimicJointPlugin::UpdateChild, this));
}

void MimicJointPlugin::UpdateChild()
{
  static ros::Duration period(world_->Physics()->GetMaxStepSize());

  // Set mimic joint's angle based on joint's angle
  double angle = joint_->Position(0)*multiplier_+offset_;
  
  if(abs(angle-mimic_joint_->Position(0))>=sensitiveness_)
  {
    if(has_pid_)
    {
      double a = mimic_joint_->Position(0);
      if(a!=a)
        a = angle;
      double error = angle-a;
      double effort = ignition::math::clamp(pid_.computeCommand(error, period), -max_effort_, max_effort_);
      // UPSTREAM BUG: this fork computed `effort` and then threw it away -- there was no
      // SetForce() call here at all, so <hasPID/> mode left the mimic joints completely
      // unactuated and only the SetPosition (teleport) branch below ever did anything.
      //
      // That matters because SetPosition is a KINEMATIC override: the mimic joints, which
      // carry the finger pads, get forced to their commanded angle regardless of contact.
      // They exert no force on a grasped object, and once finger_joint is moved to an
      // EffortJointInterface (so the master IS dynamic) the half-kinematic/half-dynamic
      // linkage fights itself and the joint runs away past its limit.
      mimic_joint_->SetForce(0, effort);
    }
    else
    {
      #if GAZEBO_MAJOR_VERSION >= 4
        mimic_joint_->SetPosition(0, angle);
      #else
        mimic_joint_->SetAngle(0, angle);
      #endif
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin);

}
