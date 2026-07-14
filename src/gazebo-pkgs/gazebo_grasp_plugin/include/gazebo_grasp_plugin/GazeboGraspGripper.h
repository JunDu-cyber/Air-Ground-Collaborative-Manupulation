#ifndef GAZEBO_GAZEBOGRASPGRIPPER_H
#define GAZEBO_GAZEBOGRASPGRIPPER_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_version_helpers/GazeboVersionHelpers.h>
#include <stdio.h>

namespace gazebo
{

/**
 * \brief Helper class for GazeboGraspFix which holds information for one arm.
 * Attaches /detaches objects to the palm of this arm.
 *
 * \author Jennifer Buehler
 */
class GazeboGraspGripper
{
  public:
    GazeboGraspGripper();
    GazeboGraspGripper(const GazeboGraspGripper &o);
    virtual ~GazeboGraspGripper();

    /**
     *
     * \param disableCollisionsOnAttach when an object is attached, collisions with it will be disabled. This is useful
     *      if the robot then still keeps wobbling.
     */
    bool Init(physics::ModelPtr &_model,
              const std::string &_gripperName,
              const std::string &palmLinkName,
              const std::vector<std::string> &fingerLinkNames,
              bool _disableCollisionsOnAttach,
              std::map<std::string, physics::CollisionPtr> &_collisions);

    const std::string &getGripperName() const;

    /**
     * Has the link name (URDF)
     */
    bool hasLink(const std::string &linkName) const;

    /**
     * Has the collision link name (Gazebo collision element name)
     */
    bool hasCollisionLink(const std::string &linkName) const;

    bool isObjectAttached() const;

    const std::string &attachedObject() const;

    /**
     * \param gripContacts contact forces on the object sorted by the link name colliding.
     */
    bool HandleAttach(const std::string &objName);
    /**
     * Simulator-only transport lock.  The object remains visible in Gazebo
     * and follows the palm exactly, but it is kinematic and therefore cannot
     * transmit terrain/contact impulses back into the mobile base.
     */
    bool HandleKinematicAttach(const std::string &objName);
    void UpdateKinematicAttachment();
    bool isKinematicAttachment() const;
    void HandleDetach(const std::string &objName);

  private:

    physics::ModelPtr model;

    // name of the gripper
    std::string gripperName;

    // names of the gripper links
    std::vector<std::string> linkNames;
    // names and Collision objects of the collision links in Gazebo (scoped names)
    // Not necessarily equal names and size to linkNames.
    std::map<std::string, physics::CollisionPtr> collisionElems;

    physics::JointPtr fixedJoint;

    physics::LinkPtr palmLink;

    // when an object is attached, collisions with it may be disabled, in case the
    // robot still keeps wobbling.
    bool disableCollisionsOnAttach;

    // A forced mission lock is represented as a visible, pose-following
    // Gazebo model instead of a physical closed-chain joint.  This guarantees
    // retention without allowing a mine which still touches the terrain to
    // lever the complete UGV around the wrist joint.
    bool kinematicAttachment;
    physics::LinkPtr kinematicObjectLink;
    physics::ModelPtr kinematicObjectModel;
    GzPose3 kinematicRelativeModelPose;
    bool kinematicOriginalGravityMode;
    bool kinematicOriginalMode;

    // flag holding whether an object is attached. Object name in \e attachedObjName
    bool attached;
    // name of the object currently attached.
    std::string attachedObjName;
};

}
#endif  // GAZEBO_GAZEBOGRASPGRIPPER_H
