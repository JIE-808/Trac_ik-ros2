/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#ifndef TRAC_IK_KINEMATICS_PLUGIN_HPP_
#define TRAC_IK_KINEMATICS_PLUGIN_HPP_

#include <moveit/kinematics_base/kinematics_base.h>
#include <kdl/chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace trac_ik_kinematics_plugin
{

class TRAC_IKKinematicsPlugin : public kinematics::KinematicsBase
{
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  unsigned int num_joints_;
  bool active_;

  KDL::Chain chain;
  bool position_ik_;

  KDL::JntArray joint_min, joint_max;

  std::string solve_type;

public:
  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  }
  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names_;
  }

  TRAC_IKKinematicsPlugin() : num_joints_(0), active_(false), position_ik_(false) {}

  ~TRAC_IKKinematicsPlugin() override = default;

  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                     const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution,
                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options =
                         kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string>& link_names,
                     const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const moveit::core::RobotModel& robot_model,
                  const std::string& group_name,
                  const std::string& base_frame,
                  const std::vector<std::string>& tip_frames,
                  double search_discretization) override;

private:
  int getKDLSegmentIndex(const std::string& name) const;
};

}  // namespace trac_ik_kinematics_plugin

#endif  // TRAC_IK_KINEMATICS_PLUGIN_HPP_
