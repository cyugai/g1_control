/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "g1_dummy/visualization/g1Visualizer.h"

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include "g1_interface/gait/MotionPhaseDefinition.h"

// Additional messages not in the helpers file
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace ocs2 {
namespace g1 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
g1Visualizer::g1Visualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                             const PinocchioEndEffectorKinematics& endEffectorKinematics, const rclcpp::Node::SharedPtr &node,
                                             scalar_t maxUpdateFrequency)
    : pinocchioInterface_(std::move(pinocchioInterface)),//pinocchioInterface 参数转换为右值引用
      centroidalModelInfo_(std::move(centroidalModelInfo)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency), 
      tfBroadcaster_(node) {
      clock_ = node->get_clock();
      //使末端执行器运动学计算能够使用最新的机器人模型状态
      endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
      launchVisualizerNode(node);
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void g1Visualizer::launchVisualizerNode(const rclcpp::Node::SharedPtr &node) {
  costDesiredBasePositionPublisher_ = node->create_publisher<visualization_msgs::msg::Marker>("/g1/desiredBaseTrajectory", 1);
  costDesiredFeetPositionPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
  costDesiredFeetPositionPublishers_[0] = node->create_publisher<visualization_msgs::msg::Marker>("/g1/desiredFeetTrajectory/LTOE", 1);
  costDesiredFeetPositionPublishers_[1] = node->create_publisher<visualization_msgs::msg::Marker>("/g1/desiredFeetTrajectory/LHEEL", 1);
  costDesiredFeetPositionPublishers_[2] = node->create_publisher<visualization_msgs::msg::Marker>("/g1/desiredFeetTrajectory/RTOE", 1);
  costDesiredFeetPositionPublishers_[3] = node->create_publisher<visualization_msgs::msg::Marker>("/g1/desiredFeetTrajectory/RHEEL", 1);
  stateOptimizedPublisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/g1/optimizedStateTrajectory", 1);
  currentStatePublisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/g1/currentState", 1);
  jointPublisher_ = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);



  
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void g1Visualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto timeStamp = clock_->now();
    publishObservation(timeStamp, observation);
    publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    lastTime_ = observation.time;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void g1Visualizer::publishObservation(const rclcpp::Time& timeStamp, const SystemObservation& observation) {
  // Extract components from state
  const auto basePose = centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
  const auto qJoints = centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

  // Compute cartesian state and inputs
  const auto feetPositions = endEffectorKinematicsPtr_->getPosition(observation.state);
  std::vector<vector3_t> feetForces(centroidalModelInfo_.numThreeDofContacts);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    feetForces[i] = centroidal_model::getContactForces(observation.input, i, centroidalModelInfo_);
  }

  // Publish
  publishJointTransforms(timeStamp, qJoints);
  publishBaseTransform(timeStamp, basePose);
  publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode), feetPositions, feetForces);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void g1Visualizer::publishJointTransforms(const rclcpp::Time& timeStamp, const vector_t& jointAngles) const {
  if (jointPublisher_ != nullptr) {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = clock_->now();
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    joint_state.name[0] = "left_hip_pitch_joint";
    joint_state.name[1] = "left_hip_roll_joint";
    joint_state.name[2] = "left_hip_yaw_joint";
    joint_state.name[3] = "left_knee_joint";
    joint_state.name[4] = "left_ankle_pitch_joint";
    joint_state.name[5] = "left_ankle_roll_joint";
    joint_state.name[6] = "right_hip_pitch_joint";
    joint_state.name[7] = "right_hip_roll_joint";
    joint_state.name[8] = "right_hip_yaw_joint";
    joint_state.name[9] = "right_knee_joint";
    joint_state.name[10] = "right_ankle_pitch_joint";
    joint_state.name[11] = "right_ankle_roll_joint";
    joint_state.position[0] = jointAngles[0];
    joint_state.position[1] = jointAngles[1];
    joint_state.position[2] = jointAngles[2];
    joint_state.position[3] = jointAngles[3];
    joint_state.position[4] = jointAngles[4];
    joint_state.position[5] = jointAngles[5];
    joint_state.position[6] = jointAngles[6];
    joint_state.position[7] = jointAngles[7];
    joint_state.position[8] = jointAngles[8];
    joint_state.position[9] = jointAngles[9];
    joint_state.position[10] = jointAngles[10];
    joint_state.position[11] = jointAngles[11];

    jointPublisher_->publish(joint_state);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void g1Visualizer::publishBaseTransform(const rclcpp::Time& timeStamp, const vector_t& basePose) {
  
    //geometry_msgs::msg::TransformStamped 是 ROS2 中用于表示带有时间戳的坐标变换的消息类型
    //child_frame_id: 子坐标系的ID
    //header: 消息头，包含时间戳和坐标系ID
    //transform: 变换信息，包含旋转和位移
    //translation 是一个 geometry_msgs::msg::Vector3 类型，有 x, y, z 三个分量
    //rotation 是一个 geometry_msgs::msg::Quaternion 类型，有 x, y, z, w 四个分量
    geometry_msgs::msg::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    baseToWorldTransform.child_frame_id = "dummy_link";

    const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
    baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
    baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
    tfBroadcaster_.sendTransform(baseToWorldTransform);
  
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//按照指定的速度发布一系列系统观测数据，用于可视化机器人的轨迹
//使得轨迹可视化的速度可以通过 speed 参数进行控制，比如加快或减慢可视化过程
//system_observation_array 中的元素是按时间顺序排列的，代表了机器人轨迹的时间序列
void g1Visualizer::publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed) {
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    scalar_t frameDuration = speed * (system_observation_array[k + 1].time - system_observation_array[k].time);
    //timedExecutionInSeconds 函数：这是一个工具函数，它接受一个可调用对象（这里是 lambda 表达式），执行这个对象，并返回执行所需的时间（以秒为单位）
    scalar_t publishDuration = timedExecutionInSeconds([&]() { publishObservation(clock_->now(), system_observation_array[k]); });
    if (frameDuration > publishDuration) {
      const rclcpp::Duration duration = rclcpp::Duration::from_seconds(frameDuration - publishDuration);
      rclcpp::sleep_for((std::chrono::nanoseconds(duration.nanoseconds())));
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void g1Visualizer::publishCartesianMarkers(const rclcpp::Time& timeStamp, const contact_flag_t& contactFlags,
                                                    const std::vector<vector3_t>& feetPositions,
                                                    const std::vector<vector3_t>& feetForces) const {
  // Reserve message
  const size_t numberOfCartesianMarkers = 10;
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i) {
    markerArray.markers.emplace_back(
        getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
    markerArray.markers.emplace_back(getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green, forceScale_));
  }

  // Center of pressure
  markerArray.markers.emplace_back(getCenterOfPressureMarker(feetForces.begin(), feetForces.end(), feetPositions.begin(),
                                                             contactFlags.begin(), Color::green, copMarkerDiameter_));

  // Support polygon
  markerArray.markers.emplace_back(
      getSupportPolygonMarker(feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_->publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void g1Visualizer::publishDesiredTrajectory(const rclcpp::Time& timeStamp, const TargetTrajectories& targetTrajectories) {
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;

  // Reserve com messages
  std::vector<geometry_msgs::msg::Point> desiredBasePositionMsg;
  desiredBasePositionMsg.reserve(stateTrajectory.size());

  // Reserve feet messages
  feet_array_t<std::vector<geometry_msgs::msg::Point>> desiredFeetPositionMsgs;
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
  }

  for (size_t j = 0; j < stateTrajectory.size(); j++) {
    const auto state = stateTrajectory.at(j);
    vector_t input(centroidalModelInfo_.inputDim);
    if (j < inputTrajectory.size()) {
      input = inputTrajectory.at(j);
    } else {
      input.setZero();
    }

    // Construct base pose msg
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
    geometry_msgs::msg::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());

    // Fill message containers
    desiredBasePositionMsg.push_back(pose.position);

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
      geometry_msgs::msg::Pose footPose;
      footPose.position = getPointMsg(feetPositions[i]);
      desiredFeetPositionMsgs[i].push_back(footPose.position);
    }
  }

  // Headers
  auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_);
  comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
  comLineMsg.id = 0;

  // Publish
  costDesiredBasePositionPublisher_->publish(comLineMsg);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
    footLineMsg.header = getHeaderMsg(frameId_, timeStamp);
    footLineMsg.id = 0;
    costDesiredFeetPositionPublishers_[i]->publish(footLineMsg);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void g1Visualizer::publishOptimizedStateTrajectory(const rclcpp::Time& timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                                            const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule) {
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
    return;  // Nothing to publish
  }

  // Reserve Feet msg
  feet_array_t<std::vector<geometry_msgs::msg::Point> > feetMsgs;
  std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::msg::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

  // Reserve Com Msg
  std::vector<geometry_msgs::msg::Point> mpcComPositionMsgs;
  mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

  // Extract Com and Feet from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);

    // Fill com position and pose msgs
    geometry_msgs::msg::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());
    mpcComPositionMsgs.push_back(pose.position);

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
      const auto position = getPointMsg(feetPositions[i]);
      feetMsgs[i].push_back(position);
    }
  });

  // Convert feet msgs to Array message
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts +
                              2);  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM Trajectory";

  // Future footholds
  visualization_msgs::msg::Marker sphereList;
  sphereList.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  sphereList.scale.x = footMarkerDiameter_;
  sphereList.scale.y = footMarkerDiameter_;
  sphereList.scale.z = footMarkerDiameter_;
  sphereList.ns = "Future footholds";
  sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto& subsystemSequence = modeSchedule.modeSequence;
  const auto tStart = mpcTimeTrajectory.front();
  const auto tEnd = mpcTimeTrajectory.back();
  for (size_t event = 0; event < eventTimes.size(); ++event) {
    if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {  // Only publish future footholds within the optimized horizon
      const auto preEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event]);
      const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]);
      const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

      const auto& model = pinocchioInterface_.getModel();
      auto& data = pinocchioInterface_.getData();
      pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(postEventState, centroidalModelInfo_));
      pinocchio::updateFramePlacements(model, data);

      const auto feetPosition = endEffectorKinematicsPtr_->getPosition(postEventState);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        if (!preEventContactFlags[i] && postEventContactFlags[i]) {  // If a foot lands, a marker is added at that location.
          sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
          sphereList.colors.push_back(getColor(feetColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  stateOptimizedPublisher_->publish(markerArray);
}

}  // namespace g1
}  // namespace ocs2
