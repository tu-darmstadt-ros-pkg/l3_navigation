//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef L3_NAVIGATION_NAV_GOAL_MARKER_VIS_H__
#define L3_NAVIGATION_NAV_GOAL_MARKER_VIS_H__

#include <l3_footstep_planning_vis_tools/base/planning_vis_plugin.h>

#include <l3_navigation/nav_goal_marker.h>
#include <l3_navigation/step_controller_interface.h>

#include <l3_footstep_planning_msgs/StepPlanRequestAction.h>

#include <l3_footstep_planning_tools/feet_pose_generator_client.h>

namespace l3_navigation
{
using namespace l3;
using namespace l3_footstep_planning;

class NavGoalMarkerVis : public PlanningVisPlugin
{
public:
  NavGoalMarkerVis();

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

protected:
  void resetStepPlan();

  void update(const ros::TimerEvent& event);

  void processMarkerPoseUpdate(const geometry_msgs::Pose& pose);
  void processStepContollerFeedback(const l3_footstep_planning_msgs::ExecuteStepPlanFeedback& feedback);

  void sendStepPlanRequest(const l3_msgs::FootholdArray& start_footholds, const l3_msgs::FootholdArray& goal_footholds, double max_planning_time = 0.0, const StepIndex& step_idx = 0);

  // visualization
  void publishCurrentStepPlan();
  void publishStepExecution();

  // callbacks from menu
  void snapToRobotCb();
  void sendStepPlanRequestCb();
  void startNavGoalCb();

  void stopNavGoalCb();

  // action client callbacks
  void stepPlanResultCb(const l3_footstep_planning_msgs::StepPlanRequestResultConstPtr& result);

  // interactive marker
  NavGoalMarker::Ptr nav_goal_marker_;

  // step controller interface
  StepControllerInterface::Ptr step_controller_;

  // feet pose generator
  FeetPoseGeneratorClient::Ptr feet_pose_generator_;

  // internal state
  bool auto_planning_ = false;
  bool auto_execute_ = false;
  bool nav_goal_running_;
  bool nav_goal_reached_;
  StepIndex currently_executed_step_index_;

  StepPlan step_plan_;

  // vis
  std::string marker_ns_;
  visualization_msgs::MarkerArray step_plan_markers_;
  visualization_msgs::MarkerArray step_execute_markers_;

  // parameters
  BaseIndex base_idx_;
  std::string nav_frame_;
  double planning_horizon_;   // distance to plan
  double max_planning_time_;  // maximal allocated planning time

  // publisher
  ros::Publisher step_plan_pub_;
  ros::Publisher step_execution_pub_;

  // service clients
  ros::ServiceClient update_feet_client_;

  // action clients
  SimpleActionClient<l3_footstep_planning_msgs::StepPlanRequestAction>::Ptr step_plan_request_ac_;

  // timer
  ros::Timer update_timer_;
};
}  // namespace l3_navigation

#endif
