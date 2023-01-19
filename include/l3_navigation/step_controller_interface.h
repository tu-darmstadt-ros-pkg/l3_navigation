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

#ifndef L3_STEP_CONTROLLER_INTERFACE_H__
#define L3_STEP_CONTROLLER_INTERFACE_H__

#include <ros/ros.h>

#include <l3_libs/helper.h>

#include <l3_footstep_planning_msgs/ExecuteStepPlanAction.h>

namespace l3_navigation
{
using namespace l3;

class StepControllerInterface
{
public:
  // typedefs
  typedef l3::SharedPtr<StepControllerInterface> Ptr;
  typedef l3::SharedPtr<const StepControllerInterface> ConstPtr;

  typedef boost::function<void(const l3_footstep_planning_msgs::ExecuteStepPlanFeedback& feedback)> FeedbackCB;

  StepControllerInterface(ros::NodeHandle& nh, const std::string& topic);
  virtual ~StepControllerInterface();

  inline void setFeedbackCallback(const FeedbackCB& callback) { feedback_cb_ = callback; }

  void execute(const l3_footstep_planning_msgs::StepPlan& step_plan);

  bool isRunning() const { return !execute_step_plan_ac_->getState().isDone(); }

  const l3_footstep_planning_msgs::ExecuteStepPlanFeedback& getFeedback() const { return feedback_; }

protected:
  void stepControllerFeedbackCB(const l3_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback);
  void stepControllerResultCB(const l3_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result);

  l3_footstep_planning_msgs::ExecuteStepPlanFeedback feedback_;

  // callbacks
  FeedbackCB feedback_cb_;

  // action clients
  SimpleActionClient<l3_footstep_planning_msgs::ExecuteStepPlanAction>::Ptr execute_step_plan_ac_;
};
}  // namespace l3_navigation

#endif
