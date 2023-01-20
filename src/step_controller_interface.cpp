#include <l3_navigation/step_controller_interface.h>

#include <l3_step_controller_plugins/base/step_controller_plugin.h>

namespace l3_navigation
{
using namespace l3;

StepControllerInterface::StepControllerInterface(ros::NodeHandle& nh, const std::string& topic)
{
  feedback_.currently_executing_step_idx = -1;

  // action clients
  execute_step_plan_ac_ = SimpleActionClient<l3_footstep_planning_msgs::ExecuteStepPlanAction>::create(nh, topic, false);
}

StepControllerInterface::~StepControllerInterface() {}

void StepControllerInterface::execute(const l3_footstep_planning_msgs::StepPlan& step_plan)
{
  if (!execute_step_plan_ac_->isServerConnected())
  {
    ROS_ERROR("[StepControllerInterface] Step controller not available!");
    return;
  }

  ROS_INFO("[StepControllerInterface] Start step plan execution...");

  l3_footstep_planning_msgs::ExecuteStepPlanGoal goal;

  goal.step_plan = step_plan;

  execute_step_plan_ac_->sendGoal(goal, boost::bind(&StepControllerInterface::stepControllerResultCB, this, _2),
                                  SimpleActionClient<l3_footstep_planning_msgs::ExecuteStepPlanAction>::SimpleActiveCallback(),
                                  boost::bind(&StepControllerInterface::stepControllerFeedbackCB, this, _1));
}

void StepControllerInterface::stepControllerFeedbackCB(const l3_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback)
{
  feedback_ = *feedback;

  if (feedback_cb_)
    feedback_cb_(feedback_);
}

void StepControllerInterface::stepControllerResultCB(const l3_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
  feedback_.currently_executing_step_idx = -1;

  ROS_INFO("[StepControllerInterface] Step execution finished with state '%s'.",
           l3_step_controller::toString(static_cast<l3_step_controller::StepControllerState>(result->controller_state)).c_str());
}
}  // namespace l3_navigation
