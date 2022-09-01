#include <l3_navigation/nav_goal_marker_vis.h>

#include <l3_libs/conversions/l3_msg_foothold_conversions.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_msgs/UpdateMode.h>
#include <l3_footstep_planning_msgs/UpdateFeetService.h>
#include <l3_footstep_planning_msgs/StepPlanRequestResult.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>
#include <l3_footstep_planning_tools/feet_pose_generator.h>

#include <l3_footstep_planning_vis_tools/visualization.h>

namespace l3_navigation
{
using namespace l3;
using namespace l3_footstep_planning;

NavGoalMarkerVis::NavGoalMarkerVis()
  : PlanningVisPlugin("nav_goal_marker_vis")
  , nav_goal_running_(false)
{
  marker_ns_ = "nav_step_plan";
}

bool NavGoalMarkerVis::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PlanningVisPlugin::initialize(params))
    return false;

  // load parameters
  nav_frame_ = param("frame", std::string("odom"), true);

  const std::string& marker_topic = param("marker_topic", std::string("nav_goal"), true);
  const std::string& step_controller_topic = param("step_controller_topic", std::string("execute_step_plan"), true);

  double scaling = param("scaling", 1.0, true);
  double marker_offset_z = param("marker_offset_z", 0.0, true);

  auto_planning_ = param("auto_planning", true, true);
  auto_execute_ = param("auto_execute", false, true);

  planning_horizon_ = param("planning_horizon", 1.0, true);
  max_planning_time_ = param("max_planning_time", 2.0, true);

  // determine center to base
  Transform center_to_base;
  if (RobotModel::kinematics())
    center_to_base = RobotModel::kinematics()->calcStaticFeetCenterToBase(*RobotModel::description());
  else
    center_to_base.setZ(0.5 * RobotModel::description()->getBaseInfo(BaseInfo::MAIN_BODY_IDX).size.x());

  center_to_base.setZ(center_to_base.z() + marker_offset_z);

  // init marker
  nav_goal_marker_.reset(new NavGoalMarker(nh_, marker_topic, nav_frame_, *RobotModel::description(), scaling, center_to_base));
  nav_goal_marker_->setPoseUpdateCallback(boost::bind(&NavGoalMarkerVis::processMarkerPoseUpdate, this, _1));

  // init marker menu
  nav_goal_marker_->insertMenuItem("Generate Plan", boost::bind(&NavGoalMarkerVis::sendStepPlanRequestCb, this));
  nav_goal_marker_->insertCheckableMenuItem("Auto Generate Plan", auto_planning_, [this](bool checked) { auto_planning_ = checked; });
  nav_goal_marker_->insertCheckableMenuItem("Auto Execute Plan", auto_execute_, [this](bool checked) { auto_execute_ = checked; });
  nav_goal_marker_->insertMenuItem("Execute", [this](const MenuHandler::FeedbackConstPtr&) { step_controller_->execute(step_plan_.toMsg()); });
  nav_goal_marker_->insertMenuItem("Start Nav Goal", boost::bind(&NavGoalMarkerVis::startNavGoalCb, this));
  nav_goal_marker_->insertMenuItem("Stop Nav Goal", boost::bind(&NavGoalMarkerVis::stopNavGoalCb, this));

  // init step controller
  step_controller_.reset(new StepControllerInterface(nh_, step_controller_topic));
  step_controller_->setFeedbackCallback(boost::bind(&NavGoalMarkerVis::processStepContollerFeedback, this, _1));

  // publisher
  step_plan_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/nav_step_plan", 1, true);
  step_execution_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/exe_step_plan", 1, true);

  // service clients
  generate_feet_pose_client_ = nh_.serviceClient<l3_footstep_planning_msgs::GenerateFeetPoseService>("generate_feet_pose");
  update_feet_client_ = nh_.serviceClient<l3_footstep_planning_msgs::UpdateFeetService>("update_feet");

  // action clients
  step_plan_request_ac_ = SimpleActionClient<l3_footstep_planning_msgs::StepPlanRequestAction>::create(nh_, "step_plan_request", false);

  // start timer
  update_timer_ = nh_.createTimer(ros::Rate(10.0), &NavGoalMarkerVis::update, this);

  resetStepPlan();

  return true;
}

void NavGoalMarkerVis::resetStepPlan()
{
  // preempt all active goals to prevent internal deadlock of action state client
  if (!step_plan_request_ac_->getState().isDone())
    step_plan_request_ac_->stopTrackingGoal();

  nav_goal_running_ = false;
  nav_goal_reached_ = false;
  currently_executed_step_index_ = -1;

  step_plan_.clear();
  step_plan_markers_.markers.clear();

  // publish clear
  step_plan_markers_.markers.push_back(createResetMarker(marker_ns_));
  step_plan_pub_.publish(step_plan_markers_);
  step_plan_markers_.markers.clear();

  step_execution_pub_.publish(step_plan_markers_);
  step_execute_markers_.markers.clear();
}

void NavGoalMarkerVis::update(const ros::TimerEvent& /*event*/)
{
  // do nothing when no continuous walking is enabled
  if (!nav_goal_running_)
    return;

  if (!step_plan_request_ac_->getState().isDone())
    return;

  const l3_footstep_planning_msgs::ExecuteStepPlanFeedback& feedback = step_controller_->getFeedback();

  std_msgs::Header header;
  header.frame_id = nav_frame_;
  header.stamp = ros::Time::now();

  l3_msgs::FootholdArray start_footholds;
  l3_msgs::FootholdArray goal_footholds;

  StepIndex start_step_idx = 0;

  /// determine start

  // start from scratch
  if (step_plan_.empty())
  {
    ROS_INFO("Starting from scratch...");
    l3_footstep_planning::determineStartFootholds(start_footholds, generate_feet_pose_client_, header);
  }
  // continue execution
  else
  {
    // generate a new plan only after one executed step
    if (feedback.currently_executing_step_idx < 0 || currently_executed_step_index_ >= feedback.currently_executing_step_idx)
      return;

    Step::ConstPtr step = step_plan_.getStep(feedback.first_changeable_step_idx, max_planning_time_);
    if (step)
    {
      footholdArrayL3ToMsg(step->getAllFootholds(), start_footholds);
      start_step_idx = step->getStepIndex();
    }
    else
    {
      ROS_ERROR("[NavGoalMarkerVis] Ran out of steps...aborting!");
      stopNavGoalCb();
      return;
    }
  }

  /// determine goal

  FootholdArray start_feet;
  footholdArrayMsgToL3(start_footholds, start_feet);
  Pose start_pose = RobotModel::calcFeetCenter(start_feet);

  Pose goal_pose;
  poseMsgToL3(nav_goal_marker_->getPose().pose, goal_pose);

  // lerp between the poses
  Vector3 direction = goal_pose.getPosition() - start_pose.getPosition();
  double dist = direction.norm();
  if (dist > planning_horizon_)
  {
    goal_pose.position() = start_pose.getPosition() + direction * (planning_horizon_ / dist);
    goal_pose.setRPY(0.0, 0.0, calcOrientation(direction));
  }
  else
    nav_goal_reached_ = true;

  footholdArrayL3ToMsg(RobotModel::getNeutralStance(goal_pose), goal_footholds);
  FootPoseTransformer::transformToRobotFrame(goal_footholds);

  /// align target feet to terrain

  l3_footstep_planning_msgs::UpdateFeetServiceRequest req;
  l3_footstep_planning_msgs::UpdateFeetServiceResponse resp;

  req.feet = goal_footholds;
  for (l3_msgs::Foothold& f : req.feet)
    f.header = header;

  req.update_mode.mode = l3_footstep_planning_msgs::UpdateMode::UPDATE_MODE_3D | l3_footstep_planning_msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID;

  if (update_feet_client_.call(req, resp))
    goal_footholds = resp.feet;
  else
    ROS_ERROR("[NavGoalMarkerVis] Could not call UpdateFeet service!");

  /// send step plan request

  sendStepPlanRequest(start_footholds, goal_footholds, max_planning_time_, start_step_idx);

  currently_executed_step_index_ = feedback.currently_executing_step_idx;
}

void NavGoalMarkerVis::processMarkerPoseUpdate(const geometry_msgs::Pose& pose)
{
  if (auto_planning_)
    sendStepPlanRequestCb();
}

void NavGoalMarkerVis::processStepContollerFeedback(const l3_footstep_planning_msgs::ExecuteStepPlanFeedback& feedback)
{
  // publish current step execution
  publishStepExecution();
}

void NavGoalMarkerVis::sendStepPlanRequest(const l3_msgs::FootholdArray& start_footholds, const l3_msgs::FootholdArray& goal_footholds, double max_planning_time, const StepIndex& step_idx)
{
  l3_footstep_planning_msgs::StepPlanRequestGoal request;
  request.plan_request.header.frame_id = nav_goal_marker_->getNavFrame();
  request.plan_request.header.stamp = ros::Time::now();
  request.plan_request.planning_mode = l3_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_3D;
  request.plan_request.start_foot_idx = l3_footstep_planning_msgs::StepPlanRequest::AUTO_START_FOOT_IDX;
  request.plan_request.start_step_idx = step_idx;
  request.plan_request.max_planning_time = max_planning_time;
  request.plan_request.start_footholds = start_footholds;
  request.plan_request.goal_footholds = goal_footholds;
  step_plan_request_ac_->sendGoal(request, boost::bind(&NavGoalMarkerVis::stepPlanResultCb, this, _2));

  ROS_DEBUG("[NavGoalMarkerVis] Request Start with idx %u", request.plan_request.start_step_idx);
  for (const l3_msgs::Foothold& f : request.plan_request.start_footholds)
    ROS_DEBUG("[NavGoalMarkerVis] [%u] %.3f / %.3f", f.idx, f.pose.position.x, f.pose.position.y);
}

void NavGoalMarkerVis::publishCurrentStepPlan()
{
  msgs::StepArray steps;
  stepArrayL3ToMsg(step_plan_.getSteps().asArray(), steps);
  FootPoseTransformer::transformToPlannerFrame(steps);

  updateMarkerArray(step_plan_markers_, stepPlanToFootMarkerArray(steps, *RobotModel::description(), true, marker_ns_));
  step_plan_pub_.publish(step_plan_markers_);
  removeDeletedMarkers(step_plan_markers_);
}

void NavGoalMarkerVis::publishStepExecution()
{
  const l3_footstep_planning_msgs::ExecuteStepPlanFeedback& feedback = step_controller_->getFeedback();

  if (feedback.currently_executing_step_idx < 0)
    return;

  Step::ConstPtr step = step_plan_.getSteps().getStep(feedback.currently_executing_step_idx);

  if (!step)
  {
    ROS_ERROR("[NavGoalMarkerVis] Step Controller is executing unknown step '%i'!", feedback.currently_executing_step_idx);
    return;
  }

  msgs::Step step_msg;
  stepL3ToMsg(*step, step_msg);
  FootPoseTransformer::transformToPlannerFrame(step_msg);

  visualization_msgs::MarkerArray markers;
  markers = stepToFootMarkerArray(step_msg, *RobotModel::description(), createColorMsg(0.3, 0.3, 0.3, 1.0), marker_ns_);
  for (visualization_msgs::Marker& marker : markers.markers)
  {
    marker.scale.x *= 1.1;
    marker.scale.y *= 1.1;
    marker.scale.z *= 1.1;
  }

  updateMarkerArray(step_execute_markers_, markers);
  step_execution_pub_.publish(step_execute_markers_);
  removeDeletedMarkers(step_execute_markers_);
}

void NavGoalMarkerVis::sendStepPlanRequestCb()
{
  if (!nav_goal_running_)
  {
    std_msgs::Header header;
    header.frame_id = nav_frame_;
    header.stamp = ros::Time::now();

    l3_msgs::FootholdArray start_footholds;
    l3_footstep_planning::determineStartFootholds(start_footholds, generate_feet_pose_client_, header);

    sendStepPlanRequest(start_footholds, nav_goal_marker_->getFootholds());
  }
}

void NavGoalMarkerVis::startNavGoalCb()
{
  ROS_INFO("[NavGoalMarkerVis] Starting nav goal...");

  resetStepPlan();

  nav_goal_running_ = true;
}

void NavGoalMarkerVis::stopNavGoalCb()
{
  ROS_INFO("[NavGoalMarkerVis] Stopping nav goal...");
  resetStepPlan();
}

void NavGoalMarkerVis::stepPlanResultCb(const l3_footstep_planning_msgs::StepPlanRequestResultConstPtr& result)
{
  if (hasError(result->status))
  {
    ROS_ERROR("[NavGoalMarkerVis] Failed to generate step plan!");
    if (nav_goal_running_)
      currently_executed_step_index_ = step_controller_->getFeedback().currently_executing_step_idx - 1;  // trigger replanning
    return;
  }

  // ROS_INFO_STREAM("Received Start with idx " << request.plan_request.start_step_idx << ":\n" << request.plan_request.start);

  if (nav_goal_running_)
  {
    if (result->step_plan.plan.steps.empty())
    {
      ROS_ERROR("Received empty plan!");
      return;
    }

    StepPlan temp(result->step_plan);
    Step::ConstPtr step = temp.getfirstStep();

    ROS_DEBUG("[NavGoalMarkerVis] Received step plan starting with idx %u", step->getStepIndex());
    for (Foothold::ConstPtr f : step->getAllFootholds())
      ROS_DEBUG("[NavGoalMarkerVis] [%u] %.3f / %.3f", f->idx, f->x(), f->y());

    step_plan_.stitchStepPlan(result->step_plan);
    step_controller_->execute(result->step_plan);

    if (nav_goal_reached_)
    {
      ROS_INFO("[NavGoalMarkerVis] Goal reachead. Stopping nav goal.");
      nav_goal_running_ = false;
    }
  }
  else
    step_plan_.fromMsg(result->step_plan);

  // publish plan
  publishCurrentStepPlan();

  // auto execute step plan
 if (auto_execute_)
   step_controller_->execute(result->step_plan);
}
}  // namespace l3_navigation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_navigation::NavGoalMarkerVis, l3_footstep_planning::PlanningVisPlugin)
