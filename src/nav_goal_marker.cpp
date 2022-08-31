#include <l3_navigation/nav_goal_marker.h>

#include <tf/tf.h>

#include <l3_libs/conversions/l3_msg_conversions.h>
#include <l3_plugins/robot_model.h>

#include <l3_vis/visualization.h>

#include <l3_footstep_planning_msgs/UpdateMode.h>
#include <l3_footstep_planning_msgs/UpdateFeetService.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>

namespace l3_navigation
{
using namespace interactive_markers;
using namespace l3;
using namespace l3_footstep_planning;

NavGoalMarker::NavGoalMarker(ros::NodeHandle& nh, const std::string& topic, const std::string& nav_frame, const l3::RobotDescription& robot_description, double marker_scaling,
                             const Transform& center_to_base)
  : robot_description_(robot_description)
  , nav_frame_(nav_frame)
  , is_moving_(false)
{
  server_.reset(new InteractiveMarkerServer(topic, "", false));

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = nav_frame_;
  int_marker.pose.orientation.w = 1.0;
  int_marker.scale = marker_scaling;
  int_marker.name = INT_MARKER_NAME;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;

  /// init base marker
  const BaseInfo& base_info = robot_description_.getBaseInfo(BaseInfo::MAIN_BODY_IDX);

  visualization_msgs::Marker marker;
  marker.color = base_info.color;

  if (base_info.mesh_resource.empty())
  {
    marker.type = visualization_msgs::Marker::CUBE;
    poseL3ToMsg(center_to_base, marker.pose);
    marker.scale.x = 0.1 * marker_scaling;
    marker.scale.y = 0.1 * marker_scaling;
    marker.scale.z = 0.1 * marker_scaling;
  }
  else
  {
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = base_info.mesh_resource;
    poseL3ToMsg(center_to_base * base_info.mesh_offset, marker.pose);
    vectorL3ToMsg(base_info.mesh_scale, marker.scale);
  }

  control.markers.push_back(marker);
  control.orientation.w = 0.707;
  control.orientation.x = 0;
  control.orientation.y = 0.707;
  control.orientation.z = 0;
  control.name = "move_base";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  /// init moving axes
  control = visualization_msgs::InteractiveMarkerControl();

  // x-axis
  control.orientation.x = 0.707;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.orientation.w = 0.707;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // y-axis
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.707;
  control.orientation.w = 0.707;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // z-axis
  control.orientation.x = 0;
  control.orientation.y = 0.707;
  control.orientation.z = 0;
  control.orientation.w = 0.707;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  /// Menu handler
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = int_marker.scale * 0.1;
  marker.scale.y = int_marker.scale * 0.1;
  marker.scale.z = int_marker.scale * 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  control = visualization_msgs::InteractiveMarkerControl();
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.description = "Options";
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);

  // init interactive server marker
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, boost::bind(&NavGoalMarker::processFeedback, this, _1));

  menu_handler_.apply(*server_, INT_MARKER_NAME);

  /// init foothold markers
  addFootholdMarkers();

  server_->applyChanges();

  // service clients
  update_feet_client_ = nh.serviceClient<l3_footstep_planning_msgs::UpdateFeetService>("update_feet");
}

MenuHandler::EntryHandle NavGoalMarker::insertMenuItem(const std::string& title, const MenuHandler::FeedbackCallback& feedback_cb)
{
  MenuHandler::EntryHandle handle = menu_handler_.insert(title, feedback_cb);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
  return handle;
}

MenuHandler::EntryHandle NavGoalMarker::insertCheckableMenuItem(const std::string& title, const MenuHandler::CheckState& state, const MenuHandler::FeedbackCallback& feedback_cb)
{
  MenuHandler::EntryHandle handle = menu_handler_.insert(title, feedback_cb);
  menu_handler_.setCheckState(handle, state);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
  return handle;
}

MenuHandler::EntryHandle NavGoalMarker::insertCheckableMenuItem(const std::string& title, bool checked, const boost::function<void(bool)>& cb)
{
  // clang-format off
  MenuHandler::EntryHandle handle = menu_handler_.insert(title,
    [this, cb](const MenuHandler::FeedbackConstPtr& feedback)
    {
      bool checked = getCheckState(feedback->menu_entry_id) == MenuHandler::CheckState::UNCHECKED; // toggle status
      setCheckState(feedback->menu_entry_id, checked ? MenuHandler::CheckState::CHECKED : MenuHandler::CheckState::UNCHECKED);
      cb(checked);
    });
  // clang-format on
  menu_handler_.setCheckState(handle, checked ? MenuHandler::CheckState::CHECKED : MenuHandler::CheckState::UNCHECKED);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
  return handle;
}

MenuHandler::CheckState NavGoalMarker::getCheckState(const MenuHandler::EntryHandle& handle) const
{
  MenuHandler::CheckState state;
  menu_handler_.getCheckState(handle, state);
  return state;
}

void NavGoalMarker::setCheckState(const MenuHandler::EntryHandle& handle, const MenuHandler::CheckState& state)
{
  menu_handler_.setCheckState(handle, state);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void NavGoalMarker::setVisible(const MenuHandler::EntryHandle& handle, bool visible)
{
  menu_handler_.setVisible(handle, visible);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

NavGoalMarker::~NavGoalMarker() { server_->clear(); }

void NavGoalMarker::setPose(const geometry_msgs::Pose& pose, const std_msgs::Header& header)
{
  server_->setPose(INT_MARKER_NAME, pose);
  server_->applyChanges();
  pose_.header = header;
  pose_.pose = pose;
}

void NavGoalMarker::addFootholdMarkers()
{
  footholds_.clear();

  for (const FootInfoPair& p : robot_description_.getFootInfoMap())
  {
    const FootInfo& foot_info = p.second;

    // create foot marker
    Foothold foothold(foot_info.idx, Pose());
    l3_msgs::Foothold foothold_msg = foothold.toMsg();
    FootPoseTransformer::transformToPlannerFrame(foothold_msg);
    visualization_msgs::Marker marker = footToFootMarker(foothold.toMsg(), robot_description_);

    // create interactive marker
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = nav_frame_;
    poseL3ToMsg(foot_info.neutral_stance, int_marker.pose);
    int_marker.scale = 1.0;
    int_marker.name = foot_info.name;
    int_marker.controls.push_back(control);

    server_->insert(int_marker);

    foothold.setPose(foot_info.neutral_stance);
    footholds_.push_back(foothold.toMsg());
  }
}

void NavGoalMarker::updateFootholdMarkers(const l3_msgs::FootholdArray& footholds)
{
  footholds_.clear();

  for (const l3_msgs::Foothold& f : footholds)
  {
    server_->setPose(robot_description_.footId(f.idx), f.pose);
    footholds_.push_back(f);
  }

  server_->applyChanges();
}

void NavGoalMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback)
{
  // ROS_INFO_STREAM(*feedback);

  // only trigger on mouse release
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    is_moving_ = false;
  else
    is_moving_ = true;

  geometry_msgs::Pose pose_msg = feedback->pose;

  // get feet height from footstep planner
  l3_footstep_planning_msgs::UpdateFeetServiceRequest req;
  l3_footstep_planning_msgs::UpdateFeetServiceResponse resp;

  // only update central marker position when marker is not dragged
  if (!isMoving())
  {
    l3_msgs::Foothold foothold;
    foothold.header = feedback->header;
    foothold.pose = pose_msg;
    foothold.idx = RobotModel::description()->footIdxList().front();

    req.feet.push_back(foothold);
    req.update_mode.mode = l3_footstep_planning_msgs::UpdateMode::UPDATE_MODE_Z;

    if (update_feet_client_.call(req, resp))
      pose_msg = resp.feet.front().pose;
    else
      ROS_ERROR("Could not call UpdateFeet service!");

    setPose(pose_msg, feedback->header);
  }

  // updating feet pose
  req.feet.clear();
  Pose pose_l3;
  poseMsgToL3(pose_msg, pose_l3);
  footholdArrayL3ToMsg(robot_description_.getNeutralStance(pose_l3), req.feet);

  for (l3_msgs::Foothold& f : req.feet)
    f.header = feedback->header;

  req.update_mode.mode = l3_footstep_planning_msgs::UpdateMode::UPDATE_MODE_3D | l3_footstep_planning_msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID;

  if (update_feet_client_.call(req, resp))
    updateFootholdMarkers(resp.feet);
  else
    ROS_ERROR("Could not call UpdateFeet service!");

  if (!isMoving() && pose_update_cb_)
  {
    pose_update_cb_(pose_msg);
  }
}
}  // namespace l3_navigation
