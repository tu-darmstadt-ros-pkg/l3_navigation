//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_NAVIGATION_NAV_GOAL_MARKER_H__
#define L3_NAVIGATION_NAV_GOAL_MARKER_H__

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <l3_libs/robot_description/robot_description.h>

#define INT_MARKER_NAME "l3_nav_goal"

namespace l3_navigation
{
using namespace interactive_markers;
using namespace l3;

class NavGoalMarker
{
public:
  // typedefs
  typedef l3::SharedPtr<NavGoalMarker> Ptr;
  typedef l3::SharedPtr<const NavGoalMarker> ConstPtr;

  typedef boost::function<void(const geometry_msgs::Pose& pose)> PoseUpdateCB;

  NavGoalMarker(ros::NodeHandle& nh, const std::string& topic, const std::string& nav_frame, const l3::RobotDescription& robot_description, double marker_scale = 1.0,
                const Transform& center_to_base = Transform());
  virtual ~NavGoalMarker();

  inline void setPoseUpdateCallback(const PoseUpdateCB& callback) { pose_update_cb_ = callback; }

  /**
   * @brief Insert menu element to right click panel
   * @param title Title of element
   * @param feedback_cb Callback when element was changed/clicked
   * @return Handle of menu item
   */
  MenuHandler::EntryHandle insertMenuItem(const std::string& title, const MenuHandler::FeedbackCallback& feedback_cb);

  /**
   * @brief Insert checkable menu element to right click panel
   * @param title Title of element
   * @param state Intial checked state
   * @param feedback_cb Callback when element was changed/clicked
   * @return Handle of menu item
   */
  MenuHandler::EntryHandle insertCheckableMenuItem(const std::string& title, const MenuHandler::CheckState& state, const MenuHandler::FeedbackCallback& feedback_cb);
  MenuHandler::EntryHandle insertCheckableMenuItem(const std::string& title, bool checked, const boost::function<void(bool)>& cb);

  /**
   * @brief Gets current checked state from menu item
   * @param handle Handle of menu item
   * @return checked state of menu item
   */
  MenuHandler::CheckState getCheckState(const MenuHandler::EntryHandle& handle) const;

  /**
   * @brief Sets new checked state of menu item
   * @param handle Handle of menu item
   * @param state New checked state of menu item
   */
  void setCheckState(const MenuHandler::EntryHandle& handle, const MenuHandler::CheckState& state);

  /**
   * @brief Controls visibility of single menu items
   * @param handle Handle of menu item
   * @param visible Sets the visibility of the selected menu item
   */
  void setVisible(const MenuHandler::EntryHandle& handle, bool visible);

  /**
   * @brief Sets new pose of interactive marker.
   * @param pose new pose
   * @param header optional header defining frame in which the pose is given
   */
  void setPose(const geometry_msgs::Pose& pose, const std_msgs::Header& header = std_msgs::Header());

  /**
   * @brief Returns current pose of the interactive marker.
   * @return Current pose of interactive marker
   */
  inline const geometry_msgs::PoseStamped& getPose() const { return pose_; }

  inline const l3_msgs::FootholdArray& getFootholds() const { return footholds_; }

  inline const std::string& getNavFrame() const { return nav_frame_; }

  /**
   * @brief Returns state if marker is moved
   * @return true if marker is currently moving
   */
  inline bool isMoving() const { return is_moving_; }

protected:
  void addFootholdMarkers();

  void updateFootholdMarkers(const l3_msgs::FootholdArray& footholds);

  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);

  // parameters
  const l3::RobotDescription& robot_description_;

  std::string nav_frame_;

  // ui elements
  l3::SharedPtr<InteractiveMarkerServer> server_;

  MenuHandler menu_handler_;

  // callbacks
  PoseUpdateCB pose_update_cb_;

  // internal state
  geometry_msgs::PoseStamped pose_;
  l3_msgs::FootholdArray footholds_;

  bool is_moving_;

  // service clients
  ros::ServiceClient update_feet_client_;
};
}  // namespace l3_navigation

#endif
