//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"


#ifndef NAVIGATION_H
#define NAVIGATION_H

using namespace std;
using namespace Eigen;

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct CommandHistory {
  Eigen::Vector2f robot_vel;
  amrl_msgs::AckermannCurvatureDriveMsg msg;
  float angle;
};

struct History {
  Eigen::Vector2f prev_lidar_loc_; // previous LIDAR location for where sensors were read
  float prev_lidar_angle; // previous robot angle
  std::vector<CommandHistory> prev_commands; // previously sent drive messages
};

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

extern History history;

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void PolynomialRegression(const vector<double>& t, const vector<double>& v, std::vector<double>& coeff, size_t order);
  void ObserveTrackWalls(const std::vector<Eigen::Vector2f>& cloud,
                         double time,
                         int right_point_idx,
                         int left_point_idx);
  double EvaluateFunction(vector<double>& coeffs, double x);
  float DetectObstacles(double curvature, double forward_predicted_disp);
  double ForwardPredict(float actuation_latency);

 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  vector<Eigen::Vector2f> estimated_right_wall_;
  vector<Eigen::Vector2f> estimated_left_wall_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Map of the environment.
  vector_map::VectorMap map_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
