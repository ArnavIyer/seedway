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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "eigen3/Eigen/QR"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;
using namespace std;
using namespace Eigen;

DEFINE_double(cp1_distance, 2.5, "Distance to travel for 1D TOC (cp1)");
DEFINE_double(cp1_curvature, 0.5, "Curvature for arc path (cp1)");
DEFINE_double(cp2_curvature, 0.5, "Curvature for arc path (cp2)");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

const double CLEARANCE = 0.1;
const double BASETOFRONT = 0.4;
const double HALFWIDTH = 0.1405;
const float WALL_TOLL = 0.6;
const int ORDER = 7;

} //namespace

namespace navigation {

History history {};
float forward_dist = 0;
float lidarMaxRange = 10;

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {

}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

double Navigation::ForwardPredict(float actuation_latency) {
  double forward_predicted_dist = 0;
  Vector2f forward_predicted_disp {0, 0};
  Vector2f vel_vec;
  Vector2f prev_vel_vec = robot_vel_;
  for(size_t i = 0; i < history.prev_commands.size(); i++) {
    float timestamp_diff;
    if (i + 1 < history.prev_commands.size()) {
      timestamp_diff = (history.prev_commands[i+1].msg.header.stamp - history.prev_commands[i].msg.header.stamp).toSec();
      timestamp_diff -= actuation_latency;
    } else {
      timestamp_diff = (ros::Time::now() - history.prev_commands[i].msg.header.stamp).toSec();
    }
    vel_vec = history.prev_commands[i].msg.velocity * Vector2f{cos(history.prev_commands[i].angle), sin(history.prev_commands[i].angle)};
    forward_predicted_disp += actuation_latency * prev_vel_vec + timestamp_diff * vel_vec;
    forward_predicted_dist += actuation_latency * prev_vel_vec.norm() + timestamp_diff * history.prev_commands[i].msg.velocity;
    prev_vel_vec = vel_vec;
  }
  return forward_predicted_dist;
}

float Navigation::DetectObstacles(double curvature, double forward_predicted_dist) {
  double distanceLeft = navigation::lidarMaxRange;
  
  for(Vector2f point : point_cloud_) {
    // Straight Line Case
    point[0] -= forward_predicted_dist;
    if(abs(curvature) <= 1e-6) {
      if(abs(point[1]) <= CLEARANCE + HALFWIDTH) {
        double xStoppingDistance = BASETOFRONT + CLEARANCE;
        distanceLeft = std::min(distanceLeft, point[0] - xStoppingDistance);
        visualization::DrawPoint(point, 0x00FF00, local_viz_msg_);
      }
    } 
    // Curvies
    // else {
    //   bool clockwise = curvature < 0;
    //   double r = abs(1.0/curvature);
      
    //   double h = BASETOFRONT + CLEARANCE;
    //   double w = HALFWIDTH + CLEARANCE;
    //   double r_1 = r - w;
    //   double r_2 = sqrt(pow(h, 2) + pow(r + w, 2));

    //   Vector2f c{0, r};
    //   // visualization::DrawCross(Vector2f {0, clockwise ? -r : r}, 0.2, 0xFF00FF, local_viz_msg_);
    //   //visualization::DrawArc(Vector2f {0, clockwise ? -r : r}, r_1, 0, 2 * 3.14, 0x0000000, local_viz_msg_);
    //   //visualization::DrawArc(Vector2f {0, clockwise ? -r : r}, r_2, 0, 2 * 3.14, 0x0000000, local_viz_msg_);
    //   Vector2f drawpoint = point;
    //   if (clockwise) point[1] *= -1;
    //   if(point[0] > 0 && (point - c).norm() >= r_1 && (point - c).norm() <= r_2){
    //     visualization::DrawPoint(drawpoint, 0x00FF00, local_viz_msg_);
    //     double theta = atan2(point[0], r - point[1]);
    //     double omega = atan2(h, r_1);
    //     double phi = theta - omega;
    //     double f = r * phi;
    //     distanceLeft = std::min(distanceLeft, f);
    //     // printf("%f %f\n", f, distanceLeft);
    //   }
    // }
  }
  return distanceLeft;
}


void Navigation::PolynomialRegression(const vector<double>& t, const vector<double>& v, std::vector<double>& coeff, size_t order) {
  // vector<double> t(points.size());
  // vector<double> v(points.size());
  // for (size_t i = 0; i < points.size(); i++) {
  //   t[i] = points[i].x();
  //   v[i] = (points[i].y());
  // }
  // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
	Eigen::MatrixXd T(t.size(), order + 1);
	Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
	Eigen::VectorXd result;

	// check to make sure inputs are correct
	assert(t.size() == v.size());
	assert(t.size() >= order + 1);
	// Populate the matrix
	for(size_t i = 0 ; i < t.size(); ++i)
	{
		for(size_t j = 0; j < order + 1; ++j)
		{
			T(i, j) = pow(t.at(i), j);
		}
	}
	
	// Solve for linear least square fit
	result = T.householderQr().solve(V);
	coeff.resize(order+1);
	for (size_t k = 0; k < order+1; k++)
	{
		coeff[k] = result[k];
	}
}

double Navigation::EvaluateFunction(vector<double>& coeffs, double x) {
  double sum = 0;
  double x_ponent = 1;
  for(size_t i = 0; i < coeffs.size(); i++) {
    sum += x_ponent * coeffs[i];
    x_ponent *= x;
  }
  return sum;
}

void Navigation::ObserveTrackWalls(const vector<Vector2f>& cloud,
                                   double time,
                                   int right_point_idx,
                                   int left_point_idx) {
  vector<Vector2f> right_wall;
  vector<double> right_wall_x;
  vector<double> right_wall_y;
  size_t i = right_point_idx;

  // get to the first point in the cloud

  while (i != UINT32_MAX && (cloud[i] - cloud[i-1]).norm() < WALL_TOLL) {
    i--;
  }
  i++;
  // populate wall vectors
  while (i < cloud.size() && (cloud[i] - cloud[i-1]).norm() < WALL_TOLL) {
    right_wall.push_back(cloud[i]);
    right_wall_x.push_back(cloud[i].x());
    right_wall_y.push_back(cloud[i].y());
    i++;
  }

  vector<Vector2f> left_wall;
  vector<double> left_wall_x;
  vector<double> left_wall_y;
  i = left_point_idx;

  while(i < cloud.size() && (cloud[i] - cloud[i-1]).norm() < WALL_TOLL) {
    i++;
  }
  i--;

  while(i != UINT32_MAX && (cloud[i] - cloud[i-1]).norm() < WALL_TOLL) {
    left_wall.push_back(cloud[i]);
    left_wall_x.push_back(cloud[i].x());
    left_wall_y.push_back(cloud[i].y());
    i--;
  }

  // populate path length t-vectors
  vector<double> right_wall_t;
  double curr_disp = 0;
  right_wall_t.push_back(0);
  for(size_t i = 1; i < right_wall.size(); i++) {
    curr_disp += (right_wall[i] - right_wall[i-1]).norm();
    right_wall_t.push_back(curr_disp);
  }

  vector<double> left_wall_t;
  curr_disp = 0;
  left_wall_t.push_back(0);
  for(size_t i = 1; i < left_wall.size(); i++) {
    curr_disp += (left_wall[i] - left_wall[i-1]).norm();
    left_wall_t.push_back(curr_disp);
  } 

  // std::cout << right_wall_t[0] << " " << right_wall_t[1] << std::endl;
  vector<double> right_x_coeffs, right_y_coeffs, left_x_coeffs, left_y_coeffs;
  // std::cout << "right_wall_x: " << right_wall_t.size() << " " << right_wall_x.size() << std::endl;
  // std::cout << "right_wall_y: " << right_wall_t.size() << " " << right_wall_y.size() << std::endl;
  // std::cout << "left_wall_x: " << left_wall_t.size() << " " << left_wall_x.size() << std::endl;
  // std::cout << "left_wall_y: " << left_wall_t.size() << " " << left_wall_y.size() << std::endl;
  cout << "he:"  << endl;

  if (left_wall_x.size() == 0) {
    left_wall_t.clear();
  }

  if (right_wall_x.size() == 0) {
    right_wall_t.clear();
  }

  Navigation::PolynomialRegression(right_wall_t, right_wall_x, right_x_coeffs, ORDER);
  Navigation::PolynomialRegression(right_wall_t, right_wall_y, right_y_coeffs, ORDER);
  Navigation::PolynomialRegression(left_wall_t, left_wall_x, left_x_coeffs, ORDER);
  Navigation::PolynomialRegression(left_wall_t, left_wall_y, left_y_coeffs, ORDER);

  estimated_right_wall_.clear();
  for (double t = 0; t < right_wall_t.back(); t += 0.1) {
    Vector2f point(Navigation::EvaluateFunction(right_x_coeffs, t), Navigation::EvaluateFunction(right_y_coeffs, t));
    estimated_right_wall_.push_back(point);
  }

  estimated_left_wall_.clear();
  for (double t = 0; t < left_wall_t.back(); t += 0.1) {
    Vector2f point(Navigation::EvaluateFunction(left_x_coeffs, t), Navigation::EvaluateFunction(left_y_coeffs, t));
    estimated_left_wall_.push_back(point);
  }
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;

  // right points

  // left points                           
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
  
  cout << "estimated_right_wall_.size(): " << estimated_right_wall_.size() << endl;
  for (size_t i = 1; i < estimated_right_wall_.size(); i++) {
    visualization::DrawLine(estimated_right_wall_[i - 1], estimated_right_wall_[i], 0x2934f7, global_viz_msg_);
  }

  cout << "estimated_left_wall_.size(): " << estimated_left_wall_.size() << endl;
  for (size_t i = 1; i < estimated_left_wall_.size(); i++) {
    visualization::DrawLine(estimated_left_wall_[i - 1], estimated_left_wall_[i], 0x2934f7, global_viz_msg_);
  }

  const double actuation_latency = 0.12;
  double forward_predicted_disp = ForwardPredict(actuation_latency);
  DetectObstacles(0, forward_predicted_disp);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

  history.prev_commands.push_back(CommandHistory {robot_vel_, drive_msg_, odom_angle_});
}

}  // namespace navigation
