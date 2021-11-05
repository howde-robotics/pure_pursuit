/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).

   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
 */

#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <kdl/frames.hpp>

// TODO: Figure out how to use tf2 DataConversions
// for more elegant and compact code
//#include <tf2_kdl/tf2_kdl.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <pure_pursuit/PurePursuitConfig.h>

using std::string;

class PurePursuit
{
public:

  //! Constructor
  PurePursuit();

  //! Compute velocit commands each time new odometry data is received.
  void computeVelocities(const ros::TimerEvent& t);

  //! Receive path to follow.
  void receivePath(nav_msgs::Path path);

  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose,
                                 const geometry_msgs::Transform& tf);
  
  //! Helper founction for computing eucledian distances in the x-y plane.
  template<typename T1, typename T2>
  double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
  }

  //! Run the controller.
  void run();
  
private:

  //! Dynamic reconfigure callback.
  void reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level);
  
  // Vehicle parameters
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  double ld_, pos_tol_;
  // Generic control variables
  double v_max_, v_, w_max_, v_min_, w_static_;
  // Control variables for Ackermann steering
  // Steering angle is denoted by delta
  double delta_, delta_vel_, acc_, jerk_, delta_max_;
  double controller_frequency_;
  nav_msgs::Path path_;
  unsigned idx_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  ackermann_msgs::AckermannDriveStamped cmd_acker_;
  // Outside of this +- angle, robot will only rotate in place
  double rot_only_ang_threshold_;
  
  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_odom_, sub_path_;
  ros::Publisher pub_vel_, pub_acker_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Timer controller_timer_;
  geometry_msgs::PoseStamped lookahead_;
  geometry_msgs::TransformStamped lookaheadTf_;
  string map_frame_id_, robot_frame_id_, lookahead_frame_id_, acker_frame_id_;

  dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig> reconfigure_server_;
  dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig>::CallbackType reconfigure_callback_;
  
};

PurePursuit::PurePursuit() : ld_(1.0), v_max_(0.1), v_(v_max_), w_max_(1.0), pos_tol_(0.1), idx_(0),
                             goal_reached_(true), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("map"), robot_frame_id_("base_link"),
                             lookahead_frame_id_("lookahead")
{
  // Get parameters from the parameter server
  nh_private_.param<double>("wheelbase", L_, 1.0);
  nh_private_.param<double>("lookahead_distance", ld_, 1.0);
  //nh_private_.param<double>("linear_velocity", v_, 0.1);
  nh_private_.param<double>("max_rotational_velocity", w_max_, 1.0);
  nh_private_.param<double>("static_rotational_velocity", w_static_, 0.5);
  nh_private_.param<double>("position_tolerance", pos_tol_, 0.1);
  nh_private_.param<double>("steering_angle_velocity", delta_vel_, 100.0);
  nh_private_.param<double>("acceleration", acc_, 100.0);
  nh_private_.param<double>("jerk", jerk_, 100.0);
  nh_private_.param<double>("rot_only_ang_threshold", rot_only_ang_threshold_, 0.5);
  nh_private_.param<double>("max_linear_velocity", v_max_, 1.5);
  nh_private_.param<double>("min_linear_velocity", v_min_, 0.5);
  nh_private_.param<double>("controller_frequency", controller_frequency_, 20.0);
  
  nh_private_.param<double>("steering_angle_limit", delta_max_, 1.57);
  nh_private_.param<string>("map_frame_id", map_frame_id_, "map");
  // Frame attached to midpoint of rear axle (for front-steered vehicles).
  nh_private_.param<string>("robot_frame_id", robot_frame_id_, "base_link");
  // Lookahead frame moving along the path as the vehicle is moving.
  nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");
  // Frame attached to midpoint of front axle (for front-steered vehicles).
  nh_private_.param<string>("ackermann_frame_id", acker_frame_id_, "rear_axle_midpoint");

  // Populate messages with static data
  lookahead_.header.frame_id = robot_frame_id_;
  lookaheadTf_.child_frame_id = lookahead_frame_id_;
  lookaheadTf_.header.frame_id = robot_frame_id_;

  cmd_acker_.header.frame_id = acker_frame_id_;
  cmd_acker_.drive.steering_angle_velocity = delta_vel_;
  cmd_acker_.drive.acceleration = acc_;
  cmd_acker_.drive.jerk = jerk_;
  
  sub_path_ = nh_.subscribe("path_segment", 1, &PurePursuit::receivePath, this);
//   sub_odom_ = nh_.subscribe("odometry", 1, &PurePursuit::computeVelocities, this);
  controller_timer_ = nh_.createTimer(ros::Rate(controller_frequency_), &PurePursuit::computeVelocities, this);
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_acker_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("cmd_acker", 1);

  reconfigure_callback_ = boost::bind(&PurePursuit::reconfigure, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_callback_);
}

void PurePursuit::computeVelocities(const ros::TimerEvent& t)
{
  // The velocity commands are computed, each time a new Odometry message is received.
  // Odometry is not used directly, but through the tf tree.

  // Get the current robot pose
  geometry_msgs::TransformStamped robot_in_map;
  geometry_msgs::TransformStamped map_in_robot;
  try
  {
    robot_in_map = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    map_in_robot = tf_buffer_.lookupTransform(robot_frame_id_, map_frame_id_, ros::Time(0));
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.
    for (; idx_ < path_.poses.size(); idx_++)
    {
      if (distance(path_.poses[idx_].pose.position, robot_in_map.transform.translation) > ld_)
      {

        // Transformed lookahead to base_link frame is lateral error
        tf2::doTransform(path_.poses[idx_], lookahead_, map_in_robot);

        // KDL::Frame F_bl_ld = transformToBaseLink(path_.poses[idx_].pose, robot_in_map.transform);
        // lookahead_.transform.translation.x = F_bl_ld.p.x();
        // lookahead_.transform.translation.y = F_bl_ld.p.y();
        // lookahead_.transform.translation.z = F_bl_ld.p.z();
        // F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
        //                         lookahead_.transform.rotation.y,
        //                         lookahead_.transform.rotation.z,
        //                         lookahead_.transform.rotation.w);
        
        // TODO: See how the above conversion can be done more elegantly
        // using tf2_kdl and tf2_geometry_msgs

        break;
      }
    }

    if (!path_.poses.empty() && idx_ >= path_.poses.size())
    {
      // We are approaching the goal,
      // which is closer than ld

      // This is the pose of the goal w.r.t. the base_link frame
      geometry_msgs::PoseStamped goal_in_robot;
      tf2::doTransform(path_.poses.back(), goal_in_robot, map_in_robot);
    //   KDL::Frame F_bl_end = transformToBaseLink(path_.poses.back().pose, tf.transform);

      const double x = goal_in_robot.pose.position.x;
      const double y = goal_in_robot.pose.position.y;

      if (std::sqrt(x*x + y*y) <= pos_tol_)
      {
        // We have reached the goal
        goal_reached_ = true;

        // Reset the path
        path_ = nav_msgs::Path();
      }
      else
      {
        // We need to extend the lookahead distance
        // beyond the goal point.
      
        // Find the intersection between the circle of radius ld
        // centered at the robot (origin)
        // and the line defined by the last path pose
        double roll, pitch, yaw;
        tf::Quaternion quat(goal_in_robot.pose.orientation.x, goal_in_robot.pose.orientation.y, goal_in_robot.pose.orientation.z, goal_in_robot.pose.orientation.w);
        tf::Matrix3x3 quat_matrix(quat);
        quat_matrix.getRPY(roll, pitch, yaw);
        // F_bl_end.M.GetRPY(roll, pitch, yaw);
        double k_end = tan(yaw); // Slope of line defined by the last path pose
        // double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
        double l_end = y - k_end *  x;
        double a = 1 + k_end * k_end;
        double b = 2 * l_end;
        double c = l_end * l_end - ld_ * ld_;
        double D = sqrt(b*b - 4*a*c);
        double x_ld = (-b + copysign(D,v_)) / (2*a);
        double y_ld = k_end * x_ld + l_end;
        
        lookahead_.pose.position.x = x_ld;
        lookahead_.pose.position.y = y_ld;
        lookahead_.pose.position.z = goal_in_robot.pose.position.z;
        // lookahead_.transform.translation.x = x_ld;
        // lookahead_.transform.translation.y = y_ld;
        // lookahead_.transform.translation.z = F_bl_end.p.z();
        // F_bl_end.M.GetQuaternion(lookahead_.transform.rotation.x,
        //                          lookahead_.transform.rotation.y,
        //                          lookahead_.transform.rotation.z,
        //                          lookahead_.transform.rotation.w);
      }
    }

    if (!goal_reached_)
    {
      // We are tracking.
     
      // Compute the linear and angular velocity.
      // Lateral error is the y-value of the lookahead point (in base_link frame)
      double yt = lookahead_.pose.position.y;
      double ld_2 = ld_ * ld_;

      // d_yaw is the yaw error from base_link to the lookahead frame
      double d_yaw = std::atan2(lookahead_.pose.position.y, lookahead_.pose.position.x);
      // if d_yaw is too big, pure pursuit will drift sideways, prevent this by doing pure rotation
      if (std::fabs(d_yaw) > rot_only_ang_threshold_) {
        v_ = 0.0;
        cmd_vel_.angular.z = std::copysign(w_static_, yt);
      } else {
        // linear velocity is computed with a linear ramp based on d_yaw
        // the smaller d_yaw (lookahead is straight ahead), the faster we can go
        v_ = (1 - std::fabs(d_yaw) / rot_only_ang_threshold_) * (v_max_ - v_min_) + v_min_;
        cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );
      }
      ROS_DEBUG("d_yaw: %f v: %f w: %f", d_yaw, v_, cmd_vel_.angular.z);

      // Compute desired Ackermann steering angle
      cmd_acker_.drive.steering_angle = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );
      
      // Set linear velocity for tracking.
      cmd_vel_.linear.x = v_;
      cmd_acker_.drive.speed = v_;

      cmd_acker_.header.stamp = ros::Time::now();
    }
    else
    {
      // We are at the goal!

      // Stop the vehicle
      
      // The lookahead target is at our current pose.
      lookahead_.pose = geometry_msgs::Pose();
      lookahead_.pose.orientation.w = 1.0;
      
      // Stop moving.
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;

      cmd_acker_.header.stamp = ros::Time::now();
      cmd_acker_.drive.steering_angle = 0.0;
      cmd_acker_.drive.speed = 0.0;
    }

    // Publish the lookahead target transform.
    lookaheadTf_.header.stamp = ros::Time::now();
    lookaheadTf_.transform.translation.x = lookahead_.pose.position.x;
    lookaheadTf_.transform.translation.y = lookahead_.pose.position.y;
    lookaheadTf_.transform.translation.z = lookahead_.pose.position.z;
    lookaheadTf_.transform.rotation.x = lookahead_.pose.orientation.x;
    lookaheadTf_.transform.rotation.y = lookahead_.pose.orientation.y;
    lookaheadTf_.transform.rotation.z = lookahead_.pose.orientation.z;
    lookaheadTf_.transform.rotation.w = lookahead_.pose.orientation.w;
    tf_broadcaster_.sendTransform(lookaheadTf_);
    
    // Publish the velocities
    pub_vel_.publish(cmd_vel_);
    
    // Publish ackerman steering setpoints
    pub_acker_.publish(cmd_acker_);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM(ex.what());
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    pub_vel_.publish(cmd_vel_);
    cmd_acker_.header.stamp = ros::Time::now();
    cmd_acker_.drive.steering_angle = 0.0;
    cmd_acker_.drive.speed = 0.0;
    pub_acker_.publish(cmd_acker_);
  }
}

void PurePursuit::receivePath(nav_msgs::Path new_path)
{
  // When a new path received, the previous one is simply discarded
  // It is up to the planner/motion manager to make sure that the new
  // path is feasible.
  // Callbacks are non-interruptible, so this will
  // not interfere with velocity computation callback.
  
  if (new_path.header.frame_id == map_frame_id_)
  {
    path_ = new_path;
    idx_ = 0;
    if (new_path.poses.size() > 0)
    {
      goal_reached_ = false;
    }
    else
    {
      goal_reached_ = true;
      ROS_WARN_STREAM("Received empty path!");
    }
  }
  else
  {
    ROS_WARN_STREAM("The path must be published in the " << map_frame_id_
                    << " frame! Ignoring path in " << new_path.header.frame_id
                    << " frame!");
  }
  
}

KDL::Frame PurePursuit::transformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs

  return F_map_tf.Inverse()*F_map_pose;
}

void PurePursuit::run()
{
  ros::spin();
}

void PurePursuit::reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level)
{
  v_max_ = config.max_linear_velocity;
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "pure_pursuit");

  PurePursuit controller;
  controller.run();

  return 0;
}
