#pragma once

#include <fstream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "fsr_glove/glove.h"

class Glove {
public:
  Glove(bool replay, bool record, std::string dest_dir, ros::NodeHandle pnh = ros::NodeHandle("~"));
  ~Glove();

private:

  Eigen::Vector3d quaternion_rotate(Eigen::Quaterniond q, Eigen::Vector3d u);
  visualization_msgs::Marker genmark(Eigen::Vector3d pt_marker, Eigen::Quaterniond q_marker, double length, double radius, float chroma, std::string ns);
  visualization_msgs::Marker genarrow(Eigen::Vector3d pt_marker, Eigen::Quaterniond q_marker, double length, double radius, double force, std::string ns);
  void vicon_wrist_cb(const geometry_msgs::TransformStamped msg);
  void vicon_bottle_cb(const geometry_msgs::TransformStamped msg);
  void vicon_lid_cb(const geometry_msgs::TransformStamped msg);
  void imu_cb(const geometry_msgs::PoseArrayPtr msg);
  void imu_rel_cb(const geometry_msgs::PoseArrayPtr msg);
  void cal_cb(const std_msgs::StringPtr msg);
  void force_cb(const fsr_glove::glovePtr msg);
  void image_raw_cb(const sensor_msgs::Image msg);
  void bad_cb(const std_msgs::StringPtr msg);

  void force_vector(tf::Transform glove_tf, std::string child_frame);
  void zero_rel_poses();

  void publish_state_finger(int idx_from_, int idx_to_);
  void publish_state_palm();
  void publish_state();

  void add_joint_state(sensor_msgs::JointState& js, std::string name, Eigen::Quaterniond& q);

  bool calibrated_;
  std::vector<std::string> link_names_;
  std::vector<double> link_lengths_;
  std::vector<int> parents_;
  std::vector<Eigen::Quaterniond> canonical_pose_;
  std::vector<Eigen::Vector3d> canonical_origin_;
  //std::vector<Eigen::Quaterniond> last_pose_;
  //std::vector<Eigen::Quaterniond> cal_data_;
  std::vector<Eigen::Quaterniond> global2sensors_;
  std::vector<Eigen::Quaterniond> global2phalanges_;
  std::vector<Eigen::Quaterniond> phalanges2sensors_;

  std::vector<Eigen::Quaterniond> global2sensors_zero_;
  std::vector<Eigen::Quaterniond> global2phalanges_zero_;
  std::vector<Eigen::Quaterniond> phalanges2sensors_zero_;

  std::vector<double> forces_;

  ros::NodeHandle nh_;

  ros::Publisher state_pub_;
  ros::Subscriber vicon_wrist_sub_;
  ros::Subscriber vicon_bottle_sub_;
  ros::Subscriber vicon_lid_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber imu_rel_sub_;
  ros::Subscriber cal_sub_;
  ros::Subscriber image_raw_sub_;
  ros::Subscriber force_sub_;
  ros::Subscriber bad_sub_;

  tf::TransformBroadcaster br_;
  tf::Transform vicon_wrist_tf;
  tf::Transform vicon_bottle_tf;
  tf::Transform vicon_lid_tf;
  ros::Publisher marker_pub_;
  ros::Publisher arrow_pub_;
  bool vicon_wrist_;
  bool vicon_bottle_;
  bool vicon_lid_;

  //Note: bags split by replay_ couldn't be visualized directly unless modifying writing imuctrl
  bool record_;
  bool replay_;
  rosbag::Bag bag;
  bool continue_;
  int bag_counter_;
  bool bad_trigger_;
  std::string dest_dir_;
  bool begin_;

  const double Pi;
  const double PalmWidth;
  const double PalmLength;
  const double PalmHeight;
  const double ProximalPhalangeLength;
  const double ProximalPhalanxLength;
  const double MiddlePhalangeLength;
  const double DistalPhalangeLength;
  const double DistalPhalanxLength;
  const double radius;
  const double arrayLength;
};
