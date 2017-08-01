#include "glove.h"

Glove::Glove(bool replay, bool record, std::string dest_dir, ros::NodeHandle pnh)
    : calibrated_(false),
      continue_(false),
      bad_trigger_(false),
      Pi(3.14159265),
      PalmWidth(pnh.param("PalmWidth", 0.09)),
      PalmLength(pnh.param("PalmLength", 0.09)),
      PalmHeight(pnh.param("PalmHeight", 0.03)),
      ProximalPhalangeLength(pnh.param("ProximalPhalangeLength", 0.03)),
      ProximalPhalanxLength(pnh.param("ProximalPhalanxLength", 0.03)),
      MiddlePhalangeLength(pnh.param("MiddlePhalangeLength", 0.025)),
      DistalPhalangeLength(pnh.param("DistalPhalangeLength", 0.025)),
      DistalPhalanxLength(pnh.param("DistalPhalanxLength", 0.03)),
      radius(pnh.param("radius", 0.008)),
      arrayLength(pnh.param("gridDimension", 0.08))
{
  record_ = record;
  replay_ = replay;
  dest_dir_ = dest_dir;
  if (replay_)
  {
    std::string filename = dest_dir_ + "0.bag";
    bag.open(filename.c_str(), rosbag::bagmode::Write);
    begin_ = false;
    bag_counter_ = 0;
  }

  vicon_wrist_ = false;
  vicon_bottle_ = false;
  vicon_lid_ = false;

  link_names_.resize(15);
  link_names_[0] = "palm_link";
  link_names_[1] = "proximal_phalanx_link_1";
  link_names_[2] = "distal_phalanx_link_1";
  link_names_[3] = "proximal_phalange_link_1";
  link_names_[4] = "middle_phalange_link_1";
  link_names_[5] = "distal_phalange_link_1";
  link_names_[6] = "proximal_phalange_link_2";
  link_names_[7] = "middle_phalange_link_2";
  link_names_[8] = "distal_phalange_link_2";
  link_names_[9] = "proximal_phalange_link_3";
  link_names_[10] = "middle_phalange_link_3";
  link_names_[11] = "distal_phalange_link_3";
  link_names_[12] = "proximal_phalange_link_4";
  link_names_[13] = "middle_phalange_link_4";
  link_names_[14] = "distal_phalange_link_4";

  link_lengths_.resize(15);
  link_lengths_[0] = -1;
  link_lengths_[1] = ProximalPhalanxLength;
  link_lengths_[2] = DistalPhalanxLength;
  link_lengths_[3] = ProximalPhalangeLength;
  link_lengths_[4] = MiddlePhalangeLength;
  link_lengths_[5] = DistalPhalangeLength;
  link_lengths_[6] = ProximalPhalangeLength;
  link_lengths_[7] = MiddlePhalangeLength;
  link_lengths_[8] = DistalPhalangeLength;
  link_lengths_[9] = ProximalPhalangeLength;
  link_lengths_[10] = MiddlePhalangeLength;
  link_lengths_[11] = DistalPhalangeLength;
  link_lengths_[12] = ProximalPhalangeLength;
  link_lengths_[13] = MiddlePhalangeLength;
  link_lengths_[14] = DistalPhalangeLength;

  parents_.resize(15);
  parents_[0] = -1;
  parents_[1] = 0;
  parents_[2] = 1;
  parents_[3] = 0;
  parents_[4] = 3;
  parents_[5] = 4;
  parents_[6] = 0;
  parents_[7] = 6;
  parents_[8] = 7;
  parents_[9] = 0;
  parents_[10] = 9;
  parents_[11] = 10;
  parents_[12] = 0;
  parents_[13] = 12;
  parents_[14] = 13;

  forces_.resize(30);

  vicon_wrist_sub_ = nh_.subscribe("vicon/wrist/wrist", 1000, &Glove::vicon_wrist_cb, this);
  vicon_bottle_sub_ = nh_.subscribe("vicon/bottle64/bottle64", 1000, &Glove::vicon_bottle_cb, this);
  vicon_lid_sub_ =
      nh_.subscribe("vicon/bottle64_lid/bottle64_lid", 1000, &Glove::vicon_lid_cb, this);
  imu_sub_ = nh_.subscribe("fsr_glove_imutracker_raw", 1000, &Glove::imu_cb, this);
  // imu_rel_sub_ = nh_.subscribe("fsr_glove_imutracker_rel", 1000, &Glove::imu_rel_cb, this);
  cal_sub_ = nh_.subscribe("fsr_glove_imutracker_imuctrl", 1000, &Glove::cal_cb, this);
  force_sub_ = nh_.subscribe("force_msg", 1000, &Glove::force_cb, this);
  image_raw_sub_ = nh_.subscribe("usb_cam/image_raw", 1000, &Glove::image_raw_cb, this);
  bad_sub_ = nh_.subscribe("fsr_glove_bad_data", 1000, &Glove::bad_cb, this);
  state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1000);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("force_marker", 1000);
  arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("force_arrow", 1000);

  // Set up canonical hand pose. This roughly approximates the joint angles of the hand in a flat
  // position, and is totally eyeballed and non-scientific.

  Eigen::Quaterniond q_ident;
  Eigen::Quaterniond q_thumb, q_1, q_2, q_3, q_4;
  q_ident.setIdentity();
  q_thumb = Eigen::AngleAxisd(0.2 * Pi, Eigen::Vector3d::UnitZ());
  q_1 = Eigen::AngleAxisd(0.05 * Pi, Eigen::Vector3d::UnitZ());
  q_2 = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  q_3 = Eigen::AngleAxisd(-0.05 * Pi, Eigen::Vector3d::UnitZ());
  q_4 = Eigen::AngleAxisd(-0.1 * Pi, Eigen::Vector3d::UnitZ());

  // global2sensors_.resize(15);
  canonical_pose_.resize(15);
  canonical_origin_.resize(15);
  for (int i = 0; i < 15; ++i)
  {
    canonical_pose_[i] = q_ident;
    canonical_origin_[i] = Eigen::Vector3d(0, 0, 0);
  }
  canonical_origin_[1] = Eigen::Vector3d(-.4 * PalmLength, 2 * (PalmWidth / 4), 0);
  canonical_origin_[2] = Eigen::Vector3d(ProximalPhalanxLength, 0, 0);
  canonical_origin_[3] = Eigen::Vector3d(PalmLength / 2, 1.5 * (PalmWidth / 4), 0);
  canonical_origin_[4] = Eigen::Vector3d(ProximalPhalangeLength, 0, 0);
  canonical_origin_[5] = Eigen::Vector3d(MiddlePhalangeLength, 0, 0);
  canonical_origin_[6] = Eigen::Vector3d(PalmLength / 2, 0.5 * (PalmWidth / 4), 0);
  canonical_origin_[7] = Eigen::Vector3d(ProximalPhalangeLength, 0, 0);
  canonical_origin_[8] = Eigen::Vector3d(MiddlePhalangeLength, 0, 0);
  canonical_origin_[9] = Eigen::Vector3d(PalmLength / 2, -0.5 * (PalmWidth / 4), 0);
  canonical_origin_[10] = Eigen::Vector3d(ProximalPhalangeLength, 0, 0);
  canonical_origin_[11] = Eigen::Vector3d(MiddlePhalangeLength, 0, 0);
  canonical_origin_[12] = Eigen::Vector3d(PalmLength / 2, -1.5 * (PalmWidth / 4), 0);
  canonical_origin_[13] = Eigen::Vector3d(ProximalPhalangeLength, 0, 0);
  canonical_origin_[14] = Eigen::Vector3d(MiddlePhalangeLength, 0, 0);
  canonical_pose_[1] = q_thumb;
  canonical_pose_[2] = q_thumb;
  canonical_pose_[3] = q_1;
  canonical_pose_[4] = q_1;
  canonical_pose_[5] = q_1;
  canonical_pose_[6] = q_2;
  canonical_pose_[7] = q_2;
  canonical_pose_[8] = q_2;
  canonical_pose_[9] = q_3;
  canonical_pose_[10] = q_3;
  canonical_pose_[11] = q_3;
  canonical_pose_[12] = q_4;
  canonical_pose_[13] = q_4;
  canonical_pose_[14] = q_4;
}

Glove::~Glove()
{
  bag.close();
  if (bad_trigger_)
  {
    std::stringstream ss;
    ss << dest_dir_ << bag_counter_ << ".bag";
    if (remove(ss.str().c_str()) != 0)
      perror("error delete the rear file");
  }
}

Eigen::Vector3d Glove::quaternion_rotate(Eigen::Quaterniond q, Eigen::Vector3d u)
{
  Eigen::Quaterniond q_u(0.0, u[0], u[1], u[2]);
  Eigen::Quaterniond q_v = q * (q_u * q.inverse());
  Eigen::Vector3d v(q_v.x(), q_v.y(), q_v.z());

  return v;
}

visualization_msgs::Marker Glove::genmark(Eigen::Vector3d pt_marker,
                                          Eigen::Quaterniond q_marker,
                                          double length,
                                          double radius,
                                          float chroma,
                                          std::string ns)
{
  visualization_msgs::Marker cylinder;
  cylinder.header.frame_id = "glove_link";
  cylinder.header.stamp = ros::Time::now();
  cylinder.ns = ns.c_str();
  cylinder.type = visualization_msgs::Marker::CYLINDER;

  cylinder.pose.position.x = pt_marker[0];
  cylinder.pose.position.y = pt_marker[1];
  cylinder.pose.position.z = pt_marker[2];

  cylinder.pose.orientation.w = q_marker.w();
  cylinder.pose.orientation.x = q_marker.x();
  cylinder.pose.orientation.y = q_marker.y();
  cylinder.pose.orientation.z = q_marker.z();

  cylinder.scale.x = radius * 2;
  cylinder.scale.y = radius * 2;
  cylinder.scale.z = length;

  float maxF = 400.0f, minF = 20.0f;
  if (chroma > maxF)
  {
    chroma = 1.0f;
    cylinder.color.r = chroma;
    cylinder.color.g = 0.0f;
    cylinder.color.b = 0.0f;
    cylinder.color.a = 1.0f;
  }
  else if (chroma < minF)
  {
    cylinder.color.r = 0.0f;
    cylinder.color.g = 1.0f;
    cylinder.color.b = 0.0f;
    cylinder.color.a = 1.0f;
  }
  else
  {
    chroma /= maxF;
    cylinder.color.r = chroma;
    cylinder.color.g = 1.0f - chroma;
    cylinder.color.b = 0.0f;
    cylinder.color.a = 1.0f;
  }

  return cylinder;
}

visualization_msgs::Marker Glove::genarrow(Eigen::Vector3d pt_marker,
                                           Eigen::Quaterniond q_marker,
                                           double length,
                                           double radius,
                                           double force,
                                           std::string ns)
{
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "glove_link";
  arrow.header.stamp = ros::Time::now();
  arrow.ns = ns.c_str();
  arrow.type = visualization_msgs::Marker::ARROW;

  arrow.pose.position.x = pt_marker[0];
  arrow.pose.position.y = pt_marker[1];
  arrow.pose.position.z = pt_marker[2];

  arrow.pose.orientation.w = q_marker.w();
  arrow.pose.orientation.x = q_marker.x();
  arrow.pose.orientation.y = q_marker.y();
  arrow.pose.orientation.z = q_marker.z();

  arrow.scale.x = radius / 5;
  arrow.scale.y = radius / 5;
  arrow.scale.z = length;

  arrow.color.r = 1.0f;
  arrow.color.g = 0.0f;
  arrow.color.b = 0.0f;
  arrow.color.a = 1.0f;

  return arrow;
}

void Glove::bad_cb(const std_msgs::StringPtr msg)
{
  ROS_INFO_STREAM("Bad data session");

  if (replay_)
  {
    bad_trigger_ = true;
    continue_ = false;
  }
}

void Glove::vicon_wrist_cb(const geometry_msgs::TransformStamped msg)
{
  if (replay_)
  {
    if (begin_ && continue_)
      bag.write("/vicon/wrist/wrist", ros::Time::now(), msg);
  }
  if (!vicon_wrist_)
    vicon_wrist_ = true;
  vicon_wrist_tf.setOrigin(tf::Vector3(
      msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z));
  vicon_wrist_tf.setRotation(tf::Quaternion(msg.transform.rotation.x,
                                            msg.transform.rotation.y,
                                            msg.transform.rotation.z,
                                            msg.transform.rotation.w));

  br_.sendTransform(tf::StampedTransform(vicon_wrist_tf, ros::Time::now(), "world", "glove_link"));
}

void Glove::vicon_bottle_cb(const geometry_msgs::TransformStamped msg)
{
  if (replay_)
  {
    if (begin_ && continue_)
      bag.write("/vicon/bottle64/bottle64", ros::Time::now(), msg);
  }
  if (!vicon_bottle_)
    vicon_bottle_ = true;
  vicon_bottle_tf.setOrigin(tf::Vector3(
      msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z));
  vicon_bottle_tf.setRotation(tf::Quaternion(msg.transform.rotation.x,
                                             msg.transform.rotation.y,
                                             msg.transform.rotation.z,
                                             msg.transform.rotation.w));

  if (!record_)
    br_.sendTransform(tf::StampedTransform(
        vicon_bottle_tf, ros::Time::now(), "world", "vicon/bottle64/bottle64"));
}

void Glove::vicon_lid_cb(const geometry_msgs::TransformStamped msg)
{
  if (replay_)
  {
    if (begin_ && continue_)
      bag.write("/vicon/bottle64_lid/bottle64_lid", ros::Time::now(), msg);
  }
  if (!vicon_lid_)
    vicon_lid_ = true;
  vicon_lid_tf.setOrigin(tf::Vector3(
      msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z));
  vicon_lid_tf.setRotation(tf::Quaternion(msg.transform.rotation.x,
                                          msg.transform.rotation.y,
                                          msg.transform.rotation.z,
                                          msg.transform.rotation.w));

  if (!record_)
    br_.sendTransform(tf::StampedTransform(
        vicon_lid_tf, ros::Time::now(), "world", "vicon/bottle64_lid/bottle64_lid"));
}

void Glove::cal_cb(const std_msgs::StringPtr msg)
{
  ROS_INFO_STREAM("Calibration triggered");

  if (replay_)
  {
    continue_ = false;
    bag.close();
    if (bad_trigger_)
      bad_trigger_ = false;
    else
      bag_counter_++;
    std::stringstream ss;
    ss << dest_dir_ << bag_counter_ << ".bag";
    bag.open(ss.str().c_str(), rosbag::bagmode::Write);
    if (!begin_)
      begin_ = true;
    bag.write("/fsr_glove_imutracker_imuctrl", ros::Time::now(), msg);
    continue_ = true;
  }

  if (global2sensors_.empty())
  {
    ROS_WARN_STREAM("No IMU data to calibrate to");
    return;
  }
  phalanges2sensors_.resize(global2sensors_.size());
  phalanges2sensors_zero_.resize(global2sensors_.size());

  Eigen::Quaterniond palm2sensor0;
  palm2sensor0 = Eigen::AngleAxisd(0.5 * Pi, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(-1.0 / 18 * Pi, Eigen::Vector3d::UnitX());  // changable
  Eigen::Quaterniond global2glove = palm2sensor0 * global2sensors_[0].inverse();
  Eigen::Quaterniond global2glove_zero = palm2sensor0 * global2sensors_zero_[0].inverse();
  global2glove = global2glove.inverse();
  global2glove_zero = global2glove_zero.inverse();

  for (int i = 0; i < global2sensors_.size(); i++)
  {
    phalanges2sensors_[i] = global2sensors_[i].inverse() * (global2glove * canonical_pose_[i]);
    phalanges2sensors_[i] = phalanges2sensors_[i].inverse();

    phalanges2sensors_zero_[i] =
        global2sensors_zero_[i].inverse() * (global2glove_zero * canonical_pose_[i]);
    phalanges2sensors_zero_[i] = phalanges2sensors_zero_[i].inverse();
  }
  calibrated_ = true;
}

void Glove::imu_cb(const geometry_msgs::PoseArrayPtr msg)
{
  if (replay_)
  {
    if (begin_ && continue_)
    {
      geometry_msgs::PoseArray rawposes;
      rawposes.poses.resize(msg->poses.size());
      rawposes.header.frame_id = "raw_poses_array";
      for (int i = 0; i < msg->poses.size(); i++)
      {
        rawposes.poses[i] = msg->poses[i];
      }
      bag.write("/fsr_glove_imutracker_raw", ros::Time::now(), rawposes);
    }
  }
  global2sensors_.resize(msg->poses.size());
  global2sensors_zero_.resize(msg->poses.size());
  for (int i = 0; i < msg->poses.size(); ++i)
  {
    geometry_msgs::Pose p = msg->poses[i];
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    global2sensors_[i] = q;
    global2sensors_zero_[i] = q;
    // global2sensors_zero_[i] = global2sensors_zero_[0].inverse()*global2sensors_zero_[i];
  }

  // if (calibrated_ && vicon_wrist_) {
  if (calibrated_)
  {
    publish_state();
  }
}

void Glove::imu_rel_cb(const geometry_msgs::PoseArrayPtr msg)
{
  if (replay_)
  {
    if (begin_ && continue_)
    {
      geometry_msgs::PoseArray relposes;
      relposes.poses.resize(msg->poses.size());
      relposes.header.frame_id = "rel_poses_array";
      for (int i = 0; i < msg->poses.size(); i++)
      {
        relposes.poses[i] = msg->poses[i];
      }
      bag.write("/fsr_glove_imutracker_rel", ros::Time::now(), relposes);
    }
  }
}

void Glove::force_cb(const fsr_glove::glovePtr msg)
{
  if (replay_)
  {
    if (begin_ && continue_)
    {
      bag.write("/force_msg", ros::Time::now(), msg);
    }
  }
  int count = 0;
  for (int i = 0; i < 5; i++)
  {
    forces_[count] = msg->segments[i].force[0];
    forces_[count + 1] = msg->segments[i].force[1];
    count += 2;
  }
  for (int i = 5; i < 9; i++)
  {
    forces_[count] = msg->segments[i].force[0];
    forces_[count + 1] = msg->segments[i].force[1];
    forces_[count + 2] = msg->segments[i].force[2];
    forces_[count + 3] = msg->segments[i].force[3];
    count += 4;
  }
}

void Glove::image_raw_cb(const sensor_msgs::Image msg)
{
  if (replay_)
  {
    if (begin_ && continue_)
      bag.write("/usb_cam/image_raw", ros::Time::now(), msg);
  }
}

void Glove::zero_rel_poses()
{
  if (replay_)
  {
    if (begin_ && continue_)
    {
      geometry_msgs::PoseArray relposes;
      relposes.poses.resize(global2phalanges_zero_.size());
      relposes.header.frame_id = "zero_rel_poses_array";
      for (int i = 0; i < global2phalanges_zero_.size(); i++)
      {
        relposes.poses[i].orientation.x = global2phalanges_zero_[i].x();
        relposes.poses[i].orientation.y = global2phalanges_zero_[i].y();
        relposes.poses[i].orientation.z = global2phalanges_zero_[i].z();
        relposes.poses[i].orientation.w = global2phalanges_zero_[i].w();
      }
      bag.write("/fsr_glove_imutracker_zero_rel", ros::Time::now(), relposes);
    }
  }
}

// hand size is not the real hand size to make the force orientational representation
void Glove::force_vector(tf::Transform glove_tf, std::string child_frame)
{
  if (replay_)
  {
    if (begin_ && continue_)
    {
      tf::StampedTransform glove_tfs(glove_tf, ros::Time::now(), "glove_link", child_frame);
      geometry_msgs::TransformStamped msg;
      tf::transformStampedTFToMsg(glove_tfs, msg);
      std::string topic_name = "/glove_link/" + child_frame;
      bag.write(topic_name.c_str(), ros::Time::now(), msg);
    }
  }
}

void Glove::publish_state_finger(int idx_from_, int idx_to_)
{
  if (idx_to_ >= global2phalanges_.size())
    return;

  tf::Transform transform;
  Eigen::Vector3d pt_link(0.0, 0.0, 0.0);
  for (int i = idx_from_; i <= idx_to_; ++i)
  {
    pt_link += quaternion_rotate(global2phalanges_[parents_[i]], canonical_origin_[i]);
    transform.setOrigin(tf::Vector3(pt_link[0], pt_link[1], pt_link[2]));
    Eigen::Quaterniond q_link = global2phalanges_[i];
    transform.setRotation(tf::Quaternion(q_link.x(), q_link.y(), q_link.z(), q_link.w()));
    // transform*=vicon_wrist_tf;

    // marker
    Eigen::Quaterniond q_glove(transform.getRotation().w(),
                               transform.getRotation().x(),
                               transform.getRotation().y(),
                               transform.getRotation().z());
    Eigen::Vector3d pt_glove(
        transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

    Eigen::Quaterniond q_marker = q_glove * Eigen::AngleAxisd(0.5 * Pi, Eigen::Vector3d::UnitY());
    Eigen::Vector3d pt_marker =
        pt_glove + quaternion_rotate(q_marker, Eigen::Vector3d(0.0, 0.0, link_lengths_[i] / 2));
    std::string ns = "fsr_glove_marker" + link_names_[i];
    float force = ((i == idx_from_) ? forces_[idx_from_ * 2 / 3] : forces_[idx_from_ * 2 / 3 + 1]);
    visualization_msgs::Marker mk =
        genmark(pt_marker, q_marker, link_lengths_[i], radius, force, ns);

    float factor = 10000;
    Eigen::Quaterniond q_arrow = q_marker * Eigen::AngleAxisd(-0.5 * Pi, Eigen::Vector3d::UnitY());
    Eigen::Vector3d pt_arrow =
        pt_marker + quaternion_rotate(q_arrow, Eigen::Vector3d(0, 0, -radius - force / factor));
    ns = "fsr_glove_arrow" + link_names_[i];
    visualization_msgs::Marker ar = genarrow(pt_arrow, q_arrow, force / factor, radius, force, ns);

    // br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), link_names_[parents_[i]],
    // link_names_[i]));
    br_.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "glove_link", link_names_[i]));
    marker_pub_.publish(mk);
    arrow_pub_.publish(ar);

    if (!record_ && replay_)
    {
      tf::Quaternion q_finger(q_glove.x(), q_glove.y(), q_glove.z(), q_glove.w());
      tf::Vector3 pt_finger(pt_marker[0], pt_marker[1], pt_marker[2]);
      force_vector(tf::Transform(q_finger, pt_finger), link_names_[i]);
    }
  }
}

void Glove::publish_state_palm()
{
  Eigen::Quaterniond q_palm = global2phalanges_[0];
  tf::Transform palm_tf;
  palm_tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  palm_tf.setRotation(tf::Quaternion(q_palm.x(), q_palm.y(), q_palm.z(), q_palm.w()));
  br_.sendTransform(tf::StampedTransform(palm_tf, ros::Time::now(), "glove_link", link_names_[0]));

  double X_center = arrayLength / 2 - arrayLength / 8;
  double Y_center = -(arrayLength / 2 - arrayLength / 8);

  // translation with respect to world frame
  int count = 0;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      Eigen::Quaterniond q_glove(palm_tf.getRotation().w(),
                                 palm_tf.getRotation().x(),
                                 palm_tf.getRotation().y(),
                                 palm_tf.getRotation().z());
      Eigen::Vector3d pt_glove(
          palm_tf.getOrigin().getX(), palm_tf.getOrigin().getY(), palm_tf.getOrigin().getZ());

      Eigen::Vector3d pt_marker =
          pt_glove + quaternion_rotate(q_glove, Eigen::Vector3d(X_center, Y_center, 0.0));
      Eigen::Quaterniond q_marker = q_glove;
      std::stringstream ss;
      ss << "fsr_glove_marker" << i << "&" << j;
      float force = forces_[count + 10];
      visualization_msgs::Marker mk =
          genmark(pt_marker, q_marker, PalmHeight, arrayLength / 8, force, ss.str());

      float factor = 10000;
      Eigen::Quaterniond q_arrow = q_marker;
      Eigen::Vector3d pt_arrow =
          pt_marker +
          quaternion_rotate(q_arrow, Eigen::Vector3d(0, 0, -PalmHeight / 2 - force / factor));
      ss.clear();
      ss.str("");
      ss << "fsr_glove_arrow" << i << "&" << j;
      visualization_msgs::Marker ar =
          genarrow(pt_arrow, q_arrow, force / factor, radius, force, ss.str());

      marker_pub_.publish(mk);
      arrow_pub_.publish(ar);

      Y_center += arrayLength / 4;
      count++;

      if (!record_ && replay_)
      {
        std::stringstream ss;
        ss << link_names_[0] << "_" << i << "_" << j;
        tf::Quaternion q_grid(q_glove.x(), q_glove.y(), q_glove.z(), q_glove.w());
        tf::Vector3 pt_grid(pt_marker[0], pt_marker[1], pt_marker[2]);
        force_vector(tf::Transform(q_grid, pt_grid), ss.str());
      }
    }
    X_center -= arrayLength / 4;
    Y_center = -(arrayLength / 2 - arrayLength / 8);
  }
}

void Glove::publish_state()
{
  global2phalanges_.resize(global2sensors_.size());
  global2phalanges_zero_.resize(global2sensors_.size());

  Eigen::Quaterniond palm2sensor0;
  palm2sensor0 = Eigen::AngleAxisd(0.5 * Pi, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(-1.0 / 18 * Pi, Eigen::Vector3d::UnitX());
  for (int i = 0; i < global2sensors_.size(); i++)
  {
    global2sensors_zero_[i] = global2sensors_zero_[i] * global2sensors_zero_[0].inverse();
    global2sensors_zero_[i] = global2sensors_zero_[i].inverse();
    global2sensors_zero_[i] = palm2sensor0 * global2sensors_zero_[i];
  }

  for (int i = 0; i < global2sensors_.size(); i++)
  {
    global2phalanges_[i] = global2sensors_[i] * phalanges2sensors_[i].inverse();
    global2phalanges_zero_[i] = global2sensors_zero_[i] * phalanges2sensors_zero_[i].inverse();
  }
  // palm
  publish_state_palm();
  // thumb
  publish_state_finger(1, 2);
  // index
  publish_state_finger(3, 5);
  // middle
  publish_state_finger(6, 8);
  // ring
  publish_state_finger(9, 11);
  // pinkie
  publish_state_finger(12, 14);

  if (!record_ && replay_)
    zero_rel_poses();
  // ros::spinOnce();
}
