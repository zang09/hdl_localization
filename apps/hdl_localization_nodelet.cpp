#include <mutex>
#include <memory>
#include <iostream>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <hdl_localization/pose_estimator.hpp>

using namespace std;

namespace hdl_localization {

class HdlLocalizationNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNodelet() {
  }
  virtual ~HdlLocalizationNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    processing_time.resize(16);
    initialize_params();

    use_imu      = private_nh.param<bool>("use_imu", true);
    invert_imu   = private_nh.param<bool>("invert_imu", false);
    imu_topic    = private_nh.param<std::string>("imu_topic", "/gpsimu_driver/imu_data");
    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");

    if(use_imu) {
      NODELET_INFO("\033[0;33m----> enable imu-based prediction\033[0m");
      imu_sub = mt_nh.subscribe(imu_topic, 256, &HdlLocalizationNodelet::imu_callback, this);
    }
    points_sub      = mt_nh.subscribe(points_topic, 5, &HdlLocalizationNodelet::points_callback, this);
    globalmap_sub   = nh.subscribe("hdl_localization/globalmap", 1, &HdlLocalizationNodelet::globalmap_callback, this);
    initialpose_sub = nh.subscribe("/initialpose", 8, &HdlLocalizationNodelet::initialpose_callback, this);
    local_flag_sub  = nh.subscribe("hdl_localization/localization_flag", 1, &HdlLocalizationNodelet::local_flag_callback, this);

    initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, false);
    pose_pub        = nh.advertise<nav_msgs::Odometry>("hdl_localization/odom", 5, false);
    alignState_pub  = nh.advertise<std_msgs::Bool>("hdl_localization/align_state", 1, true);
    aligned_pub     = nh.advertise<sensor_msgs::PointCloud2>("hdl_localization/aligned_points", 5, false);
    velodyne_pub    = nh.advertise<sensor_msgs::PointCloud2>("hdl_localization/velodyne_points", 5, false);
    correctImu_pub  = nh.advertise<sensor_msgs::Imu>("hdl_localization/imu_correct", 5, false);
  }

private:
  void initialize_params() {
    // initialize scan matching method
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    std::string ndt_neighbor_search_method = private_nh.param<std::string>("ndt_neighbor_search_method", "DIRECT7");

    double ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());

    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(ndt_resolution);
    if(ndt_neighbor_search_method == "DIRECT1") {
      NODELET_INFO("\033[0;33m----> search_method DIRECT1 is selected\033[0m");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      registration = ndt;
    } else if(ndt_neighbor_search_method == "DIRECT7") {
      NODELET_INFO("\033[0;33m----> search_method DIRECT7 is selected\033[0m");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      registration = ndt;
    } else if(ndt_neighbor_search_method == "GICP_OMP"){
      NODELET_INFO("\033[0;33m----> search_method GICP_OMP is selected\033[0m");
      registration = gicp;
    }
    else {
      if(ndt_neighbor_search_method == "KDTREE") {
        NODELET_INFO("\033[0;33m----> search_method KDTREE is selected\033[0m");
      }
      else {
        NODELET_WARN("invalid search method was given");
        NODELET_WARN("default method is selected (KDTREE)");
      }
      ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      registration = ndt;
    }

    // initialize pose estimator
    if(private_nh.param<bool>("specify_init_pose", true)) {
      NODELET_INFO("\033[0;33m----> initialize pose estimator with specified parameters!!\033[0m");
      pose_estimator.reset(new hdl_localization::PoseEstimator(registration,
                                                               ros::Time::now(),
                                                               Eigen::Vector3f(private_nh.param<double>("init_pos_x", 0.0), private_nh.param<double>("init_pos_y", 0.0), private_nh.param<double>("init_pos_z", 0.0)),
                                                               Eigen::Quaternionf(private_nh.param<double>("init_ori_w", 1.0), private_nh.param<double>("init_ori_x", 0.0), private_nh.param<double>("init_ori_y", 0.0), private_nh.param<double>("init_ori_z", 0.0)),
                                                               private_nh.param<double>("cool_time_duration", 0.5)
                                                               ));
    }

    // initialize params
    private_nh.param<vector<double>>("/hdl_localization/extrinsicRot", extRotV, vector<double>());
    private_nh.param<vector<double>>("/hdl_localization/extrinsicRPY", extRPYV, vector<double>());
    private_nh.param<vector<double>>("/hdl_localization/extrinsicTrans", extTransV, vector<double>());
    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
    extQRPY = Eigen::Quaterniond(extRPY);

    alignFlag.data = false;
    local_state = true;
    failCnt   = 0;
  }

private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);

    //pub correct imu
    sensor_msgs::Imu thisImu = imuConverter(*imu_msg);
    thisImu.header.frame_id = "base_link";
    //correctImu_pub.publish(thisImu);
  }

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      //NODELET_ERROR("waiting for initial pose input!!");
      return;
    }

    if(!globalmap) {
      NODELET_ERROR("globalmap has not been received!!");
      return;
    }

    // get timestamp
    cloud_header = points_msg->header;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);

    if(cloud->empty()) {
      NODELET_ERROR("cloud is empty!!");
      return;
    }

    // transform pointcloud into odom_child_frame_id
    //    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    //    if(!pcl_ros::transformPointCloud("velodyne", *pcl_cloud, *cloud, this->tf_listener)) {
    //      NODELET_ERROR("point cloud cannot be transformed into target frame!!");
    //      return;
    //    }

    auto filtered = downsample(cloud);

    // Set localization off
    if (local_state == false)
    {
      cloud->header.frame_id = "base_link";
      aligned_pub.publish(cloud);

      publish_odometry(cloud_header.stamp, pre_odom);
      return;
    }
    // Set re-localization
    else if (local_state == 2)
    {
      geometry_msgs::PoseWithCovarianceStamped relocal_pose;
      relocal_pose.pose.pose = pre_odom.pose.pose;
      relocal_pose.header.frame_id = "map";
      relocal_pose.header.stamp = ros::Time::now();

      initialpose_pub.publish(relocal_pose);
      local_state = true;
      return;
    }

    // predict
    if(!use_imu) {
      pose_estimator->predict(cloud_header.stamp, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      for(imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if(cloud_header.stamp < (*imu_iter)->header.stamp) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double gyro_sign = invert_imu ? -1.0 : 1.0;
        pose_estimator->predict((*imu_iter)->header.stamp, Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // correct
    double score;
    auto t1 = ros::Time::now();
    auto aligned = pose_estimator->correct(filtered, score);
    auto t2 = ros::Time::now();

    processing_time.push_back((t2 - t1).toSec());
    double avg_processing_time = std::accumulate(processing_time.begin(), processing_time.end(), 0.0) / processing_time.size();
    //NODELET_INFO_STREAM("processing_time: " << avg_processing_time * 1000.0 << "[msec]");
    //NODELET_INFO_STREAM("Fitness score: " << score << "\n");

    if(score < 0.2) {
      if(!alignFlag.data) {
        failCnt = 0;
        alignFlag.data = true;
        NODELET_INFO_STREAM("Correct align!");
      }
    }
    else {
      failCnt++;

      if(failCnt == 3) {
        alignFlag.data = false;
        NODELET_INFO_STREAM("Fail align!");
      }
    }

    if(aligned_pub.getNumSubscribers() || velodyne_pub.getNumSubscribers())
    {
      aligned->header.frame_id = "odom";
      aligned->header.stamp = cloud->header.stamp;
      aligned_pub.publish(aligned);
      alignState_pub.publish(alignFlag);

      cloud->header.frame_id = "base_link";
      velodyne_pub.publish(cloud);
    }

    publish_odometry(cloud_header.stamp, pose_estimator->matrix());
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    NODELET_INFO("\033[0;33m----> globalmap received!\033[0m");
    pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *map_cloud);
    globalmap = map_cloud;

    registration->setInputTarget(globalmap);
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    NODELET_INFO("initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(
          new hdl_localization::PoseEstimator(
            registration,
            cloud_header.stamp,
            Eigen::Vector3f(p.x, p.y, p.z),
            Eigen::Quaternionf(q.w, q.x, q.y, q.z),
            private_nh.param<double>("cool_time_duration", 0.5))
          );
  }

  void local_flag_callback(const std_msgs::BoolConstPtr& msg) {
    NODELET_INFO("localization flag received!!");
    bool flag = !msg->data;

    //0: false, 1: true, 2: relocal
    if (!local_state && flag)
      local_state = 2;
    else if (local_state && !flag)
      local_state = 0;
  }

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf

    //    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, "map", odom_child_frame_id);
    //    pose_broadcaster.sendTransform(odom_trans);

    static tf::TransformBroadcaster tfMap2Odom;
    static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, stamp, "map", "odom"));

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";

    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_quat;

    //odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub.publish(odom);
    pre_odom = odom;

    static tf::TransformBroadcaster tfOdom2BaseLink;
    tf::Transform tCur;
    tf::poseMsgToTF(odom.pose.pose, tCur);
    tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, stamp, "odom", "base_link");
    tfOdom2BaseLink.sendTransform(odom_2_baselink);
  }

  void publish_odometry(const ros::Time& stamp, const nav_msgs::Odometry &pose) {
    // broadcast the transform over tf
    static tf::TransformBroadcaster tfMap2Odom;
    static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, stamp, "map", "odom"));

    // publish the transform
    nav_msgs::Odometry odom;
    odom = pose;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";

    pose_pub.publish(odom);
    pre_odom = odom;

    static tf::TransformBroadcaster tfOdom2BaseLink;
    tf::Transform tCur;
    tf::poseMsgToTF(odom.pose.pose, tCur);
    tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, stamp, "odom", "base_link");
    tfOdom2BaseLink.sendTransform(odom_2_baselink);
  }

  /**
   * @brief convert a Eigen::Matrix to TransformedStamped
   * @param stamp           timestamp
   * @param pose            pose matrix
   * @param frame_id        frame_id
   * @param child_frame_id  child_frame_id
   * @return transform
   */
  geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
  }

  sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
  {
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();

    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    {
      ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
      ros::shutdown();
    }

    return imu_out;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  std::string odom_child_frame_id;

  bool use_imu;
  bool invert_imu;
  std::string imu_topic;
  std::string points_topic;

  ros::Subscriber imu_sub;
  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber initialpose_sub;
  ros::Subscriber local_flag_sub;

  ros::Publisher pose_pub;
  ros::Publisher initialpose_pub;
  ros::Publisher alignState_pub;
  ros::Publisher aligned_pub;
  ros::Publisher velodyne_pub;
  ros::Publisher correctImu_pub;

  tf::TransformBroadcaster pose_broadcaster;
  tf::TransformListener tf_listener;

  std_msgs::Header cloud_header;
  nav_msgs::Odometry pre_odom;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::ImuConstPtr> imu_data;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;
  // processing time buffer
  boost::circular_buffer<double> processing_time;

  // IMU
  vector<double> extRotV;
  vector<double> extRPYV;
  vector<double> extTransV;
  Eigen::Matrix3d extRot;
  Eigen::Matrix3d extRPY;
  Eigen::Vector3d extTrans;
  Eigen::Quaterniond extQRPY;

  // Flag  
  std_msgs::Bool alignFlag;
  int  local_state;
  int  failCnt;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)
