#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <hdl_localization/initGPS.h>

#include <../include/hdl_localization/utm.h>

using namespace hdl_localization;

class initialPose
{
public:
  ros::NodeHandle nh_;

  ros::Subscriber subCorrectIMU_;
  ros::Subscriber subGPS_;
  ros::Subscriber subEkfGPS_;
  ros::Publisher  pubInitialPose_;
  ros::ServiceServer getInitGPSServiceServer_;

  geometry_msgs::Quaternion imu_quaternion_;
  geometry_msgs::Point      gps_position_;
  geometry_msgs::Quaternion gps_quaternion_;

  bool first_run_ = true;

  double altitude_ = 0.0;
  double latitude_ = 0.0;
  double longitude_ = 0.0;

  double init_altitude_ = 0.0;
  double init_latitude_ = 0.0;
  double init_longitude_ = 0.0;

  double pos_x_ = 0.0;
  double pos_y_ = 0.0;
  double pos_z_ = 0.0;

  initialPose()
  {
    subCorrectIMU_ = nh_.subscribe<sensor_msgs::Imu>("imu_correct", 2000, &initialPose::imuHandler, this, ros::TransportHints().tcpNoDelay());
    subGPS_        = nh_.subscribe<sensor_msgs::NavSatFix> ("pwk7/gps/fix", 200, &initialPose::gpsHandler, this, ros::TransportHints().tcpNoDelay());
    //subEkfGPS_     = nh_.subscribe<nav_msgs::Odometry>("odometry/gps", 200, &initialPose::gpsOdomHandler, this, ros::TransportHints().tcpNoDelay());

    pubInitialPose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    getInitGPSServiceServer_ = nh_.advertiseService("local/gps", &initialPose::getInitGPSService, this);
  }

//  void gpsOdomHandler(const nav_msgs::Odometry::ConstPtr& gpsOdomMsg)
//  {
//    gps_position_   = gpsOdomMsg->pose.pose.position;
//    gps_quaternion_ = gpsOdomMsg->pose.pose.orientation;
//  }

  void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
  {
    latitude_ = gpsMsg->latitude;
    longitude_ = gpsMsg->longitude;
    altitude_ = gpsMsg->altitude;

    int zone = 52;
    Utm::LatLonToUTMXY(latitude_, longitude_, zone, pos_x_, pos_y_);
    pos_z_ = altitude_;
  }

  void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
  {
    imu_quaternion_ = imuMsg->orientation;
  }

  bool getInitGPSService(initGPSRequest& req, initGPSResponse& res)
  {
    init_latitude_ = req.lat;
    init_longitude_ = req.lon;
    init_altitude_ = req.alt;

    int zone = 52;
    double pos_x, pos_y, pos_z;
    Utm::LatLonToUTMXY(init_latitude_, init_longitude_, zone, pos_x, pos_y);
    pos_z = init_altitude_;

    geometry_msgs::PoseWithCovarianceStamped poseMsg;

    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = "map";
    poseMsg.pose.pose.position.x = pos_x_ - pos_x;
    poseMsg.pose.pose.position.y = pos_y_ - pos_y;
    poseMsg.pose.pose.position.z = pos_z_ - pos_z;
    poseMsg.pose.pose.orientation = imu_quaternion_;

    pubInitialPose_.publish(poseMsg);

    res.success = true;
    return true;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "initial_pose");

  initialPose IP;

  ROS_INFO("\033[1;32m----> Initial Pose node started.\033[0m");

  ros::spin();

  return 0;
}
