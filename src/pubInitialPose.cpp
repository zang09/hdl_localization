#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf/transform_broadcaster.h>
#include <XmlRpcException.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelUtmPosition.h>
#include <hdl_localization/initGPS.h>

#include <../include/hdl_localization/utm.h>

using namespace hdl_localization;

class initialPose
{
public:
  ros::Subscriber subGlobalMap_;
  ros::Subscriber subCorrectIMU_;
  ros::Subscriber subGPS_;
  ros::Subscriber subBestUtm_;

  ros::Publisher  pubInitialPose_;
  ros::Publisher  pubFilteredCloud_;

  ros::ServiceServer getInitGPSServiceServer_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap_;
  geometry_msgs::Quaternion robotOrientation_;

  initGPSRequest initRequest_;
  initGPSResponse initResponse_;

  int    gpsCnt_;
  bool   initFlag_;
  double robotPosX_;
  double robotPosY_;
  double robotPosZ_;
  double preRobotPosX_;
  double preRobotPosY_;

  initialPose(ros::NodeHandle nh, ros::NodeHandle priv_nh) :
    gpsCnt_(0),
    initFlag_(false),
    robotPosX_(0.0),
    robotPosY_(0.0),
    robotPosZ_(0.0),
    preRobotPosX_(0.0),
    preRobotPosY_(0.0)
  {
    subGlobalMap_  = nh.subscribe("globalmap", 1, &initialPose::globalmapHandler, this);
    subGPS_        = nh.subscribe<sensor_msgs::NavSatFix>("pwk7/gps/fix", 200, &initialPose::gpsHandler, this, ros::TransportHints().tcpNoDelay());
    //subCorrectIMU_ = nh.subscribe<sensor_msgs::Imu>("imu_correct", 2000, &initialPose::imuHandler, this, ros::TransportHints().tcpNoDelay());
    //subBestUtm_    = nh.subscribe<novatel_gps_msgs::NovatelUtmPosition>("pwk7/bestutm", 200, &initialPose::utmHandler, this, ros::TransportHints().tcpNoDelay());

    pubInitialPose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    pubFilteredCloud_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 5, true); //test
    getInitGPSServiceServer_ = nh.advertiseService("local/gps", &initialPose::getInitGPSService, this);

    if (priv_nh.hasParam("map_origin"))
    {
      XmlRpc::XmlRpcValue mapConfig;

      try
      {
        priv_nh.getParam("map_origin", mapConfig);
        ROS_INFO("get map origin!");

        initRequest_.lat = mapConfig[0];
        initRequest_.lon = mapConfig[1];

        //printf("%lf, %lf\n", request_.lat, request_.lon);
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() <<
                         " for process_noise_covariance (type: " << mapConfig.getType() << ")");
      }
    }
  }

  void globalmapHandler(const sensor_msgs::PointCloud2ConstPtr& pointsMsg) {
    //ROS_INFO("globalmap received(pubInitialPose)!");
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pointsMsg, *mapCloud);

    globalMap_ = mapCloud;
  }

  void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
  {
    double lat = gpsMsg->latitude;
    double lon = gpsMsg->longitude;

    int zone = 52;
    Utm::LatLonToUTMXY(lat, lon, zone, robotPosX_, robotPosY_);

    if(gpsCnt_ == 5)
    {
        double dX = robotPosX_ - preRobotPosX_;
        double dY = robotPosY_ - preRobotPosY_;

        tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, atan2(dY, dX));
        robotOrientation_.w = q.w();
        robotOrientation_.x = q.x();
        robotOrientation_.y = q.y();
        robotOrientation_.z = q.z();

        gpsCnt_ = 0;
    }

    preRobotPosX_ = robotPosX_;
    preRobotPosY_ = robotPosY_;

    gpsCnt_++;
  }

  /*
  void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
  {
    robotOrientation_ = imuMsg->orientation;
  }
  */

  /*
  void utmHandler(const novatel_gps_msgs::NovatelUtmPosition::ConstPtr& utmMsg)
  {
    robotPosX_ = utmMsg->easting;
    robotPosY_ = utmMsg->northing;

    float x_error = utmMsg->easting_sigma;
    float y_error = utmMsg->northing_sigma;

    float error = sqrt(x_error*x_error + y_error*y_error) * 100.;
    printf("error: %lf\n", error);

    //Service Call
    if(!initFlag_ && error < 50.0) {
      getInitGPSService(initRequest_, initResponse_);
      initFlag_ = true;
    }
  }
  */

  bool getInitGPSService(initGPSRequest& req, initGPSResponse& res)
  {
    double mapLat = req.lat;
    double mapLon = req.lon;

    double mapX, mapY;
    int zone = 52;

    Utm::LatLonToUTMXY(mapLat, mapLon, zone, mapX, mapY);

    geometry_msgs::PoseWithCovarianceStamped poseMsg;

    double currentX = robotPosX_ - mapX;
    double currentY = robotPosY_ - mapY;
    double currentZ = 0;

    getHeightFromCloud(currentX, currentY, currentZ);

    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = "map";
    poseMsg.pose.pose.position.x = currentX;
    poseMsg.pose.pose.position.y = currentY;
    poseMsg.pose.pose.position.z = currentZ;
    poseMsg.pose.pose.orientation = robotOrientation_;

    pubInitialPose_.publish(poseMsg);

    res.success = true;
    return true;
  }

  void getHeightFromCloud(double x, double y, double &z)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> passX;
    passX.setInputCloud(globalMap_);
    passX.setFilterFieldName("x");
    passX.setFilterLimits(x-5.0, x+5.0);
    passX.filter(*cloudFiltered);

    pcl::PassThrough<pcl::PointXYZI> passY;
    passY.setInputCloud(cloudFiltered);
    passY.setFilterFieldName("y");
    passY.setFilterLimits(y-5.0, y+5.0);
    passY.filter(*cloudFiltered);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);          //(옵션) // Enable model coefficient refinement (optional).
    seg.setInputCloud(cloudFiltered);           //입력
    seg.setModelType(pcl::SACMODEL_PLANE);      //적용 모델  // Configure the object to look for a plane.
    seg.setMethodType(pcl::SAC_RANSAC);         //적용 방법   // Use RANSAC method.
    seg.setMaxIterations(1000);                 //최대 실행 수
    seg.setDistanceThreshold(0.2);              //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
    seg.segment (*inliers, *coefficients);      //세그멘테이션 적용

    pcl::copyPointCloud<pcl::PointXYZI>(*cloudFiltered, *inliers, *cloudFiltered);

    double sumZ = 0;
    double averageZ = 0;
    for(size_t i=0; i<cloudFiltered->size(); i++) {
      sumZ += cloudFiltered->points[i].z;
    }
    averageZ = sumZ/cloudFiltered->size();
    z = averageZ;

    pubFilteredCloud_.publish(cloudFiltered);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pubInitialPose");

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_("~");

  ROS_INFO("\033[1;32m----> Initial Pose node started.\033[0m");

  initialPose IP(nh_, private_nh_);

  ros::spin();

  return 0;
}
