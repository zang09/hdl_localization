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
#include <nav_msgs/Odometry.h>
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
    ros::Publisher  pubGPSPos_;

    ros::ServiceServer getInitGPSServiceServer_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap_;
    geometry_msgs::Quaternion robotOrientation_;
    geometry_msgs::Quaternion imuOrientation_;

    initGPSRequest initRequest_;
    initGPSResponse initResponse_;

    int    gpsCnt_;
    bool   initFlag_;
    bool   useGPS_;
    double mapX_;
    double mapY_;
    double robotPosX_;
    double robotPosY_;
    double robotPosZ_;
    double preRobotPosX_;
    double preRobotPosY_;

    initialPose(ros::NodeHandle nh, ros::NodeHandle priv_nh) :
        gpsCnt_(0),
        initFlag_(false),
        useGPS_(false),
        mapX_(0.0),
        mapY_(0.0),
        robotPosX_(0.0),
        robotPosY_(0.0),
        robotPosZ_(0.0),
        preRobotPosX_(0.0),
        preRobotPosY_(0.0)
    {
        subGlobalMap_     = nh.subscribe("globalmap", 1, &initialPose::globalmapHandler, this);
        subGPS_           = nh.subscribe<sensor_msgs::NavSatFix>("pwk7/gps/fix", 200, &initialPose::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subCorrectIMU_    = nh.subscribe<sensor_msgs::Imu>("correct_imu", 200, &initialPose::imuHandler, this, ros::TransportHints().tcpNoDelay());
        pubInitialPose_   = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
        pubFilteredCloud_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 5, true); //test
        pubGPSPos_        = nh.advertise<nav_msgs::Odometry>("gps_test_odom", 10);
        getInitGPSServiceServer_ = nh.advertiseService("local/gps", &initialPose::getInitGPSService, this);

        if (priv_nh.hasParam("map_origin"))
        {
            XmlRpc::XmlRpcValue mapConfig;

            try
            {
                priv_nh.getParam("map_origin", mapConfig);
                ROS_INFO("\033[1;32m----> Get map origin(initialpose).\033[0m");

                int zone = 52;
                double lat = mapConfig[0];
                double lon = mapConfig[1];

                Utm::LatLonToUTMXY(lat, lon, zone, mapX_, mapY_);
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

        double dX = robotPosX_ - preRobotPosX_;
        double dY = robotPosY_ - preRobotPosY_;

        double distance = sqrt(pow(dX,2)+pow(dY,2));

        if(distance > 0.1 && distance < 10.0)
        {
            tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, atan2(dY, dX));
            robotOrientation_.w = q.w();
            robotOrientation_.x = q.x();
            robotOrientation_.y = q.y();
            robotOrientation_.z = q.z();

            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "map";

            odom.pose.pose.position.x = robotPosX_ - mapX_;
            odom.pose.pose.position.y = robotPosY_ - mapY_;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = robotOrientation_;

            initFlag_ = true;
            pubGPSPos_.publish(odom);
        }

        preRobotPosX_ = robotPosX_;
        preRobotPosY_ = robotPosY_;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        imuOrientation_ = imuMsg->orientation;
    }

    bool getInitGPSService(initGPSRequest& req, initGPSResponse& res)
    {
        double mapLat = req.lat;
        double mapLon = req.lon;
        useGPS_ = req.gps;

        if(useGPS_)
        {
            if((mapX_ == 0.) && (mapY_ == 0.))
            {
                int zone = 52;
                Utm::LatLonToUTMXY(mapLat, mapLon, zone, mapX_, mapY_);
            }

            if(initFlag_)
            {
                geometry_msgs::PoseWithCovarianceStamped poseMsg;

                double tempX = robotPosX_ - mapX_;
                double tempY = robotPosY_ - mapY_;
                double currentZ = 0;

                getHeightFromCloud(tempX, tempY, currentZ);

                poseMsg.header.stamp = ros::Time::now();
                poseMsg.header.frame_id = "map";
                poseMsg.pose.pose.position.x = robotPosX_ - mapX_;
                poseMsg.pose.pose.position.y = robotPosY_ - mapY_;
                poseMsg.pose.pose.position.z = currentZ;
                poseMsg.pose.pose.orientation = robotOrientation_;

                pubInitialPose_.publish(poseMsg);
            }
            else {
                ROS_ERROR("No orientation now!");
            }
        }
        else
        {
            geometry_msgs::PoseWithCovarianceStamped poseMsg;

            poseMsg.header.stamp = ros::Time::now();
            poseMsg.header.frame_id = "map";
            poseMsg.pose.pose.position.x = 0;
            poseMsg.pose.pose.position.y = 0;
            poseMsg.pose.pose.position.z = 0;
            poseMsg.pose.pose.orientation = imuOrientation_;

            pubInitialPose_.publish(poseMsg);
        }

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
        double minZ = cloudFiltered->points[0].z;
        double maxZ = cloudFiltered->points[0].z;

        for(size_t i=0; i<cloudFiltered->size(); i++)
        {
            double curZ = cloudFiltered->points[i].z;
            sumZ += curZ;
            if(minZ > curZ) minZ = curZ;
            if(maxZ < curZ) maxZ = curZ;
        }
        averageZ = sumZ/cloudFiltered->size();

        std::cout << "diff: " << maxZ-minZ << std::endl;
        if(maxZ - minZ > 5.0)
            z = minZ;
        else
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
