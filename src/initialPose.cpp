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
#include <robot_localization/navsat_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <hdl_localization/initGPS.h>

using namespace hdl_localization;

class initialPose
{
public:
    ros::Subscriber subGlobalMap_;
    ros::Subscriber subCorrectIMU_;
    ros::Subscriber subGPS_;
    ros::Subscriber subMapLLA_;
    ros::Subscriber subBestUtm_;

    ros::Publisher  pubInitialPose_;
    ros::Publisher  pubFilteredCloud_;
    ros::Publisher  pubGPSPos_;

    ros::ServiceServer getInitGPSServiceServer_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap_;

    geometry_msgs::Quaternion robotOrientation_;
    geometry_msgs::Quaternion imuOrientation_;
    nav_msgs::Odometry gps_odom_;

    initGPSRequest initRequest_;
    initGPSResponse initResponse_;

    int    gpsCnt_;
    bool   initFlag_;
    bool   useGPS_;
    double mapX_;
    double mapY_;
    double mapZ_;
    double robotPosX_;
    double robotPosY_;
    double robotPosZ_;
    double preRobotPosX_;
    double preRobotPosY_;
    double utm_meridian_convergence_;

    initialPose(ros::NodeHandle nh, ros::NodeHandle priv_nh) :
        gpsCnt_(0),
        initFlag_(false),
        useGPS_(false),
        mapX_(0.0),
        mapY_(0.0),
        mapZ_(0.0),
        robotPosX_(0.0),
        robotPosY_(0.0),
        robotPosZ_(0.0),
        preRobotPosX_(0.0),
        preRobotPosY_(0.0)
    {
        subGlobalMap_     = nh.subscribe("hdl_localization/globalmap", 1, &initialPose::globalmapHandler, this);
        subGPS_           = nh.subscribe<sensor_msgs::NavSatFix>("pwk7/gps/fix", 200, &initialPose::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subMapLLA_        = nh.subscribe<sensor_msgs::NavSatFix>("hdl_localization/gloablmap/origin", 5, &initialPose::mapOriginHandler, this);
        subCorrectIMU_    = nh.subscribe<sensor_msgs::Imu>("hdl_localization/correct_imu", 200, &initialPose::imuHandler, this, ros::TransportHints().tcpNoDelay());

        pubInitialPose_   = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        pubFilteredCloud_ = nh.advertise<sensor_msgs::PointCloud2>("hdl_localization/filtered_cloud", 5, true); //test
        pubGPSPos_        = nh.advertise<nav_msgs::Odometry>("hdl_localization/gps_odom", 10);

        getInitGPSServiceServer_ = nh.advertiseService("hdl_localization/init_gps", &initialPose::getInitGPSService, this);

        /*
        if (priv_nh.hasParam("map_lla"))
        {
            XmlRpc::XmlRpcValue mapConfig;

            try
            {
                priv_nh.getParam("map_lla", mapConfig);
                ROS_INFO("\033[1;32m----> Get map origin(initialpose).\033[0m");

                double lat = mapConfig[0];
                double lon = mapConfig[1];
                double alt = mapConfig[2];

                std::string utm_zone_tmp;
                RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, mapY_, mapX_, utm_zone_tmp, utm_meridian_convergence_);
                utm_meridian_convergence_ = DEG2RAD(utm_meridian_convergence_);

                mapZ_ = alt;
            }
            catch (XmlRpc::XmlRpcException &e)
            {
                ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() <<
                                 " for process_noise_covariance (type: " << mapConfig.getType() << ")");
            }
        }
        */
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
        robotPosZ_ = gpsMsg->altitude;

        std::string utm_zone_tmp;
        RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, robotPosY_, robotPosX_, utm_zone_tmp);

        robotPosX_ -= mapX_;
        robotPosY_ -= mapY_;
        robotPosZ_ -= mapZ_;

        double dX = robotPosX_ - preRobotPosX_;
        double dY = robotPosY_ - preRobotPosY_;
        double distance = sqrt(pow(dX,2)+pow(dY,2));

        if(distance > 0.1 && distance < 10.0)
        {
            tf2::Transform latest_utm_pose;
            latest_utm_pose.setOrigin(tf2::Vector3(robotPosX_, robotPosY_, robotPosZ_));

            tf2::Quaternion odom_quat;
            odom_quat.setRPY(0.0, 0.0, atan2(dY, dX));
            robotOrientation_ = tf2::toMsg(odom_quat);

            gps_odom_ = cartesianToMap(latest_utm_pose);
            gps_odom_.pose.pose.orientation = robotOrientation_;

            initFlag_ = true;
            pubGPSPos_.publish(gps_odom_);

            preRobotPosX_ = robotPosX_;
            preRobotPosY_ = robotPosY_;
        }
        else
        {
          initFlag_ = false;
        }

        if(!initFlag_)
        {
          preRobotPosX_ = robotPosX_;
          preRobotPosY_ = robotPosY_;
        }
    }

    void mapOriginHandler(const sensor_msgs::NavSatFix::ConstPtr &originMsg)
    {
        double lat = originMsg->latitude;
        double lon = originMsg->longitude;
        double alt = originMsg->altitude;

        std::string utm_zone_tmp;
        RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, mapY_, mapX_, utm_zone_tmp, utm_meridian_convergence_);
        utm_meridian_convergence_ = DEG2RAD(utm_meridian_convergence_);

        mapZ_ = alt;
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
            if(initFlag_)
            {
                geometry_msgs::PoseWithCovarianceStamped poseMsg;

                double curX = gps_odom_.pose.pose.position.x;
                double curY = gps_odom_.pose.pose.position.y;
                double curZ;

                getHeightFromCloud(curX, curY, curZ);

                poseMsg.header.stamp = ros::Time::now();
                poseMsg.header.frame_id = "map";
                poseMsg.pose.pose.position.x = curX;
                poseMsg.pose.pose.position.y = curY;
                poseMsg.pose.pose.position.z = curZ;
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
        seg.setModelType(pcl::SACMODEL_PLANE);      //적용 모델   // Configure the object to look for a plane.
        seg.setMethodType(pcl::SAC_RANSAC);         //적용 방법   // Use RANSAC method.
        seg.setMaxIterations(1000);                 //최대 실행 수
        seg.setDistanceThreshold(0.2);              //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
        seg.segment(*inliers, *coefficients);      //세그멘테이션 적용

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

        //std::cout << "diff: " << maxZ-minZ << std::endl;
        if(maxZ - minZ > 5.0)
            z = minZ;
        else
            z = averageZ;

        pubFilteredCloud_.publish(cloudFiltered);
    }

    nav_msgs::Odometry cartesianToMap(const tf2::Transform& cartesian_pose)
    {
        tf2::Transform transformed_cartesian_gps;
        tf2::Transform cartesian_world_transform;
        tf2::Quaternion yaw_difference;

        yaw_difference.setRPY(0.0, 0.0, utm_meridian_convergence_);
        cartesian_world_transform.setOrigin(tf2::Vector3(0, 0, 0));
        cartesian_world_transform.setRotation(yaw_difference);

        transformed_cartesian_gps.mult(cartesian_world_transform.inverse(), cartesian_pose);

        nav_msgs::Odometry gps_odom;
        gps_odom.header.frame_id = "map";
        gps_odom.header.stamp = ros::Time::now();
        tf2::toMsg(transformed_cartesian_gps, gps_odom.pose.pose);

        return gps_odom;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial_pose");

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_("~");

    ROS_INFO("\033[1;32m----> Initial Pose node started.\033[0m");

    initialPose IP(nh_, private_nh_);

    ros::spin();

    return 0;
}
