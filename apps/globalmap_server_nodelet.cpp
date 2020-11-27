#include <iostream>
#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geographic_msgs/GeoPoint.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include <hdl_localization/pose_estimator.hpp>

namespace hdl_localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet() {
  }
  virtual ~GlobalmapServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    get_params();
    initialize_params();

    // publish globalmap with "latched" publisher
    map_lla_pub = nh.advertise<sensor_msgs::NavSatFix>("/map/gps/origin", 5, true);
    map_lla_pub.publish(map_origin);

    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    globalmap_pub.publish(globalmap);
  }

private:
  void get_params()
  {
      if(private_nh.hasParam("/hdl_localization/latitude"))
      {
        utm_convert = true;

        private_nh.param<double>("/hdl_localization/latitude", gpsOrigin.latitude, double());
        private_nh.param<double>("/hdl_localization/longitude", gpsOrigin.longitude, double());
        private_nh.param<double>("/hdl_localization/altitude", gpsOrigin.altitude, double());
        private_nh.param<double>("/hdl_localization/easting", gpsUTMOrigin.latitude, double());
        private_nh.param<double>("/hdl_localization/northing", gpsUTMOrigin.longitude, double());
        private_nh.param<double>("/hdl_localization/height", gpsUTMOrigin.altitude, double());
      }
  }

  void initialize_params()
  {
    // read globalmap from a pcd file
    std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    // shift to local coordinate
    for(size_t i=0; i<globalmap->size(); i++)
    {
        if(!utm_convert) break;

        globalmap->points[i].x -= gpsUTMOrigin.latitude;
        globalmap->points[i].y -= gpsUTMOrigin.longitude;
        globalmap->points[i].z -= gpsUTMOrigin.altitude;
    }

    // downsample globalmap
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;

    //map origin
    map_origin.header.frame_id = "map";
    map_origin.latitude = gpsOrigin.latitude;
    map_origin.longitude = gpsOrigin.longitude;
    map_origin.altitude = gpsOrigin.altitude;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;
  ros::Publisher map_lla_pub;

  pcl::PointCloud<PointT>::Ptr globalmap;
  sensor_msgs::NavSatFix map_origin;

  geographic_msgs::GeoPoint gpsOrigin{};
  geographic_msgs::GeoPoint gpsUTMOrigin{};

  bool utm_convert = false;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)
