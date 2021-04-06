#include <iostream>
#include <fstream>
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
#include <visualization_msgs/Marker.h>

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

    globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(1.0), &GlobalmapServerNodelet::pub_once_cb, this, true, true);

    // publish globalmap with "latched" publisher
    map_lla_pub = nh.advertise<sensor_msgs::NavSatFix>("hdl_localization/gloablmap/origin", 5, true);
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("hdl_localization/globalmap", 5, true);
    globalmap_csv_pub = nh.advertise<visualization_msgs::Marker>("hdl_localization/globalmap/trajectory", 1, true);
  }

private:
  void get_params()
  {
    private_nh.param<bool>("/globalmap_server_nodelet/use_gps_map", use_gps_map, bool());

    if(private_nh.hasParam("/globalmap_server_nodelet/latitude"))
    {
      utm_convert = true;

      private_nh.param<double>("/globalmap_server_nodelet/latitude", gpsOrigin.latitude, double());
      private_nh.param<double>("/globalmap_server_nodelet/longitude", gpsOrigin.longitude, double());
      private_nh.param<double>("/globalmap_server_nodelet/altitude", gpsOrigin.altitude, double());
      private_nh.param<double>("/globalmap_server_nodelet/easting", gpsUTMOrigin.latitude, double());
      private_nh.param<double>("/globalmap_server_nodelet/northing", gpsUTMOrigin.longitude, double());
      private_nh.param<double>("/globalmap_server_nodelet/height", gpsUTMOrigin.altitude, double());
    }
  }

  void initialize_params()
  {
    // read globalmap from a pcd file
    std::string globalmap_pcd = std::getenv("HOME") + private_nh.param<std::string>("globalmap_pcd", "");
    ROS_INFO("Map file: %s", globalmap_pcd.c_str());

    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    // shift to local coordinate
    for(size_t i=0; i<globalmap->size(); i++)
    {
      if(!utm_convert || !use_gps_map) break;

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

    //read trajectory from a csv
    std::string globalmap_csv = std::getenv("HOME") + private_nh.param<std::string>("globalmap_csv", "");

    std::ifstream read_file;
    read_file.open(globalmap_csv.c_str());

    line_strip.header.frame_id = "map";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.scale.x = 0.3;
    line_strip.color.a = line_strip.color.r = line_strip.color.b = 1.0;
    line_strip.pose.orientation.w = 1.0;

    if(read_file.is_open())
    {
      ROS_INFO("Trajectory file: %s", globalmap_csv.c_str());

      geometry_msgs::Point p;
      int read_cnt = 0;

      while(!read_file.eof())
      {
        double x, y, z;
        char buf[1024];

        read_file.getline(buf, 1024);
        read_cnt++;

        if (read_file.fail()) break;
        if (read_cnt==1) continue;

        sscanf(buf, "%lf,%lf,%lf", &x,&y,&z);

        if(read_cnt==2) {
          p.x=x; p.y=y; p.z=z;
          continue;
        }
        else {
          line_strip.points.push_back(p);
          p.x=x; p.y=y; p.z=z;
          line_strip.points.push_back(p);
        }
      }

      read_file.close();
    }
  }

  void pub_once_cb(const ros::WallTimerEvent& event)
  {
    map_lla_pub.publish(map_origin);
    globalmap_pub.publish(globalmap);
    globalmap_csv_pub.publish(line_strip);
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;
  ros::Publisher globalmap_csv_pub;
  ros::Publisher map_lla_pub;

  ros::WallTimer globalmap_pub_timer;

  pcl::PointCloud<PointT>::Ptr globalmap;
  sensor_msgs::NavSatFix map_origin;

  geographic_msgs::GeoPoint gpsOrigin{};
  geographic_msgs::GeoPoint gpsUTMOrigin{};

  visualization_msgs::Marker line_strip;

  bool utm_convert = false;
  bool use_gps_map = false;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)
