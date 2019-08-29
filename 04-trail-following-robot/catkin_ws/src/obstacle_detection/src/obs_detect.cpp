#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions.h>
#include <boost/foreach.hpp>
#include <laser_geometry/laser_geometry.h>

laser_geometry::LaserProjection projector_
sensor_msgs::PointCloud2 point_cloud_publisher_;
tf::TransformListener listener_;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void laserscan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    if(!listener_.waitForTransform(scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("base_link", *scan_in, cloud, Listener_);
    point_cloud_publisher_.publish(cloud);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/scan", 1, laserscan_cb);
    point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("/cloud", 100);
    ros::spin();
}