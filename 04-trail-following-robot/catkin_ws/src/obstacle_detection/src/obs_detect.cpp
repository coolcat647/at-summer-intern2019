#include <string.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h> 
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <haptic_msgs/Wristband.h>
#include <haptic_msgs/VibrationArray.h>
#include <haptic_msgs/Vibration.h>
#include <std_msgs/Int32.h>

#define PI 3.14159265f
#define DEGREE_OF_VIEW 90.0f
#define DEGREE_OF_CENTER 40.0f
#define DANGER_DISTANCE 1
#define UNSAFETY_DISTANCE 1.5
#define NUM_MOTORS 3


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

class LaserObstacleDetection{
    public:
    ros::NodeHandle nh_;
    
    ros::Publisher pub_cloud_;
    ros::Publisher pub_range_;
    ros::Subscriber sub_laser_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    sensor_msgs::Range detection_range;

    enum FILED{
        DANGER_FRONT= 0,
        DANGER_LEFT,
        DANGER_RIGHT,
        UNSAFETY_FRONT,
        UNSAFETY_LEFT,
        UNSAFETY_RIGHT,
        DONT_CARE,
    };

    const std::string FILED_NAMES[7] = {"DANGER_FRONT",
        "DANGER_LEFT",
        "DANGER_RIGHT",
        "UNSAFETY_FRONT",
        "UNSAFETY_LEFT",
        "UNSAFETY_RIGHT",
        "DONT_CARE"
    };
    

    LaserObstacleDetection(ros::NodeHandle n): nh_(n){
        // ROS subscriber
        sub_laser_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &LaserObstacleDetection::laserscan_cb, this);
        // ROS publisher
        pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("cloud", 1);
        pub_range_ = nh_.advertise<sensor_msgs::Range> ("detection_field", 1);
        pub_wristband_ = nh_.advertise<haptic_msgs::Wristband> ("wristbands_msg", 1);
        detection_range.header.frame_id = "laser_frame";
        detection_range.radiation_type = sensor_msgs::Range::INFRARED;
        detection_range.field_of_view = DEGREE_OF_VIEW * PI / 180.0;
        detection_range.min_range = 0.1;
        detection_range.max_range = UNSAFETY_DISTANCE;
        detection_range.range = UNSAFETY_DISTANCE;

    }

    uint16_t point_to_field(float x, float y){
        float d = sqrt(y*y + x*x);
        float angle = atan2(y, x) * 180 / PI;
        FILED label;
        if(d < DANGER_DISTANCE){
            if(abs(angle) <= DEGREE_OF_CENTER/2)
                label = DANGER_FRONT; 
            else if(angle > DEGREE_OF_CENTER/2 && angle < DEGREE_OF_VIEW/2)
                label = DANGER_LEFT;
            else if(angle < -DEGREE_OF_CENTER/2 && angle > -DEGREE_OF_VIEW/2)
                label = DANGER_RIGHT;          
        }
        else if(d >= DANGER_DISTANCE && d < UNSAFETY_DISTANCE){
            if(abs(angle) <= DEGREE_OF_CENTER/2)
                label = UNSAFETY_FRONT; 
            else if(angle > DEGREE_OF_CENTER/2 && angle < DEGREE_OF_VIEW/2)
                label = UNSAFETY_LEFT;
            else if(angle < -DEGREE_OF_CENTER/2 && angle > -DEGREE_OF_VIEW/2)
                label = UNSAFETY_RIGHT;
        }
        else
            label = DONT_CARE;

        return label;
    }

    void laserscan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in){
        sensor_msgs::PointCloud2 cloud_in;
        PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
        PointCloudXYZ::Ptr cloud_filtered(new PointCloudXYZ);
        
        // Convert laser scan data to pointcloud
        projector_.projectLaser (*scan_in, cloud_in);
        pcl::fromROSMsg(cloud_in, *cloud);

        // Filter point cloud by specific 
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
            pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -1.5)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
            pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
            pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -0.8)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
            pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, 0.8)));
        // Build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        // Apply filter
        condrem.filter (*cloud_filtered);
        

        
        // Transform pointcloud to base_link view
        tf::TransformListener listener;
        tf::StampedTransform tf1;
        try{
            listener.waitForTransform("base_link", "laser_frame", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("base_link", "laser_frame", ros::Time(0), tf1);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
        
        pcl_ros::transformPointCloud (*cloud_filtered, *cloud_filtered, tf1);


        uint16_t vote[7] = {0};
        for (int i = 0; i < cloud_filtered->points.size(); i++){
            int idx = point_to_field(cloud_filtered->points[i].x, cloud_filtered->points[i].y);
            vote[idx]++;
        }
        for(int i=0; i < 7; i++){
            if(i==3) cout << "\n";
            cout << "[" << FILED_NAMES[i] << "]:" << vote[i] << ",\t";
        }
        cout << "\n======================" << endl;
        
        haptic_msgs::Wristband wristbands_msg;
        wristbands_msg.left.motors = (haptic_msgs::Vibration*)malloc(NUM_MOTORS * sizeof(haptic_msgs::Vibration));
        wristbands_msg.right.motors = (haptic_msgs::Vibration*)malloc(NUM_MOTORS * sizeof(haptic_msgs::Vibration));
        for(int i = 0; i < NUM_MOTORS; i++){
            wristbands_msg.left.motors[i].frequency = 0;
            wristbands_msg.left.motors[i].intensity = 0;
            wristbands_msg.right.motors[i].frequency = 0;
            wristbands_msg.right.motors[i].intensity = 0;
        }
        for (int i = 0; i < 6; i++){
            if(vote[i] > 30){
                switch(i){
                    case 0:
                        for(int j = 0; j < 3; j++){
                            wristbands_msg.left.motors[j].frequency = 3;
                            wristbands_msg.left.motors[j].intensity = 5;
                            wristbands_msg.right.motors[j].frequency = 3;
                            wristbands_msg.right.motors[j].intensity = 5;
                        }
                        break;
                    case 1:
                        for(int j = 0; j < 3; j++){
                            wristbands_msg.left.motors[j].frequency = 5;
                            wristbands_msg.left.motors[j].intensity = 5;
                        }
                        break;
                    case 2:
                        for(int j = 0; j < 3; j++){
                            wristbands_msg.right.motors[j].frequency = 5;
                            wristbands_msg.right.motors[j].intensity = 5;
                        }
                        break;
                    case 3:
                        if(wristbands_msg.left.motors[0].frequency != 0 && wristbands_msg.right.motors[0].frequency != 0)
                            break;
                        else{
                            for(int j = 0; j < 3; j++){
                                wristbands_msg.left.motors[j].frequency = 1;
                                wristbands_msg.left.motors[j].intensity = 3;
                                wristbands_msg.right.motors[j].frequency = 1;
                                wristbands_msg.right.motors[j].intensity = 3;
                            }
                        }
                        break;
                    case 4:
                        if(wristbands_msg.left.motors[0].frequency != 0)
                            break;
                        else{
                            for(int j = 0; j < 3; j++){
                                wristbands_msg.left.motors[j].frequency = 3;
                                wristbands_msg.left.motors[j].intensity = 3;
                            }
                        }
                        break;
                    case 5:
                        if(wristbands_msg.right.motors[0].frequency != 0)
                            break;
                        else{
                            for(int j = 0; j < 3; j++){
                                wristbands_msg.right.motors[j].frequency = 3;
                                wristbands_msg.right.motors[j].intensity = 3;
                            }
                        }
                        break;
                    default: break;
                }
            }     
        }
        // Publish the data
        pub_cloud_.publish(*cloud_filtered);

        detection_range.header.stamp = ros::Time::now();
        pub_range_.publish(detection_range);
        pub_wristband_.publish(wristbands_msg);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    LaserObstacleDetection obj(nh);
    ros::spin();
    return 0;
}