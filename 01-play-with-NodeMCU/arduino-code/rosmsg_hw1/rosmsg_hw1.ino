#include <ESP8266WiFi.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#define PI 3.14159265359f

// WiFi configuration. Replace '***' with your data
const char* ssid = "***";
const char* password = "********";
IPAddress server(192, 168, 50, 126);  
const uint16_t serverPort = 11411;      // Set the rosserial socket server port


void setupWiFi() {                      // connect to ROS server as as a client
    Serial.begin(115200);               // Use ESP8266 serial only for to monitor the process
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// ROS node handler
ros::NodeHandle nh;

// ROS topics object definitions 
geometry_msgs::TransformStamped tf_msg;
nav_msgs::Odometry odom_msg;
nav_msgs::Odometry odom_msg2;
ros::Publisher pub_odom("odom", &odom_msg);
ros::Publisher pub_odom2("odom2", &odom_msg2);

// Transform Broadcaster
tf::TransformBroadcaster broadcaster;

double x = 0.0f;
double y = 0.0f;
double theta = 0.0f; //1.57;

double x_2 = 0.0f;
double y_2 = 0.0f;
double theta_2 = 0.0f; //1.57;

// Constant speed x, y, th directions
double vx = -0.1;
double vy = 0.1;
double vth = -0.1;

double vx_2 = 0.1;
double vy_2 = -0.1;
double vth_2 = 0.1;

char base_link[] = "/base_link";
char base_link2[] = "/base_link2";
char odom[] = "/odom";
char odom2[] = "/odom";

//reset_pose
std_msgs::Bool bool_msg;
ros::Publisher pub_bool("reset_pose", &bool_msg);
ros::Subscriber<std_msgs::Bool> sub_reset("flag", &flag_msg);

void setup() {  
    Serial.begin(115200);
    setupWiFi();

    // Port NodeMCU Wi-Fi interface to ROS interface
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(pub_odom);
    nh.advertise(pub_odom2);
    nh.advertise(pub_bool);
    nh.subscribe(sub_reset);
    broadcaster.init(nh);
    
    // LED pin output configuration
    pinMode(LED_BUILTIN, OUTPUT);
    delay(5000); // Wait for ROS + Wi-Fi ready...
}

ros::Time current_time;
ros::Time last_time;
bool flag_first_run = true;
void flag_msg(const std_msgs::Bool& f){
    bool_msg.data = f.data;
}

void loop() {
    if (nh.connected()) {
      
        //if-not reset
        if(bool_msg.data) {
            x = 0.0;
            y = 0.0;
            theta = 0.0;
            x_2 = 0.0;
            y_2= 0.0;
            theta_2 = 0.0;
            bool_msg.data = false;
        }
                
        if(flag_first_run == true){
            flag_first_run = false;
            last_time = nh.now();
        }

        // try to drive in a circle
        current_time = nh.now();
        double dt = (current_time.toSec() - last_time.toSec());
        
        if(dt > 100){
            last_time = current_time;
            return;
        }
        
        double dx = (vx * cos(theta) - vy * sin(theta)) * dt;
        double dy = (vx * sin(theta) + vy * cos(theta)) * dt;
        double dtheta = vth * dt;
        double dx_2 = (vx_2 * cos(theta_2) - vy_2 * sin(theta_2)) * dt;
        double dy_2 = (vx_2 * sin(theta_2) + vy_2 * cos(theta_2)) * dt;
        double dtheta_2 = vth_2 * dt;
        
/*        Serial.print("x= ");
        Serial.print(x);
        Serial.print(", y= ");
        Serial.print(y);
        Serial.print(", dt= ");
        Serial.print(dt);
        Serial.print(", dx= ");
        Serial.print(dx);
        Serial.print(", dy= ");
        Serial.println(dy);
*/
        x += dx;
        y += dy;
        theta += dtheta;

        x_2 += dx_2;
        y_2 += dy_2;
        theta_2 += dtheta_2;
        
        geometry_msgs::Quaternion q = tf::createQuaternionFromYaw(theta);
        geometry_msgs::Quaternion q_2 = tf::createQuaternionFromYaw(theta_2);
                
        // tf odom->base_link(odom/tf)
        tf_msg.header.frame_id = odom;
        tf_msg.child_frame_id = base_link;
        
        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.translation.z = 0.0f;
        tf_msg.transform.rotation = tf::createQuaternionFromYaw(theta);
        tf_msg.header.stamp = nh.now();
        
        broadcaster.sendTransform(tf_msg);

        //next, we'll publish the odometry message over ROS
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
    
        //set the position
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = q;
    
        //set the velocity
        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = vth;

        // tf odom->base_link(odom2/tf2)
        tf_msg.header.frame_id = odom;
        tf_msg.child_frame_id = base_link2;
        
        tf_msg.transform.translation.x = x_2;
        tf_msg.transform.translation.y = y_2;
        tf_msg.transform.translation.z = 0.0f;
        tf_msg.transform.rotation = tf::createQuaternionFromYaw(theta_2);
        tf_msg.header.stamp = nh.now();
        
        broadcaster.sendTransform(tf_msg);

        //next, we'll publish the odometry message over ROS
        odom_msg2.header.stamp = current_time;
        odom_msg2.header.frame_id = "odom";
    
        //set the position
        odom_msg2.pose.pose.position.x = x_2;
        odom_msg2.pose.pose.position.y = y_2;
        odom_msg2.pose.pose.position.z = 0.0;
        odom_msg2.pose.pose.orientation = q_2;
    
        //set the velocity
        odom_msg2.child_frame_id = "base_link2";
        odom_msg2.twist.twist.linear.x = vx_2;
        odom_msg2.twist.twist.linear.y = vy_2;
        odom_msg2.twist.twist.angular.z = vth_2;

        //publish the message
        pub_odom.publish(&odom_msg);
        pub_odom2.publish(&odom_msg2);
        pub_bool.publish(&bool_msg);

        last_time = current_time;
        
    }else{
        Serial.println("Not Connected to ROS socket server");
    }
    nh.spinOnce();
    delay(100);
}
