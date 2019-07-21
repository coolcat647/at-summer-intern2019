#include <ESP8266WiFi.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159265359f

// WiFi configuration. Replace '***' with your data
const char* ssid = "***";
const char* password = "***";
IPAddress server(10, 42, 0, 133);  
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
ros::Publisher pub_odom("odom", &odom_msg);

// Transform Broadcaster
tf::TransformBroadcaster broadcaster;

double x = 0.0f;
double y = 0.0f;
double theta = 0.0f; //1.57;

// Constant speed x, y, th directions
double vx = 0.1;
double vy = -0.1;
double vth = 0.1;

char base_link[] = "/base_link";
char odom[] = "/odom";


void setup() {  
    Serial.begin(115200);
    setupWiFi();

    // Port NodeMCU Wi-Fi interface to ROS interface
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(pub_odom);
    broadcaster.init(nh);
    
    // LED pin output configuration
    pinMode(LED_BUILTIN, OUTPUT);
    delay(5000); // Wait for ROS + Wi-Fi ready...
}

ros::Time current_time;
ros::Time last_time;
bool flag_first_run = true;

void loop() {
    if (nh.connected()) {
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
        Serial.print("x= ");
        Serial.print(x);
        Serial.print(", y= ");
        Serial.print(y);
        Serial.print(", dt= ");
        Serial.print(dt);
        Serial.print(", dx= ");
        Serial.print(dx);
        Serial.print(", dy= ");
        Serial.println(dy);

        x += dx;
        y += dy;
        theta += dtheta;
        
        geometry_msgs::Quaternion q = tf::createQuaternionFromYaw(theta);
                
        // tf odom->base_link
        tf_msg.header.frame_id = odom;
        tf_msg.child_frame_id = base_link;
        
        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.translation.z = 0.0f;
        tf_msg.transform.rotation = tf::createQuaternionFromYaw(theta);
        tf_msg.header.stamp = nh.now();
        
        broadcaster.sendTransform(tf_msg);


        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom_msg;
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
    
        //publish the message
        pub_odom.publish(&odom_msg);

        last_time = current_time;
        
    }else{
      Serial.println("Not Connected to ROS socket server");
    }
    nh.spinOnce();
    delay(20);
}
