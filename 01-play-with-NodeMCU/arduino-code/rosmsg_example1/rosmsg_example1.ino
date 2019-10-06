#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// WiFi configuration. Replace '***' with your data
const char* ssid = "******";
const char* password = "********";
IPAddress server(192, 168, 50, 219);  
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

volatile int32_t cnt = 0;

// ROS callback function
void chatter_cb(const std_msgs::String& msg){
    Serial.print("NodeMCU get message: ");
    Serial.println(msg.data);
    cnt++;
}

// ROS node handler
ros::NodeHandle nh;
// ROS topics object definitions publisher and subscriber
std_msgs::Int32 num_msg;
ros::Publisher pub_counter("msg_counter", &num_msg);
ros::Subscriber<std_msgs::String> sub_chat("chatter", &chatter_cb);


void setup() {  
    Serial.begin(115200);
    setupWiFi();

    // Port NodeMCU Wi-Fi interface to ROS interface
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(pub_counter);
    nh.subscribe(sub_chat);
    
    // LED pin output configuration
    pinMode(LED_BUILTIN, OUTPUT);
    delay(5000); // Wait for ROS + Wi-Fi ready...
}


void loop() {
    if (nh.connected()) {
        num_msg.data = cnt;
        pub_counter.publish(&num_msg);    // publish ROS topic
    }else{
      Serial.println("Not Connected to ROS socket server");
      delay(100);
    }
    nh.spinOnce();
    delay(100);
}
