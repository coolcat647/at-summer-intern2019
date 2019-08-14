#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <haptic_msgs/Vibration.h>
#include <haptic_msgs/VibrationArray.h>

// WiFi configuration. Replace '***' with your data
const char* ssid = "EE622";
const char* password = "assistiverobotics";
IPAddress server(192, 168, 50, 212);  
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

haptic_msgs::VibrationArray vb_msg;
int motor_size = sizeof(vb_msg.motors[0])/sizeof(vb_msg.motors[0]);

// ROS node handler
ros::NodeHandle nh;
// ROS topics object definitions publisher and subscriber
ros::Publisher pub_motor("motor_msg", &vb_msg);
ros::Subscriber<haptic_msgs::VibrationArray> sub_motor("change_msg", &change_data);

void change_data(const haptic_msgs::VibrationArray &msg) {
  int msg_size = sizeof(msg.motors)/sizeof(msg.motors[0]);
  for(int i = 0; i<msg_size; i++) {
    vb_msg.motors[i].intensity = msg.motors[i].intensity;
    vb_msg.motors[i].frequency = msg.motors[i].frequency;
    /*Serial.print("motor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(vb_msg.motors[i].frequency);
    Serial.print(" ,");
    Serial.println(vb_msg.motors[i].intensity);*/
  }
  //Serial.println(sizeof(vb_msg));
  //Serial.println(sizeof(msg.motors[0]));
  //Serial.println(sizeof(haptic_msgs::Vibration));
  //Serial.println("\n=============\n");
}

const int motor_pin[3] = {5, 4, 0};   //motor connect signal pin
const int motor_interval[5] = {500, 250, 125, 100, 50};  //vibrate cycle T/2
const int pwm_min = 500;
int pwm[100];

void setup() {
  // ---------- set up bt and serial monitor baud rate------------
  Serial.begin(115200);
  setupWiFi();

  // Port NodeMCU Wi-Fi interface to ROS interface
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(pub_motor);
  nh.subscribe(sub_motor);
    
  // ---------------- set up -----------------
  for(int i=0; i<motor_size; i++) {
    pinMode(motor_pin[i], OUTPUT);
    pwm[i] = 0;
    analogWrite(motor_pin[i], 0);
  }
  // LED pin output configuration
  pinMode(LED_BUILTIN, OUTPUT);
  delay(5000); // Wait for ROS + Wi-Fi ready...
}

void loop() {
  if (nh.connected()) {
    for(int i = 0; i<motor_size; i++)
      pwm[i] = intensity2pwm(vb_msg.motors[i].intensity);
    outIO();
    //printOut();
    pub_motor.publish(&vb_msg);
  }
  else
    Serial.println("Not Connected to ROS socket server");
  nh.spinOnce();
  delay(100);
}

int intensity2pwm (int intensity_level) {
  int pwm = pwm_min + intensity_level * 100;
  if (intensity_level==0)
    return 0;
  else if (pwm<=1023)
    return pwm;
  else
    return 1023;
}

void outIO() {
  for(int i = 0; i<motor_size; i++) {
    analogWrite(motor_pin[i], pwm[i]);
    delay(motor_interval[vb_msg.motors[i].frequency-1]);
    analogWrite(motor_pin[i], 0);
    delay(motor_interval[vb_msg.motors[i].frequency-1]); 
  }
}

void printOut() {
  Serial.print("intensity: ");
  for(int i = 0; i<motor_size; i++) {
    Serial.print(pwm[i]);
    Serial.print(" ");
  }
  Serial.print(", ");
  Serial.print("frequency: ");
  for(int i = 0; i<motor_size; i++) {
    Serial.print(motor_interval[vb_msg.motors[i].frequency-1]);
    Serial.print(" ");
  }
  Serial.println();  
}
