#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <haptic_msgs/Vibration.h>
#include <haptic_msgs/VibrationArray.h>
#include <std_srvs/Trigger.h>

#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2

// WiFi configuration. Replace '***' with your data
const char* ssid = "EE622";
const char* password = "assistiverobotics";
IPAddress server(192, 168, 50, *);
const uint16_t serverPort = 11411;      // Set the rosserial socket server port

const int motor_pins[] = {D1, D2, D3};   //motor connect signal pin
const int NUM_MOTORS = sizeof(motor_pins)/sizeof(motor_pins[0]);
const int motor_interval[] = {500, 250, 125, 100, 50};  //vibrate cycle T/2
const int pwm_min = 500;

void vibration_cb(const haptic_msgs::VibrationArray &msg);

// ROS node handler
ros::NodeHandle nh;

// ROS msg instance
haptic_msgs::VibrationArray vb_msg;

// ROS topics object definitions publisher and subscriber
ros::Subscriber<haptic_msgs::VibrationArray> sub_motor("change_msg", &vibration_cb);

int pwm[NUM_MOTORS];

// ROS callback function
void vibration_cb(const haptic_msgs::VibrationArray &msg) {
    for(int i = 0; i < msg.motors_length; i++) {
        vb_msg.motors[i].intensity = msg.motors[i].intensity;
        vb_msg.motors[i].frequency = msg.motors[i].frequency;
    }
    Serial.println("Get motors cmd.");
}

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

void setup() {
    // ---------------- GPIO setup -----------------
    for(int i=0; i < NUM_MOTORS; i++) {
        pinMode(motor_pins[i], OUTPUT);
        pwm[i] = 0;
        analogWrite(motor_pins[i], 0);
    }
    
    Serial.begin(115200);
    setupWiFi();
    
    vb_msg.motors = (haptic_msgs::Vibration*)malloc(NUM_MOTORS * sizeof(haptic_msgs::Vibration));
    vb_msg.motors_length = NUM_MOTORS;

    // Motors init
    for(int i=0; i < NUM_MOTORS; i++) {
        vb_msg.motors[i].intensity = 0;
        vb_msg.motors[i].frequency = 0;
    }
    
    // LED pin output configuration
    // pinMode(LED_BUILTIN, OUTPUT);

    // Port NodeMCU Wi-Fi interface to ROS interface
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.subscribe(sub_motor);
    
} 

int intensity2pwm (int intensity_level) {
    int pwm = pwm_min + intensity_level * 100;
    if (intensity_level == 0)
        return 0;
    else if (pwm<=1023)
        return pwm;
    else
        return 1023;
}

void outIO() {
    for(int i = 0; i < NUM_MOTORS; i++) {
        analogWrite(motor_pins[i], pwm[i]);
        delay(motor_interval[vb_msg.motors[i].frequency-1]);
        analogWrite(motor_pins[i], 0);
        delay(motor_interval[vb_msg.motors[i].frequency-1]); 
    }
}

void printOut() {
  Serial.print("intensity: ");
  for(int i = 0; i < NUM_MOTORS; i++) {
      Serial.print(pwm[i]);
      Serial.print(" ");
  }
  Serial.print(", ");
  Serial.print("frequency: ");
  for(int i = 0; i < NUM_MOTORS; i++) {
      //Serial.print(motor_interval[vb_msg.motors[i].frequency-1]);
      Serial.print(" ");
  }
  Serial.println();  
}

void loop() {
    if (nh.connected()) {
        for(int i = 0; i < NUM_MOTORS; i++)
            pwm[i] = intensity2pwm(vb_msg.motors[i].intensity);
        outIO();
        // printOut();
    }else{
        Serial.println("Not Connected to ROS socket server");
        delay(100);
    }
    nh.spinOnce();
    delay(100);
}
