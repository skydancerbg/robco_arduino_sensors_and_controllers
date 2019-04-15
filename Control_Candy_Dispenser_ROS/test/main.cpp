#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
// https://github.com/agnunez/espros/blob/master/examples/CarEspRos/CarEspRos.ino

//////////////////////
// WiFi Definitions //
//////////////////////
// WiFi configuration. Replace '***' with your data
const char* ssid = "code";
const char* password = "codeass123";
IPAddress server(192,168,1,2);      // Set the rosserial socket server IP address
const uint16_t serverPort = 11411;    // Set the rosserial socket server port
std_msgs::String str_msg;
// std_msgs::Int16 int_msg;
ros::Publisher candy_disp_pub("/robko/candy_dispenser/", &str_msg);
// ros::Publisher rightenc("/car/rightencoder", &str_msg);

// ros::Subscriber<std_msgs::String> sub_command("/car/forward", &forwardCallback);
// ros::Subscriber<std_msgs::Float32> sub_time("/car/backward", &backwardCallback);

void setupWiFi() {                    // connect to ROS server as as a client
  Serial.begin(115200);               // Use ESP8266 serial only for to monitor the process
  Serial.println();
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

ros::NodeHandle nh;

void setup() {
  Serial.begin(115200);
  setupWiFi();
  delay(1000);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // nh.subscribe(sub_time);
  // nh.subscribe(sub_command);



}

void loop() {
  str_msg.data = "test";
  candy_disp_pub.publish( &str_msg );
  // int_msg.data = rmc;
  // rightenc.publish( &int_msg );
  nh.spinOnce();
  delay(1000);
}