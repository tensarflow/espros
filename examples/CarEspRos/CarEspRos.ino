////////////////////////////////
//
// Wifi ROS Car with ESP8266
//
// Find last versions at:
// https://github.com/agnunez/espros.git
//
// MIT License 2017 Agustin Nunez
//
// EDIT: is not using any encorder!!!
/////////////////////////////////

#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

#define DEBUG 1

int lpwm = 0;
int rpwm = 0;
int len = 250; // period in ms
int ldir = 1;  // left motor direction
int rdir = 1;  // right motor direction

//////////////////////
// WiFi Definitions //
const char *ssid = "TTNET_TP-LINK_B58D";
const char *password = "rsQTaSXY";
IPAddress server(192, 168, 1, 110); // Set the rosserial socket server IP address
const uint16_t serverPort = 11411;  // Set the rosserial socket server port
//////////////////////

void stop(void)
{ // Stop both motors
  analogWrite(D1, 0);
  analogWrite(D6, 0);
}

void motion(int lpw, int rpw, int ldir, int rdir, int period)
{ // generic motion for a period in milisecods
  bool llevel_1 = ldir;
  bool llevel_2 = !(ldir);
  bool rlevel_1 = rdir;
  bool rlevel_2 = !(rdir);

  // Set speed
  analogWrite(D1, lpw);
  analogWrite(D6, rpw);

  // Give direction
  digitalWrite(D2, llevel_1);
  digitalWrite(D3, llevel_2);
  digitalWrite(D4, rlevel_1);
  digitalWrite(D5, rlevel_2);

  delay(period);
  stop();
}

//  All subscriber messages callbacks here
void leftCallback(const std_msgs::Int16 &msg)
{
  rpwm = abs(msg.data);
  motion(lpwm, rpwm, LOW, HIGH, len);
  Serial.println("Driving left.");
}
void rightCallback(const std_msgs::Int16 &msg)
{
  lpwm = abs(msg.data);
  motion(lpwm, rpwm, HIGH, LOW, len);
  Serial.println("Driving right.");
}
void forwardCallback(const std_msgs::Int16 &msg)
{
  lpwm = abs(msg.data);
  rpwm = abs(msg.data);
  motion(lpwm, rpwm, HIGH, HIGH, len);
  Serial.println("Driving forward.");
}
void backwardCallback(const std_msgs::Int16 &msg)
{
  lpwm = abs(msg.data);
  rpwm = abs(msg.data);
  motion(lpwm, rpwm, LOW, LOW, len);
  Serial.println("Driving backwards.");
}

ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);

void setupWiFi()
{
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

ros::NodeHandle nh;

void setup()
{
  Serial.begin(115200);
  setupWiFi();
  delay(2000);

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub_r);
  nh.subscribe(sub_l);
  nh.subscribe(sub_f);
  nh.subscribe(sub_b);

  // configure GPIO's
  pinMode(D1, OUTPUT); // ENA, PWM left motor (A)
  pinMode(D2, OUTPUT); // IN1
  pinMode(D3, OUTPUT); // IN2
  pinMode(D4, OUTPUT); // IN3
  pinMode(D5, OUTPUT); // IN4
  pinMode(D6, OUTPUT); // ENB, PWM right motor (B)

  analogWriteFreq(50);
}

void loop()
{
  nh.spinOnce();
  delay(500);
}
