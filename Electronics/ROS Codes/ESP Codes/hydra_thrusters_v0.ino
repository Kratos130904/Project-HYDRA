#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <ESP32Servo.h>

const int thruster1Pin = 16;  
const int thruster2Pin = 17;
const int thruster3Pin = 18;
const int thruster4Pin = 19;

Servo thruster1Servo;
Servo thruster2Servo;
Servo thruster3Servo;
Servo thruster4Servo;

int16_t feedbackValues[4] = {1500, 1500, 1500, 1500};

std_msgs::Int16MultiArray feedback_msg;
ros::NodeHandle nh;
ros::Publisher feedback_pub("thruster_feedback", &feedback_msg);

int mapToPulseWidth(int input) {
  int pulseWidth = map(input, -400, 400, 1100, 1900);
  return constrain(pulseWidth, 1100, 1900);
}

void publishFeedback() {
  feedback_pub.publish(&feedback_msg);
}

void thruster1Callback(const std_msgs::Int16 &msg) {
  int pulseWidth = mapToPulseWidth(msg.data);
  thruster1Servo.writeMicroseconds(pulseWidth);
  feedbackValues[0] = pulseWidth;
  publishFeedback();
}

void thruster2Callback(const std_msgs::Int16 &msg) {
  int pulseWidth = mapToPulseWidth(msg.data);
  thruster2Servo.writeMicroseconds(pulseWidth);
  feedbackValues[1] = pulseWidth;
  publishFeedback();
}

void thruster3Callback(const std_msgs::Int16 &msg) {
  int pulseWidth = mapToPulseWidth(msg.data);
  thruster3Servo.writeMicroseconds(pulseWidth);
  feedbackValues[2] = pulseWidth;
  publishFeedback();
}

void thruster4Callback(const std_msgs::Int16 &msg) {
  int pulseWidth = mapToPulseWidth(msg.data);
  thruster4Servo.writeMicroseconds(pulseWidth);
  feedbackValues[3] = pulseWidth;
  publishFeedback();
}

ros::Subscriber<std_msgs::Int16> sub_thruster1("thruster1", thruster1Callback);
ros::Subscriber<std_msgs::Int16> sub_thruster2("thruster2", thruster2Callback);
ros::Subscriber<std_msgs::Int16> sub_thruster3("thruster3", thruster3Callback);
ros::Subscriber<std_msgs::Int16> sub_thruster4("thruster4", thruster4Callback);

void setup() {
  nh.initNode();

  thruster1Servo.attach(thruster1Pin);
  thruster2Servo.attach(thruster2Pin);
  thruster3Servo.attach(thruster3Pin);
  thruster4Servo.attach(thruster4Pin);

  feedback_msg.data = feedbackValues;
  feedback_msg.data_length = 4;

  nh.subscribe(sub_thruster1);
  nh.subscribe(sub_thruster2);
  nh.subscribe(sub_thruster3);
  nh.subscribe(sub_thruster4);

  nh.advertise(feedback_pub);
}

void loop() {
  nh.spinOnce();
  delay(10); 
}
