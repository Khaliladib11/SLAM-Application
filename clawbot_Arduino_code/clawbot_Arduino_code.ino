#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>


//Encoders Pin
const int encoderR = 2;
const int encoderL = 3;

//Right Motor Control
const int enR = 10;
const int in1 = 8;
const int in2 = 9;

//Left Motor Control
const int enL = 5;
const int in3 = 6;
const int in4 = 7;

//Encoder Tricks counter
volatile int encoderR_tricks = 0;
volatile int encoderL_tricks = 0;


//ROS messeges
std_msgs::Int16 lwheel;
std_msgs::Int16 rwheel;


//Moving Functions
void moveForward(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enR, 255);
  analogWrite(enL, 255);
}

void moveBackward(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enR, 255);
  analogWrite(enL, 255);
}

void moveToRight(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enR, 125);
  analogWrite(enL, 255);
}

void moveToLeft(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enR, 255);
  analogWrite(enL, 125);
}

void stopMoving(){
  analogWrite(enR, 0);
  analogWrite(enL, 0);
}

ros::NodeHandle nh;

//CallBack function for ROS
void cmd_vel_cb(const geometry_msgs::Twist & msg){
  const float x = msg.linear.x;
  const float z = msg.angular.z;
  if(x > 0 && z == 0){
     moveForward();
  }
  else if(x < 0 && z == 0){
    moveBackward();
  }
  else if(z > 0){
    moveToRight();
  }
  else if(z < 0){
    moveToLeft();
  }
  else{
    stopMoving();
   }
}

//Punlishing Right and Left tricks over ROS
ros::Publisher lwheelPub("lwheel", &lwheel);
ros::Publisher rwheelPub("rwheel", &rwheel);


//CallBack function for encoder's Interrupt
void ISR_countR(){
  encoderR_tricks++;
  rwheel.data = encoderR_tricks;
  rwheelPub.publish( &rwheel );
}
void ISR_countL(){
  encoderL_tricks++;
  lwheel.data = encoderL_tricks;
  lwheelPub.publish( &lwheel );
}

//Subscribe to geometry twist messages
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

void setup() {
  pinMode(enR, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(encoderR, INPUT_PULLUP);
  pinMode(encoderL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderR), ISR_countR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderL), ISR_countL, RISING);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
