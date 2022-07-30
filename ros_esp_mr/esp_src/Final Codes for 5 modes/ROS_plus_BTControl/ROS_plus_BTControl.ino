#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

char receivedChar;


const uint8_t R_F_PWM = 12;
const uint8_t R_F_BACK = 33;
const uint8_t R_F_FORW = 32;

const uint8_t L_F_PWM = 13;
const uint8_t L_F_BACK = 25;
const uint8_t L_F_FORW = 26;

const uint8_t channel_L =0;
const uint8_t channel_R= 1;


int count_r=0;
int count_l=0;

float left_wheel;
float right_wheel;


IPAddress server("Ip address ex: 192.168.38.24);
uint16_t serverPort = 11411;
const char*  ssid = "device wifi network";
const char*  password = "device wifi network password";

void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg);
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb );



char command;

void setup(){
    Serial.begin(115200);
    SerialBT.begin("ESP32_BT_ROS_ROVER");
    setupWiFi();

    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();

    nh.subscribe(sub);
    pin_defination();

}

void forward(){

    digitalWrite(R_F_FORW, HIGH);
    digitalWrite(L_F_FORW, HIGH);
    digitalWrite(R_F_BACK, LOW);
    digitalWrite(L_F_BACK, LOW);

}

void back(){

    digitalWrite(R_F_FORW, LOW);
    digitalWrite(L_F_FORW, LOW);
    digitalWrite(R_F_BACK, HIGH);
    digitalWrite(L_F_BACK, HIGH);

}

void right(){

    digitalWrite(R_F_FORW, LOW);
    digitalWrite(L_F_FORW, HIGH);
    digitalWrite(R_F_BACK, HIGH);
    digitalWrite(L_F_BACK, LOW);

}

void left(){

    digitalWrite(R_F_FORW, HIGH);
    digitalWrite(L_F_FORW, LOW);
    digitalWrite(R_F_BACK, LOW);
    digitalWrite(L_F_BACK, HIGH);

}
  
void Stop(){
 
  digitalWrite(R_F_BACK, LOW);
  digitalWrite(R_F_FORW, LOW);
  digitalWrite(L_F_BACK, LOW);
  digitalWrite(L_F_FORW, LOW);
  
}



void direction(){

    digitalWrite(L_F_FORW, left_wheel > 0);
    digitalWrite(L_F_BACK, left_wheel < 0);
    digitalWrite(R_F_FORW, right_wheel > 0);
    digitalWrite(R_F_BACK, right_wheel < 0);

}

void speed (){
    ledcWrite(channel_R, 300);  
    ledcWrite(channel_L, 300);
}

void stop()
{
   ledcWrite(channel_R, 0);  
   ledcWrite(channel_L, 0);
}


void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg){

    right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z ) / 2 ;
    left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z ) /2 ;

    direction();
    speed();
    if ( velocity_msg.linear.x ==0.0 & velocity_msg.angular.z ==0.0){
        stop();
    }
    Serial.print(left_wheel);Serial.print(" / ");Serial.println(right_wheel);

}

void setupWiFi()
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());

}

void pin_defination(){
    
    
  const int freq = 5000;
  const int res = 8;

  //pinMode(L_F_PWM,  OUTPUT);
  pinMode(L_F_FORW, OUTPUT);
  pinMode(L_F_BACK, OUTPUT);
  //pinMode(R_F_PWM,  OUTPUT);
  pinMode(R_F_FORW, OUTPUT);
  pinMode(R_F_BACK, OUTPUT);

  
  ledcSetup(channel_R ,freq , res);
  ledcSetup(channel_L ,freq , res);

  ledcAttachPin(R_F_PWM,channel_R);
  ledcAttachPin(L_F_PWM,channel_L);

}

void loop(){

   
  receivedChar = (char)SerialBT.read();
  if(Serial.available() > 0){
    SerialBT.write(Serial.read());
  }
  

  if(SerialBT.available()) {
    Serial.print("Recieved:");
    Serial.println(receivedChar);

    if(receivedChar == 'F')
    {
      forward();
       
    }
    if(receivedChar == 'B')
    {
 
      back(); 
    }         
     if(receivedChar == 'L')
    {

      left();
    }        
    if(receivedChar == 'R')
    {

      right(); 
    }
    if(receivedChar == '1')
    {
      Stop();
    }
  }
   
  else{
    nh.spinOnce();
    delay(500);

  }
  delay(90);
    
}
