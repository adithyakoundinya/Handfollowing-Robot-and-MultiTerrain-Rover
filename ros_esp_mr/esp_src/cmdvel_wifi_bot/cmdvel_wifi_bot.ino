#include "WiFi.h"
#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <geometry_msgs/Twist.h>



const uint8_t R_F_PWM =  12;
const uint8_t R_F_BACK = 2;
const uint8_t R_F_FORW = 4;

const uint8_t L_F_PWM =  33;
const uint8_t L_F_BACK = 27;
const uint8_t L_F_FORW = 26;

const uint8_t R_B_BACK = 18;
const uint8_t R_B_FORW = 19;

const uint8_t L_B_BACK = 25;
const uint8_t L_B_FORW = 33;


const uint8_t channel_L =0;
const uint8_t channel_R= 1;

float f_left_wheel;
float f_right_wheel;
float b_left_wheel;
float b_right_wheel;

IPAddress server(192, 168, 43, 55);
uint16_t serverPort = 11411;
const char*  ssid = "Arya";
const char*  password = "teriyaki";


void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg);
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb );

void setup(){


    Serial.begin(115200);
    setupWiFi();
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.subscribe(sub);


    pin_def();
    stop();
    Serial.println("Get Ready");
    delay(2000);
}

void pin_def(){

  const int freq = 5000;
  const int res = 8;
  
  pinMode(L_F_PWM,  OUTPUT);
  pinMode(L_F_FORW, OUTPUT);
  pinMode(L_F_BACK, OUTPUT);
  pinMode(R_F_PWM,  OUTPUT);
  pinMode(R_F_FORW, OUTPUT);
  pinMode(R_F_BACK, OUTPUT);
//  pinMode(L_B__PWM, OUTPUT);
  pinMode(L_B_FORW, OUTPUT);
  pinMode(L_B_BACK, OUTPUT);
//  pinMode(R_B_PWM,  OUTPUT);
  pinMode(R_B_FORW, OUTPUT);
  pinMode(R_B_BACK, OUTPUT);
  
  ledcSetup(channel_R ,freq , res);
  ledcSetup(channel_L ,freq , res);

  ledcAttachPin(R_F_PWM,channel_R);
  ledcAttachPin(L_F_PWM,channel_L);

}


void loop(){

  nh.spinOnce();

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



void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg){

    f_right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z ) / 2 ;
    f_left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z ) /2 ;
    b_right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z ) / 2 ;
    b_left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z ) /2 ;
    direction();
    speed();
    if ( velocity_msg.linear.x ==0.0 & velocity_msg.angular.z ==0.0){
        stop();
    }
    Serial.print(f_left_wheel);Serial.print(" / ");Serial.println(f_right_wheel);
    Serial.print(b_left_wheel);Serial.print(" / ");Serial.println(b_right_wheel);

}

void direction(){
    digitalWrite(L_F_FORW&&L_B_BACK, f_left_wheel&&b_left_wheel >0 );
    digitalWrite(L_F_FORW&&L_B_BACK,f_left_wheel&&b_left_wheel < 0);
    digitalWrite(R_F_FORW&&R_B_BACK,f_right_wheel&&b_right_wheel > 0 );
    digitalWrite(R_F_FORW&&R_B_BACK,f_right_wheel&&b_right_wheel < 0);
}

void speed (){
    ledcWrite(channel_R, 200);  
    ledcWrite(channel_L, 200);
}

void stop()
{
   
   
   ledcWrite(channel_R, 0);  
   ledcWrite(channel_L, 0);
}
