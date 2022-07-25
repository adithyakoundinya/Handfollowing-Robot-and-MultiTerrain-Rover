#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <AsyncTCP.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <ESPAsyncWebServer.h>

#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define UP_LEFT 5
#define UP_RIGHT 6
#define DOWN_LEFT 7
#define DOWN_RIGHT 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define STOP 0

#define FRONT_RIGHT_MOTOR 0
#define BACK_RIGHT_MOTOR 1
#define FRONT_LEFT_MOTOR 2
#define BACK_LEFT_MOTOR 3

#define FORWARD 1
#define BACKWARD -1

const int enc_r = 25;
const int enc_l = 26;

const uint8_t R_F_PWM = 12;
const uint8_t R_F_BACK = 19;
const uint8_t R_F_FORW = 21;

const uint8_t L_F_PWM = 13;
const uint8_t L_F_BACK = 2;
const uint8_t L_F_FORW = 4;

const uint8_t R_B_BACK = 14;
const uint8_t R_B_FORW = 27;

const uint8_t L_B_BACK = 32;
const uint8_t L_B_FORW = 33;


const uint8_t R_M_BACK = 22;
const uint8_t R_M_FORW = 23;

const uint8_t L_M_BACK = 5;
const uint8_t L_M_FORW = 18;


const uint8_t channel_L =0;
const uint8_t channel_R= 1;


int count_r=0;
int count_l=0;

float left_wheel;
float right_wheel;
//float b_left_wheel;
//float b_right_wheel;
//float m_left_wheel;
//float m_right_wheel;

IPAddress server(192, 168, 30, 15);
uint16_t serverPort = 11411;
const char*  ssid = "Sanjay's device";
const char*  password = "lightweight";

void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg);
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb );


std_msgs::Int16 enc_r_msg;
std_msgs::Int16 enc_l_msg;
ros::Publisher right_enc("enc_r_values", &enc_r_msg );
ros::Publisher left_enc("enc_l_values", &enc_l_msg );



void setup(){
    Serial.begin(115200);
    setupWiFi();

    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(right_enc);
    nh.advertise(left_enc);
    nh.subscribe(sub);
    pin_defination();

}

void loop(){
    right_enc.publish(&enc_r_msg);
    left_enc.publish(&enc_l_msg);
    nh.spinOnce();
    delay(500);
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
  pinMode(L_B_FORW, OUTPUT);
  pinMode(L_B_BACK, OUTPUT);
  pinMode(R_B_FORW, OUTPUT);
  pinMode(R_B_BACK, OUTPUT);
  pinMode(L_M_FORW, OUTPUT);
  pinMode(L_M_BACK, OUTPUT);
  pinMode(R_M_FORW, OUTPUT);
  pinMode(R_M_BACK, OUTPUT);
  
  ledcSetup(channel_R ,freq , res);
  ledcSetup(channel_L ,freq , res);

  ledcAttachPin(R_F_PWM,channel_R);
  ledcAttachPin(L_F_PWM,channel_L);
    pinMode(enc_r,INPUT);
    pinMode(enc_l,INPUT);
    attachInterrupt(digitalPinToInterrupt(enc_r),Update_encR,CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_l),Update_encL,CHANGE);
}

void Update_encR(){
    enc_r_msg.data=count_r ++;
    
}

void Update_encL(){
    enc_l_msg.data=count_l ++;

}



void direction(){


    digitalWrite(L_F_FORW, left_wheel > 0);
    digitalWrite(L_F_BACK, left_wheel < 0);

    
    digitalWrite(L_M_FORW, left_wheel > 0);
    digitalWrite(L_M_BACK, left_wheel < 0);


    digitalWrite(L_B_FORW, left_wheel > 0);
    digitalWrite(L_B_BACK, left_wheel < 0);


    digitalWrite(R_F_FORW, right_wheel > 0);
    digitalWrite(R_F_BACK, right_wheel < 0);

    
    digitalWrite(R_M_FORW, right_wheel > 0);
    digitalWrite(R_M_BACK, right_wheel < 0);


    digitalWrite(R_B_FORW, right_wheel > 0);
    digitalWrite(R_B_BACK, right_wheel < 0);




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
