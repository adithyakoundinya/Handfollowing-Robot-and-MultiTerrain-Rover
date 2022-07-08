/*
 * rosserial Subscriber Example using TCP on Arduino Shield (Wiznet W5100 based)
 * Blinks an LED on callback
 */
#include <SPI.h>
#include <Ethernet.h>
#include "WiFi.h"

#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

// Shield settings
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
//IPAddress ip(192, 168, 0, 177);

// Server settings
IPAddress server(192, 168, 43, 55);
uint16_t serverPort = 11411;

const char* ssid = "<yourWifiName>";
const char* password = "yourWiFiPassword";

const uint8_t ledPin = 2; // 13 already used for SPI connection with the shield

void messageCb( const std_msgs::Empty&){
  Serial.println("Recieved packet");
  digitalWrite(ledPin, 1);   // blink the led
  delay(500);
  digitalWrite(ledPin, 0);   // blink the led

}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{
//  Ethernet.begin(mac, ip);
//  // give the Ethernet shield a second to initialize:
//  delay(1000);
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {delay(500);Serial.print(".");}
  Serial.println(WiFi.SSID());
  Serial.println(WiFi.localIP());
}
