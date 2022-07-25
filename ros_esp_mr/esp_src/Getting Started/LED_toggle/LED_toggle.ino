/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
// #define LED_BUILTIN 2

ros::NodeHandle  nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  if(digitalRead(LED_BUILTIN) == HIGH){
    delay(2000);
    digitalWrite(LED_BUILTIN, 0);
  }
  else{
    digitalWrite(LED_BUILTIN, 1);
  }

  // digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{ 
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
