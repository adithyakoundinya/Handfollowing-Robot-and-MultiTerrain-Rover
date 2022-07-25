#include<NewPing.h>           
#include<Servo.h>             
#include<AFMotor.h> 

#define RIGHT A2              // Right IR sensor connected to analog pin A2 of Arduino Uno:
#define LEFT A1               // Left IR sensor connected to analog pin A3 of Arduino Uno:
#define TRIGGER_PIN A4        // Trigger pin connected to analog pin A1 of Arduino Uno:
#define ECHO_PIN A5           // Echo pin connected to analog pin A0 of Arduino Uno:
#define MAX_DISTANCE 200      // Maximum ping distance:

unsigned int distance = 0;    //Variable to store ultrasonic sensor distance:
unsigned int Right_Value = 0; //Variable to store Right IR sensor value:
unsigned int Left_Value = 0;  //Variable to store Left IR sensor value:
  

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  //NewPing setup of pins and maximum distance:

//initial motors pin
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ); 
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);


char command; 

void setup() 
{       
  Serial.begin(9600);  //Set the baud rate to your Bluetooth module.
   pinMode(RIGHT, INPUT); //set analog pin RIGHT as an input:
   pinMode(LEFT, INPUT);  //set analog pin RIGHT as an input:
   
}

char mode = 'x';

void loop(){

  if(Serial.available() > 0){ 
    command = Serial.read();
    if(command == 'X' || command == 'x'){
      mode = command;
    }
    Serial.println(mode);
    Serial.println(command);
    
  
  if(mode=='x')
  { 
    Stop(); //initialize with motors stoped
    //Change pin mode only if new command is different from previous.   
    //Serial.println(command);
    switch(command){
    case 'F':  
      forward();
      break;
    case 'B':  
       back();
      break;
    case 'L':  
      left();
      break;
    case 'R':
      right();
      break;
    case 'G':
      forwardleft();
      break;
    case 'I':
      forwardright();
      break;
    case 'H':
      backwardleft();
      break;
    case 'J':
      backwardleft();
      break;
    
    }
    }
  }
    
    else if (mode=='X'){
      followMe();     
  }
}


void forward()
{
  motor1.setSpeed(120); //Define maximum velocity
  motor1.run(FORWARD); //rotate the motor clockwise
  motor2.setSpeed(120); //Define maximum velocity
  motor2.run(FORWARD); //rotate the motor clockwise
  motor3.setSpeed(120);//Define maximum velocity
  motor3.run(FORWARD); //rotate the motor clockwise
  motor4.setSpeed(120);//Define maximum velocity
  motor4.run(FORWARD); //rotate the motor clockwise
}

void back()
{
  motor1.setSpeed(120); //Define maximum velocity
  motor1.run(BACKWARD); //rotate the motor anti-clockwise
  motor2.setSpeed(120); //Define maximum velocity
  motor2.run(BACKWARD); //rotate the motor anti-clockwise
  motor3.setSpeed(120); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(120); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
}

void left()
{
  motor1.setSpeed(120); //Define maximum velocity
  motor1.run(BACKWARD); //rotate the motor anti-clockwise
  motor2.setSpeed(120); //Define maximum velocity
  motor2.run(BACKWARD); //rotate the motor anti-clockwise
  motor3.setSpeed(120); //Define maximum velocity
  motor3.run(FORWARD);  //rotate the motor clockwise
  motor4.setSpeed(120); //Define maximum velocity
  motor4.run(FORWARD);  //rotate the motor clockwise
}

void right()
{
  motor1.setSpeed(120); //Define maximum velocity
  motor1.run(FORWARD); //rotate the motor clockwise
  motor2.setSpeed(120); //Define maximum velocity
  motor2.run(FORWARD); //rotate the motor clockwise
  motor3.setSpeed(120); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(120); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
} 

void forwardleft()
{
  motor1.setSpeed(100); //Define maximum velocity
  motor1.run(FORWARD); //rotate the motor clockwise
  motor2.setSpeed(100); //Define maximum velocity
  motor2.run(FORWARD); //rotate the motor clockwise
  motor3.setSpeed(150); //Define maximum velocity
  motor3.run(FORWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(150); //Define maximum velocity
  motor4.run(FORWARD); //rotate the motor anti-clockwise
} 

void forwardright()
{
  motor1.setSpeed(150); //Define maximum velocity
  motor1.run(FORWARD); //rotate the motor clockwise
  motor2.setSpeed(150); //Define maximum velocity
  motor2.run(FORWARD); //rotate the motor clockwise
  motor3.setSpeed(100); //Define maximum velocity
  motor3.run(FORWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(100); //Define maximum velocity
  motor4.run(FORWARD); //rotate the motor anti-clockwise
} 

void backwardright()
{
  motor1.setSpeed(150); //Define maximum velocity
  motor1.run(BACKWARD); //rotate the motor clockwise
  motor2.setSpeed(150); //Define maximum velocity
  motor2.run(BACKWARD); //rotate the motor clockwise
  motor3.setSpeed(150); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(150); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
} 

void backwardleft()
{
  motor1.setSpeed(100); //Define maximum velocity
  motor1.run(BACKWARD); //rotate the motor clockwise
  motor2.setSpeed(100); //Define maximum velocity
  motor2.run(BACKWARD); //rotate the motor clockwise
  motor3.setSpeed(150); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(150); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
} 

void Stop()
{
  motor1.setSpeed(0); //Define minimum velocity
  motor1.run(RELEASE); //stop the motor when release the button
  motor2.setSpeed(0); //Define minimum velocity
  motor2.run(RELEASE); //rotate the motor clockwise
  motor3.setSpeed(0); //Define minimum velocity
  motor3.run(RELEASE); //stop the motor when release the button
  motor4.setSpeed(0); //Define minimum velocity
  motor4.run(RELEASE); //stop the motor when release the button
}

void followMe()
{
    
delay(50);                                        //wait 50ms between pings:
distance = sonar.ping_cm();                       //send ping, get distance in cm and store it in 'distance' variable:
Serial.print("distance");                   
Serial.println(distance);                         // print the distance in serial monitor:


    Right_Value = digitalRead(RIGHT);             // read the value from Right IR sensor:
    Left_Value = digitalRead(LEFT);               // read the value from Left IR sensor:
 
Serial.print("RIGHT");                      
Serial.println(Right_Value);                      // print the right IR sensor value in serial monitor:
Serial.print("LEFT");                       
Serial.println(Left_Value);                       //print the left IR sensor value in serial monitor:

if((distance > 1) && (distance < 15)){            //check wheather the ultrasonic sensor's value stays between 1 to 15.
                                                  //If the condition is 'true' then the statement below will execute:
  //Move Forward:
  motor1.setSpeed(130);  //define motor1 speed:
  motor1.run(FORWARD);   //rotate motor1 clockwise:
  motor2.setSpeed(130);  //define motor2 speed:
  motor2.run(FORWARD);   //rotate motor2 clockwise:
  motor3.setSpeed(130);  //define motor3 speed:
  motor3.run(FORWARD);   //rotate motor3 clockwise:
  motor4.setSpeed(130);  //define motor4 speed:
  motor4.run(FORWARD);   //rotate motor4 clockwise:
  
}else if((Right_Value==0) && (Left_Value==1)) {   //If the condition is 'true' then the statement below will execute:
  
  //Turn Left                                                
  motor1.setSpeed(150);  //define motor1 speed:
  motor1.run(FORWARD);   //rotate motor1 cloclwise:
  motor2.setSpeed(150);  //define motor2 speed:
  motor2.run(FORWARD);   //rotate motor2 clockwise:
  motor3.setSpeed(150);  //define motor3 speed:
  motor3.run(BACKWARD);  //rotate motor3 anticlockwise:
  motor4.setSpeed(150);  //define motor4 speed:
  motor4.run(BACKWARD);  //rotate motor4 anticlockwise:
  delay(150);
  
}else if((Right_Value==1)&&(Left_Value==0)) {     //If the condition is 'true' then the statement below will execute:
  
  //Turn Right
  motor1.setSpeed(150);  //define motor1 speed:
  motor1.run(BACKWARD);  //rotate motor1 anticlockwise:
  motor2.setSpeed(150);  //define motor2 speed:
  motor2.run(BACKWARD);  //rotate motor2 anticlockwise:
  motor3.setSpeed(150);  //define motor3 speed:
  motor3.run(FORWARD);   //rotate motor3 clockwise:
  motor4.setSpeed(150);  //define motor4 speed:
  motor4.run(FORWARD);   //rotate motor4 clockwise:
  delay(150);
  
}else if(distance > 15) {                          //If the condition is 'true' then the statement below will execute:
  
  //Stop
  motor1.setSpeed(0);    //define motor1 speed:
  motor1.run(RELEASE);   //stop motor1:
  motor2.setSpeed(0);    //define motor2 speed:
  motor2.run(RELEASE);   //stop motor2:
  motor3.setSpeed(0);    //define motor3 speed:
  motor3.run(RELEASE);   //stop motor3:
  motor4.setSpeed(0);    //define motor4 speed:
  motor4.run(RELEASE);   //stop motor4:
}
}
