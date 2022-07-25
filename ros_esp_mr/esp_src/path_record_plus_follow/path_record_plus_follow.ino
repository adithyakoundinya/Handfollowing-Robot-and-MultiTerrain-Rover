#include <EEPROM.h>
//#include <std_msgs/Int16.h>
#include "BluetoothSerial.h"
#define RESET_PIN      12
                  

#define R_F_BACK  33
#define R_F_FORW   32


#define L_F_BACK   25
#define L_F_FORW   26

#define FWD       'F' // go forward(2)
#define LFT       'L' // go left(4)
#define RGT       'R' // go right(6)
#define BWD       'B' // go backward(8)
#define STOP      '1' // stop(0)
#define RPEAT     '3' // repeat the stored sequence of movement from the temporary memory(automatically stores) (REPT)
#define DEL       '2' // delete the stored sequence of movement from temporary memory(EQ)
#define PERST     '4' // copy the sequence from temp. memory to the peramanent memory(EEPROM)
#define PLAYEPROM '5' // repeat the sequence stored in EEPROM(FASTFWD)
#define RESET     'q' // Resets the Arduino Board(RED)
//                  
//#define FWD       70 // go forward(2)
//#define LFT       76 // go left(4)
//#define RGT       82 // go right(6)
//#define BWD       66 // go backward(8)
//#define STOP      83 // stop(0)
//#define RPEAT     88 // repeat the stored sequence of movement from the temporary memory(automatically stores) (REPT)
//#define DEL       87 // delete the stored sequence of movement from temporary memory(EQ)
//#define PERST     85 // copy the sequence from temp. memory to the peramanent memory(EEPROM)
//#define PLAYEPROM 86 // repeat the sequence stored in EEPROM(FASTFWD)
//#define RESET     113 // Resets the Arduino Board(RED)

//counter for counting the number of times program pass through a movement function(fwd, lft etc.)
int fwd_Counter = -1;
int lft_Counter = -1;
int rgt_Counter = -1;
int bwd_Counter = -1;
int stp_Counter = -1;
static int flag = 0;
static int flagStop = 0;

//global "current time" variables for different movement functions(fwd, lft etc.)
unsigned long int current_Time0 = 0;// for FWD movement
unsigned long int current_Time1 = 0;// for LEFT movement
unsigned long int current_Time2 = 0;// for RIGHT movement
unsigned long int current_Time3 = 0;// for BWD movement
unsigned long int current_Time4 = 0;// for STOP

//total time spend by the pgm in executing the movement(fwd, lft etc.) for a particular movement counter
unsigned long int total_Fwd_Time[200];
unsigned long int total_Lft_Time[200];
unsigned long int total_Rgt_Time[200];
unsigned long int total_Bwd_Time[200];
unsigned long int total_Stp_Time[200];

byte seq = 0; //stores the current number of executed sequences
byte seq_Array[1000];// array to store the movement sequence in terms of integers(1 for FWD, 2 for LEFT and so on..)

BluetoothSerial SerialBT;


char receivedChar;
unsigned long int value = 0; // stores the incoming hex value
//
//const uint8_t R_F_PWM = 12;
//const uint8_t R_F_BACK = 33;
//const uint8_t R_F_FORW = 32;
//
//const uint8_t L_F_PWM = 13;
//const uint8_t L_F_BACK = 25;
//const uint8_t L_F_FORW = 26;
//

const uint8_t channel_L =0;
const uint8_t channel_R= 1;


int count_r=0;
int count_l=0;

float left_wheel;
float right_wheel;


char command;

void setup(){
    Serial.begin(115200);
    SerialBT.begin("ESP32");
    pinMode(L_F_FORW, OUTPUT);
    pinMode(L_F_BACK, OUTPUT);
 
    pinMode(R_F_FORW, OUTPUT);
    pinMode(R_F_BACK, OUTPUT);

}


//void speed (){
//    ledcWrite(channel_R, 300);  
//    ledcWrite(channel_L, 300);
//}

//void stop()
//{
//   
//   
//   ledcWrite(channel_R, 0);  
//   ledcWrite(channel_L, 0);
//}
//
//


//void pin_defination(){
//    
//    
//  const int freq = 5000;
//  const int res = 8;
//
//  //pinMode(L_F_PWM,  OUTPUT);
//  pinMode(L_F_FORW, OUTPUT);
//  pinMode(L_F_BACK, OUTPUT);
//  //pinMode(R_F_PWM,  OUTPUT);
//  pinMode(R_F_FORW, OUTPUT);
//  pinMode(R_F_BACK, OUTPUT);
//
//  
//  ledcSetup(channel_R ,freq , res);
//  ledcSetup(channel_L ,freq , res);
//
//  ledcAttachPin(R_F_PWM,channel_R);
//  ledcAttachPin(L_F_PWM,channel_L);
//
//}

void check_Inst(long int value) {

  switch (value) {
    case FWD:
      go_Forward();
      delay(10);
      break;
    case LFT:
      go_Left();
      delay(10);
      break;
    case RGT:
      go_Right();
      delay(10);
      break;
    case BWD:
      go_Backward();
      delay(10);
      break;
    case STOP:
      go_Stop();
      delay(10);
      break;
    case RPEAT:
      go_In_Seq();
      delay(10);
      break;
    case DEL:
      del_From_Local_Mem();
      delay(10);
      break;
    case PERST:
//      write_To_Permt_Mem();
      stop_Override();
      delay(10);
      break;  
    case PLAYEPROM:
      Read_Permt_Mem();
      delay(10);
      break;   
    case RESET:
      pinMode(RESET_PIN, OUTPUT);
      digitalWrite(RESET_PIN,HIGH);   
      break;
                
    default:
       value = 0;
  }
}

void go_Forward() {
  Serial.println(value);
  movement_Inst_Fwd();
 // delay(1000);
  current_Time0 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 1:
      total_Fwd_Time[fwd_Counter + 1] = (current_Time1 - current_Time0 );
      fwd_Counter++;
      break;
      
    case 2:
      // total time elaspsed since Left button is pressed including rest time 
      total_Lft_Time[lft_Counter + 1] = (current_Time0 - current_Time1 );
      lft_Counter++;
      break;

    case 3:
      // total time elaspsed since Right button is pressed including rest time 
      total_Rgt_Time[rgt_Counter + 1] = (current_Time0 - current_Time2 );
      rgt_Counter++;
      break;

    case 4:
      total_Bwd_Time[bwd_Counter + 1] = (current_Time0 - current_Time3 );
      bwd_Counter++;
      break;

    case 5:
      total_Stp_Time[stp_Counter + 1] = (current_Time0 - current_Time4 );
      stp_Counter++;
      break;
  }

  seq_Array[seq] = 1;
  seq++;
}

void go_Left() {
  movement_Inst_Lft();
//  delay(1000);
  current_Time1 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 1:
      total_Fwd_Time[fwd_Counter + 1] = (current_Time1 - current_Time0 );
      fwd_Counter++;
      break;

    case 2:
      // total time elaspsed since Left button is pressed including rest time 
      total_Lft_Time[lft_Counter + 1] = (current_Time0 - current_Time1 );
      lft_Counter++;
      break;

    case 3:
      total_Rgt_Time[rgt_Counter + 1] = (current_Time1 - current_Time2 );
      rgt_Counter++;
      break;

    case 4:
      total_Bwd_Time[bwd_Counter + 1] = (current_Time1 - current_Time3 );
      bwd_Counter++;
      break;

    case 5:
      total_Stp_Time[stp_Counter + 1] = (current_Time1 - current_Time4 );
      stp_Counter++;
      break;
  }

  seq_Array[seq] = 2;
  seq++;
}

void go_Right() {
  movement_Inst_Rgt();
//  delay(1000);
  current_Time2 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 1:
      total_Fwd_Time[fwd_Counter + 1] = (current_Time2 - current_Time0 );
      fwd_Counter++;
      break;

    case 2:
      total_Lft_Time[lft_Counter + 1] = (current_Time2 - current_Time1 );
      lft_Counter++;
      break;

    case 3:
      total_Rgt_Time[rgt_Counter + 1] = (current_Time1 - current_Time2 );
      rgt_Counter++;
      break;

    case 4:
      total_Bwd_Time[bwd_Counter + 1] = (current_Time2 - current_Time3 );
      bwd_Counter++;
      break;

    case 5:
      total_Stp_Time[stp_Counter + 1] = (current_Time2 - current_Time4 );
      stp_Counter++;
      break;
  }

  seq_Array[seq] = 3;
  seq++;
}

void go_Backward() {
  movement_Inst_Bwd();
 // delay(1000);
  current_Time3 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 1:
      total_Fwd_Time[fwd_Counter + 1] = (current_Time3 - current_Time0 );
      fwd_Counter++;
      break;

    case 2:
      total_Lft_Time[lft_Counter + 1] = (current_Time3 - current_Time1 );
      lft_Counter++;
      break;

    case 3:
      total_Rgt_Time[rgt_Counter + 1] = (current_Time3 - current_Time2 );
      rgt_Counter++;
      break;

    case 4:
      total_Bwd_Time[bwd_Counter + 1] = (current_Time2 - current_Time3 );
      bwd_Counter++;
      break;

    case 5:
      total_Stp_Time[stp_Counter + 1] = (current_Time3 - current_Time4 );
      stp_Counter++;
      break;
  }

  seq_Array[seq] = 4;
  seq++;
}

void go_Stop() {
  movement_Inst_Stp();
//  delay(500);
  current_Time4 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 1:
      total_Fwd_Time[fwd_Counter + 1] = (current_Time4 - current_Time0 );
      fwd_Counter++;
      break;

    case 2:
      total_Lft_Time[lft_Counter + 1] = (current_Time4 - current_Time1 );
      lft_Counter++;
      break;

    case 3:
      total_Rgt_Time[rgt_Counter + 1] = (current_Time4 - current_Time2 );
      rgt_Counter++;
      break;

    case 4:
      total_Bwd_Time[bwd_Counter + 1] = (current_Time4 - current_Time3 );
      bwd_Counter++;
      break;

    case 5:
      total_Stp_Time[stp_Counter + 1] = (current_Time3 - current_Time4 );
      stp_Counter++;
      break;
  }

  seq_Array[seq] = 5;
  seq++;
}

void go_In_Seq(void) {
  if(flag != 0){
    goto skip;
  }

  flag = 1;
  Serial.println(current_Time0);
  Serial.println(current_Time1);
  Serial.println(current_Time2);
  Serial.println(current_Time3);
  Serial.println(current_Time4);
  value = 0;
  for (int i = 0; i < (seq + 1); i++) {
    Serial.print(seq_Array[i]);
    Serial.print("    ");
  }
  for (int i = 0; i < (seq + 1); i++) {
    int value1 = 0;
    value1 = seq_Array[i];
    Serial.println(value1);
    switch (value1) {
      case 1:
        if(flagStop != 0){
          goto skip;
        }
        static int j = 0;
        go_Forward_Seq(j);
        j++;
        break;
      case 2:
        if(flagStop != 0){
          goto skip;
        }
        static int k = 0;
        go_Left_Seq(k);
        k++;
        break;
      case 3:
        if(flagStop != 0){
          goto skip;
        }
        static int l = 0;
        go_Right_Seq(l);
        l++;
        break;
      case 4:
        if(flagStop != 0){
          goto skip;
        }
        static int m = 0;
        go_Backward_Seq(m);
        m++;
        break;
      case 5:
        if(flagStop != 0){
          goto skip;
        }
        static int n = 0;
        go_Stop_Seq(n);
        n++;
        break;
      default:
        j = 0; k = 0; l = 0; m = 0; n = 0;
    }
  }

  skip: Serial.println("skipped.");
}

void del_From_Local_Mem() {
//  //set the movement counters to their default values
//  fwd_Counter = -1;
//  lft_Counter = -1;
//  rgt_Counter = -1;
//  bwd_Counter = - 1;
//  stp_Counter = - 1;
//
//  //set the total movement time to its default value
//  for (int i = 0; i < 10; i++) {
//    total_Fwd_Time[i] = 0;
//    total_Lft_Time[i] = 0;
//    total_Rgt_Time[i] = 0;
//    total_Bwd_Time[i] = 0;
//    total_Stp_Time[i] = 0;


  // Reset the sequence array(stored movement instructions)
  for (int i = 0; i < 1000; i++) {
    seq_Array[i] = 0;
  }
  for (int i = 0; i < 200; i++) {
    total_Fwd_Time[i] = 0;
  }
  for (int i = 0; i < 200; i++) {
    total_Lft_Time[i] = 0;
  }
  for (int i = 0; i < 200; i++) {
    total_Rgt_Time[i] = 0;
  }
  for (int i = 0; i < 200; i++) {
    total_Bwd_Time[i] = 0;
  }
  for (int i = 0; i < 200; i++) {
    total_Stp_Time[i] = 0;
  }

  seq = 0;
  flag = 0;
  flagStop = 0;
  current_Time0 = 0;
  current_Time1 = 0;
  current_Time2 = 0;
  current_Time3 = 0;
  current_Time4 = 0;
  
  fwd_Counter = -1;
  lft_Counter = -1;
  rgt_Counter = -1;
  bwd_Counter = -1;
  stp_Counter = -1;

  Serial.println("Reset!");

}


void write_To_Permt_Mem(){
  // total number of movement is stored in a random address i.e, 100
  EEPROM.write(100,seq);
    
  //writing the movement sequence
  for(int i=0; i<seq; i++){ 
  EEPROM.write(2*i,seq_Array[i]);
  }

  //storing the time bw two successive movements
  for(int i=1; i<seq+1; i++){           
  if(seq_Array[i-1]==1){
    static byte a=0;
    EEPROM.write(2*i-1,(total_Fwd_Time[a])/1000);// Note: One location can store maximum value of 255, hence the time is divided by 1000 here. And then multiplied by 1000 while retreiving the data from EEPROM location
    a++;
    }
  else if(seq_Array[i-1]==2){
    static byte b=0;
    EEPROM.write(2*i-1,(total_Lft_Time[b])/1000);
    b++;
    }
  else if(seq_Array[i-1]==3){
    static byte c=0;
    EEPROM.write(2*i-1,(total_Rgt_Time[c])/1000);
    c++;
    }
  else if(seq_Array[i-1]==4){
    static byte d=0;
    EEPROM.write(2*i-1,(total_Bwd_Time[d])/1000);  
    d++;
    }
  else if(seq_Array[i-1]==5){
    static byte e=0;
    EEPROM.write(2*i-1,(total_Stp_Time[e])/1000);  
    e++;
    }             
  }
 } 

 
/************
     This function reads the stored sequence from the EEPROM(permanent memory)
************/

void Read_Permt_Mem(){
  // Read from permanent memory
   byte x = EEPROM.read(100);
   for(int i=0; i<x+1; i++){
    byte r = EEPROM.read(2*i);
    switch(r){
      case 1:
        movement_Inst_Fwd();
        break;
      case 2:
        movement_Inst_Lft();
        break;
      case 3:
        movement_Inst_Rgt();
        break;
      case 4:
        movement_Inst_Bwd();
        break; 
      case 5:
        movement_Inst_Stp();
        break;                          
      }
     delay((EEPROM.read(i+1))*1000);    // multiplied by thousand because the original time was divided by 1000 while storing in EEPROM.
    }
  }
 
/************
     These function moves the car in a direction for the time specified/stored in the total_x_time array
************/
void go_Forward_Seq(int j) {
  Serial.print("go in forward direction sequence");
  movement_Inst_Fwd();
  delay(total_Fwd_Time[j]);
}

void go_Left_Seq(int k) {
  Serial.print("go in Left direction sequence");
  movement_Inst_Lft();
  delay(total_Lft_Time[k]);
}

void go_Right_Seq(int l) {
  Serial.print("go in right direction sequence");
  movement_Inst_Rgt();
  delay(total_Rgt_Time[l]);
}

void go_Backward_Seq(int m) {
  Serial.print("go in backward direction sequence");
  movement_Inst_Bwd();
  delay(total_Bwd_Time[m]);
}

void go_Stop_Seq(int n) {
  Serial.print("go in Stop sequence");
  movement_Inst_Stp();
  delay(total_Stp_Time[n]);
}

void stop_Override() {
  flagStop = 1;
}

/***********
          These movement instruction are repeated(required) several times in the code
************/
void movement_Inst_Fwd(void) {
  // forward movement instructions
  Serial.println("Going_Forward");
    digitalWrite(R_F_FORW, HIGH);
    digitalWrite(L_F_FORW, HIGH);

    digitalWrite(R_F_BACK, LOW);
    digitalWrite(L_F_BACK, LOW);
//    delay(500);

}

void movement_Inst_Lft(void) {
  // Left movement instructions
  Serial.println("Going_Left");
    digitalWrite(R_F_FORW, HIGH);
    digitalWrite(L_F_FORW, LOW);

    digitalWrite(R_F_BACK, LOW);
    digitalWrite(L_F_BACK, LOW);
    

 //   delay(500);
  // NOTE: The minimum delay for RIGHT/LEFT movement is 1S(inluding .5s ON time & .5s OFF time). Hence subtract 1s before repeating this movement
}

void movement_Inst_Rgt(void) {
  // Rgt movement instructions
    Serial.println("Going_Right"); 
    digitalWrite(R_F_FORW, LOW);
    digitalWrite(L_F_FORW, HIGH);

    digitalWrite(R_F_BACK, LOW);
    digitalWrite(L_F_BACK, LOW);

 //   delay(500);
  // NOTE: The minimum delay for RIGHT/LEFT movement is 1S(inluding .5s ON time & .5s OFF time). Hence subtract 1s before repeating this movement 

}

void movement_Inst_Bwd(void) {
  // Bwd movement instructions
    Serial.println("Going_Backward"); 
    digitalWrite(R_F_FORW, LOW);
    digitalWrite(L_F_FORW, LOW);
//    digitalWrite(R_M_FORW, LOW);
//    digitalWrite(L_M_FORW, LOW);
//    digitalWrite(R_B_FORW, LOW);
//    digitalWrite(L_B_FORW, LOW);


    digitalWrite(R_F_BACK, HIGH);
    digitalWrite(L_F_BACK, HIGH);
//    digitalWrite(R_M_BACK, HIGH);
//    digitalWrite(L_M_BACK, HIGH);
//    digitalWrite(R_B_BACK, HIGH);
//    digitalWrite(L_B_BACK, HIGH);
//    delay(500);
}

void movement_Inst_Stp(void) {
  // Stp movement instructions
  Serial.println("Stopping");
  digitalWrite(R_F_BACK, LOW);
  digitalWrite(R_F_FORW, LOW);
  digitalWrite(L_F_BACK, LOW);
  digitalWrite(L_F_FORW, LOW);

}


void loop(){
  receivedChar = (char)SerialBT.read();
  if(Serial.available() > 0){
    SerialBT.write(Serial.read());    
//    delay(50);
  }

  
  if(SerialBT.available()) {
    Serial.print("Recieved:");
    Serial.println(receivedChar);
    value = receivedChar;
    Serial.println(value);
    check_Inst(value);
    delay(90);


    
}

    
  
  }
