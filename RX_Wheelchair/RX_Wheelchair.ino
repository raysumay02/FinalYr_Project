#include <esp_now.h>
#include <WiFi.h>
#include<bits/stdc++.h>

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 0

#define RIGHT_SIDE_MOTOR 0
#define LEFT_SIDE_MOTOR 1


struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
  int pinEn; 
  int pwmSpeedChannel;
};


std::vector<MOTOR_PINS> motorPins = 
{
  {16, 17, 22, 4},  //RIGHT_SIDE_MOTOR PIN CONFIG 
  {18, 19, 23, 5},  //LEFT_SIDE_MOTOR ****FOLLOW In1, In2, pinEn, pwmSpeed*****
 
};

#define MAX_MOTOR_SPEED 200

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
unsigned long lastRecvTime = 0;

struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
};
PacketData receiverData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue + "  " + receiverData.zAxisValue;
  Serial.println(inputData);

  if (receiverData.yAxisValue < 75)
  {
    Serial.println("****Moving Forward*****");
    processCarMovement(FORWARD);
  }
  else if (receiverData.yAxisValue > 175)
  {
    Serial.println("****Moving Backward*****");
    processCarMovement(BACKWARD);    
  }
  else if (receiverData.xAxisValue > 175)
  {
    Serial.println("****Moving Right*****");
    processCarMovement(RIGHT);   
  }
  else if (receiverData.xAxisValue < 75)
  {
    Serial.println("****Moving Left*****");
    processCarMovement(LEFT);    
  } 
  else
  {
    processCarMovement(STOP);     
  }

  lastRecvTime = millis();
}

void processCarMovement(int inputValue)
{
  switch(inputValue)
  {
    case FORWARD:
      rotateMotor(LEFT_SIDE_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(RIGHT_SIDE_MOTOR, MAX_MOTOR_SPEED);                 
      break;
  
    case BACKWARD:
      rotateMotor(LEFT_SIDE_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(RIGHT_SIDE_MOTOR, -MAX_MOTOR_SPEED);   
      break;
  
    case LEFT:
      rotateMotor(RIGHT_SIDE_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(LEFT_SIDE_MOTOR, -MAX_MOTOR_SPEED);  
      break;
  
    case RIGHT:
      rotateMotor(RIGHT_SIDE_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(LEFT_SIDE_MOTOR, MAX_MOTOR_SPEED); 
      break;

    default:
      rotateMotor(LEFT_SIDE_MOTOR, STOP);
      rotateMotor(RIGHT_SIDE_MOTOR, STOP);   
      break;
  }
}

void rotateMotor(int motorNumber, int motorSpeed)
{
  if (motorSpeed < 0)
  {
    //PROCESS REVERSE MOVEMENT
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);    
  }
  else if (motorSpeed > 0)
  {
    //PROCESS FORWARD MOVEMENT
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);       
  }
  else
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);      
  }
  
  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, abs(motorSpeed));
}                                                

void setUpPinModes()
{
  for (int i = 0; i < motorPins.size(); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);  
    //Set up PWM for motor speed
    ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);  
    ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);     
    rotateMotor(i, STOP);  
  }
}

void setup() 
{
  setUpPinModes();

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop(){
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    processCarMovement(STOP); 
  }
}