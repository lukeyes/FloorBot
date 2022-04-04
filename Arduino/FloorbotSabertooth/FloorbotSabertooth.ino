#include <XBOXRECV.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#include <SabertoothSimplified.h>


const byte LEFT_MOTOR = 2;
const byte RIGHT_MOTOR = 1;
const byte DRIVEDEADZONERANGE = 20;
const byte DRIVESPEED_WALK = 50;
const byte DRIVESPEED_RUN = 127;
const int XBOX_HAT_MIN = -32768;
const int XBOX_HAT_MAX = 32767;
const int MAX_SIGNAL_DELAY = 100;

SoftwareSerial SWSerial(NOT_A_PIN, 3); // RX on no pin (unused), TX on pin 3 (to S1).
SabertoothSimplified saberTooth(SWSerial); // Use SWSerial as the serial port.

                              
int leftMotorPower;
int rightMotorPower;
int leftStickY;
int rightStickY;

int lastLeftMotorPower;
int lastRightMotorPower;

USB usb;
XBOXRECV xBox(&usb);

unsigned long currentMs;
unsigned long stopMs;

int getMotorPowerFromStick(AnalogHatEnum stick, bool isRunButtonDown) 
{
  int motorPower = 0; 
  
   // get stick y of xbox controller   
   int stickY = xBox.getAnalogHat(stick, 0);

   byte motorMagnitude = DRIVESPEED_WALK;
   if( isRunButtonDown) {
    motorMagnitude = DRIVESPEED_RUN;
   }

   motorPower = map(stickY, XBOX_HAT_MIN, XBOX_HAT_MAX, -motorMagnitude, motorMagnitude);
   
   if (motorPower > -DRIVEDEADZONERANGE && motorPower < DRIVEDEADZONERANGE){
      //stick in dead zone 
      motorPower = 0;
   }
   
   // reverse the motor power (direction).  Quickie wheelchair goes reverse direction
   motorPower = motorPower * -1;
   return motorPower;
}

void setMotors(int leftMotorPower, int rightMotorPower) {
    
  // set motor 1 to left range
  saberTooth.motor(LEFT_MOTOR,leftMotorPower);
  
  // set motor 2 to right range
  saberTooth.motor(RIGHT_MOTOR, rightMotorPower);
}

void setup() {

  delay(2000);
  leftMotorPower = 0;
  rightMotorPower = 0;  
//  Serial1.begin(9600);
 // Serial.begin(9600);
// // Serial.println("Setting up serial");
  
  SabertoothTXPinSerial.begin(9600);


  setMotors(0,0);
  pinMode(13, OUTPUT);

  // initialize USB
  if( usb.Init() == -1)
  {
    while(1); // halt
  }
}

void loop() 
{
  usb.Task();

  currentMs = millis();
  
  if( xBox.XboxReceiverConnected && xBox.Xbox360Connected[0])
  {
    bool isRunButtonDown = xBox.getButtonPress(L1, 0);// && xBox.getButtonClick(R1, 0);

    if( isRunButtonDown ) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }
    
    leftMotorPower = getMotorPowerFromStick(LeftHatY, isRunButtonDown);
    rightMotorPower = getMotorPowerFromStick(RightHatY, isRunButtonDown);
    stopMs = currentMs + MAX_SIGNAL_DELAY;
  }
  else
  {
    // if serial should be read
    if(Serial.available() >= 2) {
      char command[2];
      Serial.readBytes(command, 2);
      leftMotorPower =  map(command[0], -127, 127, -DRIVESPEED_RUN, DRIVESPEED_RUN);
      rightMotorPower = map(command[1], -127,127,-DRIVESPEED_RUN, DRIVESPEED_RUN);
      stopMs = currentMs + MAX_SIGNAL_DELAY;
    }
  }

  // if no XBox and no serial communication 
  if(currentMs > stopMs) {
    rightMotorPower = 0;
    leftMotorPower = 0;
    digitalWrite(13, LOW);
  }

  setMotors(leftMotorPower, rightMotorPower);
}
