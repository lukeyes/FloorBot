
const byte LEFT_MOTOR = 2;
const byte RIGHT_MOTOR = 1;
const int MAX_SIGNAL_DELAY = 5000;

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

// For how to configure the Sabertooth, see the DIP Switch Wizard for
//   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
// Be sure to select Simplified Serial Mode for use with this library.
// This sample uses a baud rate of 9600.
//
// Connections to make:
//   Arduino TX->11  ->  Sabertooth S1
//   Arduino GND    ->  Sabertooth 0V
//   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
//
                              
int leftMotorPower;
int rightMotorPower;
int leftStickY;
int rightStickY;

int lastLeftMotorPower;
int lastRightMotorPower;

volatile unsigned long currentMs;
volatile unsigned long stopMs;


void setMotors(int leftMotorPower, int rightMotorPower) {

  // set motor 1 to left range
  ST.motor(LEFT_MOTOR,leftMotorPower);
  
  // set motor 2 to right range
  ST.motor(RIGHT_MOTOR, rightMotorPower);

  delay(20);
}


void setup() {

 // delay(2000);
  leftMotorPower = 0;
  rightMotorPower = 0;  
  Serial.begin(9600);
  SWSerial.begin(9600);

   currentMs = millis();
   
  stopMs = currentMs;
}

void loop() 
{ 
  currentMs = millis();
  
    // if serial should be read
    if(Serial.available() >= 2) {
      byte command[2];
      Serial.readBytes(command, 2);
      leftMotorPower = command[0]-127;
      rightMotorPower = command[1]-127;    
      stopMs = currentMs + MAX_SIGNAL_DELAY;
    }

 // if no serial communication 
 
  if(currentMs > stopMs) {
    rightMotorPower = 0;
    leftMotorPower = 0;
  }

  setMotors(leftMotorPower, rightMotorPower);
}
