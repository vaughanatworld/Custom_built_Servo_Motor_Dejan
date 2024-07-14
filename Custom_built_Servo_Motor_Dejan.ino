// file: Custom_built_Servo_Motor_Dejan.ino
// 20240714 bav initial file from www.howtomechatronics.com/projects/how-to-turn-any-dc-motor-into-a-servo-motor


/*
 *    Custom-built Servo Motor - Arduino Code
 *    by Dejan, www.HowToMechatronics.com
 * 
 *   Libraries:
 *   AS5600 encoder: https://github.com/RobTillaart/AS5600
 *   PID conroller: https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
 */


#include "AS5600.h"
#include "Wire.h"
#include <PID_v1.h>


AS5600 as5600;   //  use default Wire

double Pk1 = 2;  //speed it gets there
double Ik1 = 0;
double Dk1 = 0.025;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

PID myPID(&Input, &Output, &Setpoint, Pk1, Ik1, Dk1, DIRECT);

#define motor_IN1 5
#define motor_IN2 6
#define ch1 2
#define centerSet 7
#define inputSwitch 3
#define modeSwitch 4

int ch1Value;

int encoderValue, inputValue, pwmValue;
String inString = "";  // string to hold input

int centerAngle = 2047; // 180 degrees
int angleDifference = 0;
int angleValue = 0;
int leftLimit = 30;
int rightLimit = 4067;
int rangeAdjustment = 0;
float sensitivityAdjustment = 0;
float angle = 0;

int quadrantNumber = 2;
int previousQuadrantNumber = 3;
int numberOfTurns = 0;
float totalAngle = 0;

int error = 0;

char incomingByte = 0;
int intInput = 0;

//------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  Wire.begin();

  pinMode(motor_IN1, OUTPUT);
  pinMode(motor_IN2, OUTPUT);
  // Activate the Arduino internal pull-up resistors
  pinMode(centerSet, INPUT_PULLUP);
  pinMode(inputSwitch, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  myPID.SetMode(AUTOMATIC);              // PID Setup
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(20);

}

//------------------------------------------
void loop() {
  // Read encoder value - current position
  encoderValue = as5600.readAngle();

  // Continuous rotation mode
  if (digitalRead(modeSwitch) == 0) {
    // Enter desired angle for the servo to go to through the serial monitor
    while (Serial.available() > 0) {
      int inChar = Serial.read();
      if (isDigit(inChar)) {
        // convert the incoming byte to a char and add it to the string:
        inString += (char)inChar;
      }
      // if you get a newline, print the string, then the string's value:
      if (inChar == '\n') {
        Setpoint = inString.toInt(); // Setpoint - desired angle
        // clear the string for new input:
        inString = "";
      }
    }
    if (digitalRead(inputSwitch) == 0) { // Potentiometer as input
      inputValue = analogRead(A0);
      if (inputValue < 400) {
        Setpoint = Setpoint - 0.3;
      }
      if (inputValue < 300) {
        Setpoint = Setpoint - 0.3;
      }
      if (inputValue < 200) {
        Setpoint = Setpoint - 0.3;
      }

      if (inputValue > 600) {
        Setpoint = Setpoint + 0.3;
      }
      if (inputValue > 700) {
        Setpoint = Setpoint + 0.3;
      }

      if (inputValue > 800) {
        Setpoint = Setpoint + 0.3;
      }
    }
    else if (digitalRead(inputSwitch) == 1) {
      inputValue = pulseIn(ch1, HIGH, 30000); // RC receiver as input
      if (inputValue < 1450) {
        Setpoint--;
      }
      if (inputValue < 1350) {
        Setpoint--;
      }
      if (inputValue < 1250) {
        Setpoint--;
      }
      if (inputValue < 1150) {
        Setpoint--;
      }
      if (inputValue > 1550) {
        Setpoint++;
      }
      if (inputValue > 1650) {
        Setpoint++;
      }
      if (inputValue > 1750) {
        Setpoint++;
      }
      if (inputValue > 1850) {
        Setpoint++;
      }
    }

    // Convert encoder RAW values into angle value
    angle = encoderValue * 0.087890625;
    // Quadrant 1
    if (angle >= 0 && angle <= 90) {
      quadrantNumber = 1;
    }
    // Quadrant 2
    if (angle >= 90 && angle <= 180) {
      quadrantNumber = 2;
    }
    // Quadrant 3
    if (angle >= 180 && angle <= 270) {
      quadrantNumber = 3;
    }
    // Quadrant 4
    if (angle >= 270 && angle <= 360) {
      quadrantNumber = 4;
    }

    if (quadrantNumber != previousQuadrantNumber) {
      // Transition from 4th to 1st quadrant
      if (quadrantNumber == 1 && previousQuadrantNumber == 4) {
        numberOfTurns++;
      }
      // Transition from 1st to 4th quadrant
      if (quadrantNumber == 4 && previousQuadrantNumber == 1) {
        numberOfTurns--;
      }
      previousQuadrantNumber = quadrantNumber;
    }
    if (totalAngle >= 0) {
      totalAngle = (numberOfTurns * 360) + angle;
    }
    else {
      totalAngle = (numberOfTurns * 360) + angle;
    }

    // Establish Input value for PID
    Input = totalAngle;
  }

  // Limited Rotation Mode
  else if (digitalRead(modeSwitch) == 1) {

    rangeAdjustment = analogRead(A1);
    leftLimit = 0 + 30 + rangeAdjustment;
    rightLimit = 4097 - 30 - rangeAdjustment;


    if (digitalRead(inputSwitch) == 0) {  // Analog input - Potentiometer
      // Get value from potentiometer
      inputValue = analogRead(A0);
      if (inputValue < 15) {
        inputValue = 15;
      }
      if (inputValue > 1008) {
        inputValue = 1008;
      }
      Setpoint = map(inputValue, 15, 1008, -255, 255);
    }
    else if (digitalRead(inputSwitch) == 1) {  // Digital input - RC transmitter
      inputValue = pulseIn(ch1, HIGH, 30000); // Read RC receiver as input
      Setpoint = map(inputValue, 1000, 2000, -255, 255);
    }

    // Set center angle
    if (digitalRead(centerSet) == LOW) {
      centerAngle = encoderValue;
      angleDifference = 2047 - encoderValue;
      delay(1000);
    }
    // Adjust angle value according to the center point (angleDifference)
    if (centerAngle < 2047) {
      angleValue = encoderValue + angleDifference;
      if (encoderValue < 4097 && encoderValue > (4096 - angleDifference)) {
        angleValue = encoderValue - 4097 + angleDifference;
      }
    }
    if (centerAngle > 2047) {
      angleValue = encoderValue + angleDifference;
      if (encoderValue >= 0 && encoderValue < abs(angleDifference)) {
        angleValue = encoderValue + 4097 + angleDifference;
      }
    }
    else if (centerAngle == 2047) {
      angleValue = encoderValue;
    }

    // Establish Input value for PID
    Input = map(angleValue , leftLimit, rightLimit, -255, 255);
  }
  // Adjusting sensitivity
  Pk1 = analogRead(A2) * 0.002;
  myPID.SetTunings(Pk1, Ik1, Dk1);

  // Run PID process to get Output value
  myPID.Compute();
  // Move right
  if (Output > 1 ) {
    pwmValue = Output;

    if (pwmValue < 30 && pwmValue > 5) {
      pwmValue = pwmValue + 30;
    }
    if (pwmValue <= 5) {
      pwmValue = 0;
    }
    digitalWrite(motor_IN1, LOW);
    analogWrite(motor_IN2, pwmValue);
  }
  // Move left
  else if (Output < 1 ) {
    pwmValue = abs(Output);

    if (pwmValue < 30 && pwmValue > 5) {
      pwmValue = pwmValue + 30;
    }
    if (pwmValue <= 5) {
      pwmValue = 0;
    }
    analogWrite(motor_IN1, pwmValue);
    digitalWrite(motor_IN2, LOW);
  }
  // Do not move
  else if (Output > -1 && Output < 1) {
    pwmValue = 0;
    digitalWrite(motor_IN1, LOW);
    digitalWrite(motor_IN2, LOW);
  }

  //Serial.print(Setpoint);
  //Serial.print("\t");
  //Serial.println(totalAngle);
}

//------------------------------------------
