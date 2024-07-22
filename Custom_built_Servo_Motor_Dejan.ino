// file: Custom_built_Servo_Motor_Dejan.ino
// 20240714 bav initial file from www.howtomechatronics.com/projects/how-to-turn-any-dc-motor-into-a-servo-motor

// 20240722 bav More comments. Changed degrees reference from (-180 to 180) to (0 to 360). I hope I haven't broken it.
// 20240716 bav Added comments, changed some variable names, cleaned up some of the logic.

/*
 *    Custom-built Servo Motor - Arduino Code
 *    by Dejan, www.HowToMechatronics.com
 * 
 *   Libraries:
 *   AS5600 encoder: https://github.com/RobTillaart/AS5600
 *   PID conroller: https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
 *
 *   H bridge: TI DRV8871 logic 3.3v/5V; No enable pin; IN1/IN2 to OUT1/OUT2; 
 *              logic power derived from motor power.
 *
 *      Table 1. H-Bridge Control
 *   IN1 IN2 OUT1    OUT2    DESCRIPTION
 *   0   0   High-Z  High-Z  Coast; H-bridge disabled to High-Z (sleep entered after 1 ms)
 *   0   1   L       H       Reverse (Current OUT2 → OUT1)
 *   1   0   H       L       Forward (Current OUT1 → OUT2)
 *   1   1   L       L       Brake; low-side slow decay
 *
 *    Motor: 12V brushed gear motor 50RPM
 *
 */


#include "AS5600.h"
#include "Wire.h"
#include <PID_v1.h>

// Position sensor
AS5600 as5600;  //  uses default Wire/I2C
/*                  Note: Documentation of AS5600
 *                        Top view of IC. Magnet N pointing a pin 1 end of IC -> 
 *                        0 degrees, count is 0 (N<-)
 *  cw magnet rotation   90 degrees, count 1024 (N^)
 *                      180        ,       2048 (->N)
 *                      270        ,       3072 (Nv)
 *                      360        ,       4096 (N<-)  then roll over?
 * My desire is to flip the AS5600 around so the angle positions match the unit circle.
 * The unit circle places the N pole pointing to the right. The positive rotation is ccw
 * from the x axis. This is achieved by looking at the bottom of the AS5600 rotated so 
 * pin 1 is in the lower right corner. In the finished servo, this perspective looks
 * down the  motor shaft from the encoder end.
 *                        0 degrees, count is 0 (->N)
 *                       90                  1023 (N^)
 *                      180                  2047 (N<-)
 *                      270                  3071 (Nv)
 *                      360                  4095 (->N)
 *
 * The default home position is defined at 180 degrees or AS5600 count 2047.
 */

// PID
double Kp = 2;  // Initial value. The speed with which it gets there. Varied via POT_KP_PIN in real time.
double Ki = 0;
double Kd = 0.025;

// Define Variables we'll be connecting to
double Setpoint;  // range in degrees. No limit on number of 360 degree turns.
double Input;     // range in degrees. No limit on number of 360 degree turns.
double Output;    // range -255 to 255. This is the signed PWM signal fed to the H bridge.

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// GPIO pins on Arduino Uno
#define MOTOR_IN1_PIN 5         // H bridge cw
#define MOTOR_IN2_PIN 6         // H bridge ccw
#define RC_CH1_PIN 2            // Radio Control Channel1. 1000 millisec to 2000 millisec pulse duration at 50Hz
#define POT_POSITION_IN_PIN A0  // Manually set a pot to set the motor position Setpoint. External to PCB
#define POT_RANGE_ADJ_PIN A1    // Manually set a pot to constrain the range of motor rotation between 90-270 to 0-360 degrees
#define POT_KP_PIN A2           // Allow one to play with the Kp of the PID in real time.
#define HOME_SET_SW_PIN 7       // Press this button to set the home position
#define INPUT_SELECTION_SW_PIN 3  // Rocker switches
#define MODE_SELECTION_SW_PIN 4   //

int ch1Value;                   // RC pwm signal

int encoderValue, inputValue, pwmValue;
String inString = "";    // String to hold keyboard input
int centerAngle = 2047;  // 180 degrees. Home position default
int angleDifference = 0;
int angleValue = 0;
int leftLimit = 30; // AS5600 units
int rightLimit = 4067; // AS5600 units
int rangeAdjustment = 0; // POT range 0 to 1024 and in AS5600 units
float sensitivityAdjustment = 0;
float angle = 0;

int quadrantNumber = 2;
int previousQuadrantNumber = 3;
int numberOfTurns = 0;
//bav float totalAngle = 0;

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

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  // Activate the Arduino internal pull-up resistors
  pinMode(HOME_SET_SW_PIN, INPUT_PULLUP);
  pinMode(INPUT_SELECTION_SW_PIN, INPUT_PULLUP);
  pinMode(MODE_SELECTION_SW_PIN, INPUT_PULLUP);

  myPID.SetMode(AUTOMATIC);  // PID Setup
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(20);  // No clock; Just loop time. Assume millisecs. This needs to be established via experimentation.
}

//------------------------------------------
void loop() {
  // Read encoder value - current position
  encoderValue = as5600.readAngle();  // AS5600 units. range 0 to 4095

  // Continuous rotation mode
  if (digitalRead(MODE_SELECTION_SW_PIN) == 0) {  // toggle switch

    // Get the Setpoint from the keyboard. In degrees
    // Enter desired angle for the servo to go to through the serial monitor
    while (Serial.available() > 0) {
      int inChar = Serial.read();
      if (isDigit(inChar)) {
        // convert the incoming byte to a char and add it to the string:
        inString += (char)inChar;
      }
      // if you get a newline, print the string, then the string's value:
      if (inChar == '\n') {
        Setpoint = inString.toInt();  // Setpoint - desired angle in degrees
        // clear the string for new input:
        inString = "";
      }
    }  // end Get Setpoint from keyboard

    if (digitalRead(INPUT_SELECTION_SW_PIN) == 0) {  // Toggle switch. Set pot POT_POSITION_IN_PIN as input.

      // Determine new PID Setpoints from a potentiometer.
      // The mid point rotation of the pot inputValue is 512. 
      // Take count 500 as an approximate midpoint for simplicity.
      // With the pot at the mid point, the Setpoint remains undisturbed and
      // separately, the motor tries to achieve and maintain the existing
      // Setpoint position. This action takes place over may calls to loop().
      //
      // As the user twists the pot knob the Setpoint is increased ccw 
      // or decreased cw (on the unit circle). The further the user rotates
      // the knob the greater the Setpoint is modified. The motor PID will
      // then rotate to match the modified Setpoint.
      // 
      // As the user returns the knob to the center position the Setpoint
      // is no longer modified and the motor stops.
      // 
      // No idea why the Setpoint is modified by 1/3 of a degree at a time and
      // the for the RC receiver the Setpoint is modified by a whole degree. TBD.
      //
      inputValue = analogRead(POT_POSITION_IN_PIN);  // Analog pot units 0-1024

      if (inputValue < 400) {
        Setpoint = Setpoint - 0.3; // Setpoint units are degrees.
      }
      if (inputValue < 300) {
        Setpoint = Setpoint - 0.3;
      }
      if (inputValue < 200) {
        Setpoint = Setpoint - 0.3;
      }
      // Pot mid point 512/500 so the Setpoint is not modified.
      if (600 < inputValue) {
        Setpoint = Setpoint + 0.3;
      }
      if (700 <inputValue) {
        Setpoint = Setpoint + 0.3;
      }
      if (800 < inputValue) {
        Setpoint = Setpoint + 0.3;
      }
      // End get Setpoint manipulation from a potentiometer

    } else if (digitalRead(INPUT_SELECTION_SW_PIN) == 1) {  // toggle switch

      // Modify the Setpoint based upon input from an RC receiver or RC Servo tester.
      // Every 20 millsec a pulse arrives whose duration is 1000 usec to 2000 usec
      // The mid point is 1500 usec. Joystick springs can keep the default at 1500 usec.
      // The joystick is a pot whose voltage is turned into pulse width. So the same
      // logic applies here as before. The further the joystick is moved the greater
      // the modification of the PID Setpoint.
      inputValue = pulseIn(RC_CH1_PIN, HIGH, 30000);  // Time out is 30 millsec.


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
      // Mid point is 1500 usec so no modification to the Setpoint
      if (1550  < inputValue) {
        Setpoint++;
      }
      if (1650 < inputValue) {
        Setpoint++;
      }
      if (1750 < inputValue) {
        Setpoint++;
      }
      if (1850 < inputValue) {
        Setpoint++;
      }
    }  // End get Setpoint from an RC receiver or RC Servo tester.

    // Stop fiddling with the PID Setpoint
    // Start fiddling with the PID Input. Where the
    // motor is actually positioned right now in this
    // call to loop().

    // Convert AS5600 encoder RAW values into angle value in degrees.
    angle = encoderValue * 0.087890625;

    // Determine the quadrant the angle is in
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

    // Determine the "turn" count of the AS5600 away from home position.
    if (quadrantNumber != previousQuadrantNumber) {
      // Transition from 4th to 1st quadrant
      if (quadrantNumber == 1 && previousQuadrantNumber == 4) {
        numberOfTurns++;
      }
      // Transition from 1st to 4th quadrant
      if (quadrantNumber == 4 && previousQuadrantNumber == 1) {
        numberOfTurns--;
      }
      // Record for next loop() call.
      previousQuadrantNumber = quadrantNumber;
    }

    // Determine the wind up in degrees
    // The actual position of the servo, in degrees, at this time.
    // Set as the PID Input variable
    Input = (numberOfTurns * 360) + angle;

  } // end of if (digitalRead(MODE_SELECTION_SW_PIN) == 0) {  // toggle switch


  // Limited Rotation Mode. Servo limited to 0 to 360 degrees, homed on 180 degrees (default)
  //
  else if (digitalRead(MODE_SELECTION_SW_PIN) == 1) {



    if (digitalRead(INPUT_SELECTION_SW_PIN) == 0) {  // Toggle SW. Select a pot as Setpoint input

      // Get desired Setpoint from potentiometer
      inputValue = analogRead(POT_POSITION_IN_PIN); // 0 to 1024. 512 is mid point
      // Extreme values are determined by experimentation.
      inputValue = constrain( inputValue, 15, 1008 );
      
      Setpoint = map(inputValue, 15, 1008, 0, 360); // in degrees.

    } else if (digitalRead(INPUT_SELECTION_SW_PIN) == 1) {  // Toggle switch input - RC transmitter

      inputValue = pulseIn(RC_CH1_PIN, HIGH, 30000);  // Read RC receiver as input. 
      //                                                Unit 1000-2000 usec. mid point 1500 usec
      Setpoint = map(inputValue, 1000, 2000, 0, 360); // in degrees
    }

    //
    // Set home position, ie centerAngle. 
    // Default AS5600 home position is 2047 or 180 degrees on unit circle.

    if (digitalRead(HOME_SET_SW_PIN) == LOW) {  // Momentary push button. Similar to setting the trim on an rc airplane.

      // Keep these values for future runs of loop().
      centerAngle = encoderValue;               // encoderValue is read at top of loop().
      angleDifference = 2047 - encoderValue;    // Difference between AS5600 home position and the new home position.
                                                // Positive in quadrants I and II. Negative in quadrants III and IV
      //delay(1000);                              // ??? why the delay of 1 second?
    }

    // The trim might cause roll over from Quadrant I to IV or IV to I.
    // Fix that here. Calcs done in AS5600 units.
    // Recall angleValue will be processed into the PID Input
    angleValue = encoderValue + angleDifference; // in AS5600 units
    if (centerAngle < 2047) {
      if ((4096 - angleDifference) < encoderValue && encoderValue < 4097) {
        angleValue -= 4097;
      }
    } else if (centerAngle > 2047) {
      if (0 <= encoderValue && encoderValue < abs(angleDifference)) {
        angleValue += 4097;
      }
    }



    // PID Input range of motion determined by a pot. 
    // In this mode the pot is read for every call to loop().
    // One pot, so range set up to be symmetrical.
    // Symmetrical about the trimmed output shaft home  or symmetrical
    // about the AS5600 home (motor frame home). The choice is
    // application specific.
    // Note: not fiddling the PID Setpoint but rather the PID Input.
    
    // At max pot setting of 1023
    // the range of motion is limit to +- 1023 AS5600 units. A little
    // more than +- 90 degrees.
    // Note: The leftLimit angle is measured from (1,0) to approximately (0,1) on the unit circle.
    // The rightLimit angle is measured from (1,0) to approximately (0,-1) on the unit circle.
    rangeAdjustment = analogRead(POT_RANGE_ADJ_PIN); // Pot units 0 to 1024.
    leftLimit = 0 + 30 + rangeAdjustment;     // ccw limit from (1,0). AS5600 units
    rightLimit = 4097 - 30 - rangeAdjustment; // cw limit from (1,0). AS5600 units

    // Establish PID Input value for PID.
    // Want motor motion constrained from leftLimit -> 2048 -> rightLimit in AS5600 units
    // or constrained <0 to ~90 degrees> to 180 to <360 to ~270 degrees)

    Input = map(angleValue, leftLimit, rightLimit, 0, 360); // degrees

  }  // End of if (digitalRead(MODE_SELECTION_SW_PIN) == 1)



  // Adjusting sensitivity; real time user adjustment.
  // Every run of loop() looks at this pot.
  Kp = analogRead(POT_KP_PIN) * 0.002;  // pot 0-1023 so Kp 0-2.047
  myPID.SetTunings(Kp, Ki, Kd);

  // Run PID process to get Output value
  myPID.Compute();

  // Recall: SetOutputLimits -255 to 255
  pwmValue = abs(Output);

  if (5 < pwmValue && pwmValue < 30) {
    pwmValue = pwmValue + 30;
  }
  if (pwmValue <= 5) {
    pwmValue = 0;
  }

  // Move right
  if (0 <= Output) {

    digitalWrite(MOTOR_IN1_PIN, LOW);
    analogWrite(MOTOR_IN2_PIN, pwmValue);
  }
  // Move left
  else {  // if (Output < 0) {
    analogWrite(MOTOR_IN1_PIN, pwmValue);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }

  //Serial.print(Setpoint);
  //Serial.print("\t");
  //Serial.println(totalAngle);
}

//------------------------------------------
