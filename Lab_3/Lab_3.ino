#include <Arduino.h>
#line 1 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
#line 1 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
/*RobotPDControl.ino
  Author: Carlotta. A. Berry
  Date: December 31, 2016

  This program provides the functionality of allowing a robot to perform basic wall folowing.  
  The functions that this robot can perform are:

     Follow Right wall
     Follow Left Wall
     Follow the Center of two walls
     Navigate an inner corner
     Navigate an outer corner

  This functionality is primarily present in wallBang()

  Wall Bang - This function contains the logic for making the robot follow a wall with PD control.  The state
               variable is used to track the current state of the robot.  If the robot only detects a wall on one
               side, then it will follow that wall.  If a wall is detected on both sides of the robot, then the 
               follow center code takes over.  If no walls are detected, then the robot will enter random wander.

  Additionally, to update sensor values and determine state there are the following functions

    updateIR() - This function updates the current IR values of all four IR sensors
    updateSonar2() - This function updates the current IR values of all two Sonar sensors
    updateError() - This function updates the errors of each for every wall (right error, left error, distance from center)
    updateState() - This function updates the state of the robot
 
  Hardware Connections:
  Stepper Enable          Pin 48
  Right Stepper Step      Pin 52
  Right Stepper Direction Pin 53
  Left Stepper Step       Pin 50
  Left Stepper Direction  Pin 51

  Front IR    A8
  Back IR     A15
  Right IR    A11
  Left IR     A9
  Left Sonar  A12
  Right Sonar A13
  Button      A15
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>//include timer interrupt library

//define stepper motor pin numbers
#define stepperEnable 48//stepper enable pin on stepStick
#define rtStepPin 52    //right stepper motor step pin
#define rtDirPin 53     // right stepper motor direction pin
#define ltStepPin 50    //left stepper motor step pin
#define ltDirPin 51     //left stepper motor direction pin

//define sensor pin numbers
#define irFront   A8    //front IR analog pin
#define irRear    A15    //back IR analog pin
#define irRight   A11  //right IR analog pin
#define irLeft    A9   //left IR analog pin
#define snrLeft   A12   //front left sonar 
#define snrRight  A13   //front right sonar 
#define button    A15   //pushbutton 

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin); //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);  //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                //create instance to control multiple steppers at the same time
NewPing sonarLt(snrLeft, snrLeft);    //create an instance of the left sonar
NewPing sonarRt(snrRight, snrRight);  //create an instance of the right sonar

//define stepper motor constants
#define stepperEnTrue false     //variable for enabling stepper motor
#define stepperEnFalse true     //variable for disabling stepper motor
#define test_led 13             //test led to test interrupt heartbeat
#define enableLED 13            //stepper enabled LED
#define robot_spd 500           //set robot speed
#define max_accel 10000         //maximum robot acceleration
#define max_spd 1000            //maximum robot speed
#define quarter_rotation 200    //stepper quarter rotation
#define half_rotation 400       //stepper half rotation
#define one_rotation  800       //stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  1600      //stepper motor 2 rotations
#define three_rotation 2400     //stepper rotation 3 rotations
#define four_rotation 3200      //stepper rotation 4 rotations
#define five_rotation 4000      //stepper rotation 5 rotations

//define sensor constants and variables
#define irMin    6               // IR minimum threshold for wall (use a deadband of 4 to 6 inches)
#define irMax    4               // IR maximum threshold for wall (use a deadband of 4 to 6 inches)
#define snrMin   4               // sonar minimum threshold for wall (use a deadband of 4 to 6 inches)
#define snrMax   6               // sonar maximum threshold for wall (use a deadband of 4 to 6 inches)

int irFrontArray[5] = {0, 0, 0, 0, 0};//array to hold 5 front IR readings
int irRearArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 back IR readings
int irRightArray[5] = {0, 0, 0, 0, 0};//array to hold 5 right IR readings
int irLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left IR readings
int irFrontAvg;                       //variable to hold average of current front IR reading
int irLeftAvg;                        //variable to hold average of current left IR reading
int irRearAvg;                        //variable to hold average of current rear IR reading
int irRightAvg;                       //variable to hold average of current right IR reading
int irIdx = 0;                        //index for 5 IR readings to take the average
int srLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left sonar readings
int srRightArray[5] = {0, 0, 0, 0, 0};//array to hold 5 right sonar readings
int srIdx = 0;                        //index for 5 sonar readings to take the average
int srLeft;                           //variable to hold average of left sonar current reading
int srRight;                          //variable to hold average or right sonar current reading
int srLeftAvg;                        //variable to holde left sonar data
int srRightAvg;                       //variable to hold right sonar data

//STATE MACHINE TIMER INTERRUPT VARIABLES
volatile boolean test_state;          //variable to hold test led state for timer interrupt
#define timer_int 500000              //1/2 second (500000 us) period for timer interrupt

//bit definitions for sensor data flag byte [rt_snr left_snr left_ir right_ir rear_ir front_ir]
volatile byte flag = 0;
#define obFront   0   // Front IR trip [used to detect front wall for corner]
#define obRear    1   // Rear IR trip
#define obRight   2   // Right IR trip
#define obLeft    3   // Left IR trip
#define obFLeft   4   // Left Sonar trip
#define obFRight  5   // Right Sonar trip

//bit definitions for robot motion and state byte [follow_hallway follow_right follow_left wander avoid]
volatile byte state = 0;
#define avoid     0   //avoid behavior
#define wander    1   //wander behavior
#define fleft     2   //follow left wall behavior
#define fright    3   //follow right wall behavior
#define center    4   //follow hallway behavior
#define movingL   6   //robot left wheel moving
#define movingR   7   //robot right wheel moving

//define layers of subsumption architecture that are active [hallway Wall Wander Avoid]
byte layers = 4;
#define aLayer 0      //avoid obstacle layer
#define wLayer 1      //wander layer
#define fwLayer 2     //follow wall layer
#define fhLayer 3     //follow hallway layer

#define green 9       //The pin number of the green led
#define red 8         //The pin number of the red led

//define PD control global variables, curr_error = current reading - setpoint, prev_error = curr_error on previous iteration
//store previous error to calculate, derror = curr_error-prev_error, side_derror = side front sensor - side back sensor
//store derror = difference between left and right error (used for hall follow to center the robot)

int ls_curr;    //left sonar current reading
int li_curr;    //left ir current reading
int rs_curr;    //right sonar current reading
int ri_curr;    //right ir current reading

int ls_cerror;    //left sonar current error
int li_cerror;    //left ir current error
int rs_cerror;    //right sonar current error
int ri_cerror;    //right ir current error

int ls_perror;    //left sonar previous error
int li_perror;    //left ir previous error
int rs_perror;    //right sonar previous error
int ri_perror;    //right ir previous error

int ls_derror;  //left sonar delta error
int li_derror;  //left ir delta error
int rs_derror;  //right sonar delta error
int ri_derror;  //right ir current error
int left_derror;   //difference between left front and back sensor, this may be useful for adjusting the turn angle
int right_derror;  //difference between right front and back sensor, this may be useful for adjusting the turn angle

int derror;       //difference between left and right error to center robot in the hallway

int lastState = 0;  //Tracks the lats state that the robot was in.  This is used for outside corners.

#define baud_rate 9600  //set serial communication baud rate

#line 162 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void setup();
#line 189 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void loop();
#line 208 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void wallBang();
#line 310 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void updateSensors();
#line 339 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
int getFrontDistance(long value);
#line 370 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
int getRightDistance(long value);
#line 397 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
int getRearDistance(long value);
#line 427 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
int getLeftDistance(long value);
#line 441 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void updateIR();
#line 547 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void updateSonar();
#line 607 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void updateSonar2();
#line 658 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void updateError();
#line 671 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void updateState();
#line 718 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void forward(int rot);
#line 733 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void reverse(int rot);
#line 748 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void pivot(int rot, int dir);
#line 769 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void spin(int rot, int dir);
#line 790 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void stop();
#line 798 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void runToStop( void );
#line 162 "C:\\Users\\moormaet\\AppData\\Local\\Temp\\Rar$DIa0.576\\lab_3\\lab_3.ino"
void setup()
{
  pinMode(rtStepPin, OUTPUT);                 //sets pin as output
  pinMode(rtDirPin, OUTPUT);                  //sets pin as output
  pinMode(ltStepPin, OUTPUT);                 //sets pin as output
  pinMode(ltDirPin, OUTPUT);                  //sets pin as output
  pinMode(stepperEnable, OUTPUT);             //sets pin as output
  pinMode(green, OUTPUT);                     //sets green pin as output
  pinMode(red, OUTPUT);                       //sets red pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  stepperRight.setMaxSpeed(max_spd);          //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);    //set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);           //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);     //set desired acceleration in steps/s^2
  stepperRight.setSpeed(robot_spd);           //set right motor speed
  stepperLeft.setSpeed(robot_spd);            //set left motor speed
  steppers.addStepper(stepperRight);          //add right motor to MultiStepper
  steppers.addStepper(stepperLeft);           //add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue); //turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);              //turn on enable LED
  Serial.begin(baud_rate);                    //start serial communication in order to debug the software while coding
  delay(1500);                                //wait 3 seconds before robot moves
}

void loop()
{
  updateSensors();      //Updates all of the sensor values and decides the robot state
  wallBang();           //wall following PD control
}

/*

  Description: This function contains the logic for making the robot follow a wall with PD control.  The state
               variable is used to track the current state of the robot.  If the robot only detects a wall on one
               side, then it will follow that wall.  If a wall is detected on both sides of the robot, then the 
               follow center code takes over.  If no walls are detected, then the robot will enter random wander.

  Inputs:

           VOID

  Outputs:

           VOID

*/
void wallBang() {
  digitalWrite(green, LOW);
  digitalWrite(red, LOW);
  Serial.print("\nWallBang: li_cerror ri_cerror\t");
  Serial.print(li_cerror); Serial.print("\t");
  Serial.println(ri_cerror);

  if (bitRead(state, fright)) {
    lastState = -1;  // Record the last state as right (-1)
    double rightKp = 5;  // The proportional constant for the right wall
    double rightKd = 2;  // The derivative constant for the right wall
    Serial.println("right wall found");
    if (bitRead(flag, obFront)) { //check for a front wall before moving
      Serial.print("right wall: front corner ");
      //make left turn if wall found (inner corner)
      spin(quarter_rotation, 0);              //turn left
      return;
    }
    if (ri_cerror == 0) {                 //no error, robot in deadband (4-6 inches)
      Serial.println("right wall detected, drive forward");
      forward(quarter_rotation);            //move robot forward
    }
    else {
      if (ri_cerror < 0) {          //negative error means too close
        digitalWrite(green, HIGH);  //Turn on the green light

        Serial.println("\trt wall: too close turn left");
        pivot(quarter_rotation + (abs(rightKp * ri_cerror)) - rightKd * ri_derror, 0);  //Make the larger (left) pivot
        pivot(quarter_rotation, 1);   //pivot right to straighten up
      }
      else if (ri_cerror > 0) {     //positive error means too far
        digitalWrite(red, HIGH);  // turn on the red light
        Serial.println("\trt wall: too far turn right");

        pivot(quarter_rotation + ((abs(rightKp * ri_cerror))) - rightKd * ri_derror, 1);      //Make te larger (right) pivot
        pivot(quarter_rotation, 0);  //pivot left to straighten up
      }
    }
  }

  else if (bitRead(state, fleft)  ) {
    lastState = 1;  // set the last state to be left (1)
    int leftKp = 5;  //The proportional constant for the left wall
    int leftKd = 2;  //Yhe derivative constant for the right wwall
    if (bitRead(flag, obFront)) { //check for a front wall before moving forward

      Serial.print("left wall: front corner ");
      //make right turn if wall found

      spin(quarter_rotation, 1);              //turn right
      return;
    }
    if (li_cerror == 0) {           //no error robot in dead band drives forward
      //Serial.println("lt wall detected, drive forward");
      forward(quarter_rotation);      //move robot forward
    }
    else {
      if (li_cerror < 0) { //negative error means too close

        pivot(quarter_rotation + abs(leftKp * li_cerror) - leftKd * li_derror, 1);      //make larger (right) pivot
        pivot(quarter_rotation, 0);   //pivot left to straighten
      }
      else if (li_cerror > 0)  { //positive error means too far
 
        pivot(quarter_rotation + abs(leftKp * li_cerror) - leftKd * li_derror, 0);      //make larger (left) pivot
        pivot(quarter_rotation, 1);   //pivot right to straighten
      }
    }
  }
  else if (bitRead(state, center) ) {//follow hallway
      double rightKp = 5;  //The right proportional constant
    double rightKd = 2;  //The right derivative constant
    int leftKp = 5;  //The left proportional constant
    int leftKd = 2;  //The left derivative constant
    if (((ri_cerror == 0) && (li_cerror == 0)) || (derror == 0)) {
      forward(quarter_rotation);          //drive robot forward
    }
    else {

      if (derror > 0) {
        pivot(quarter_rotation + abs(leftKp * derror) - leftKd * derror, 0);      //make larger (left) pivot
        pivot(quarter_rotation, 1);   //pivot right
      }
      else
      {
        pivot(quarter_rotation + ((abs(rightKp * derror))) - rightKd * derror, 1);      //maker larger (right) pivot
        pivot(quarter_rotation, 0);  //pivot left to straighten up
      }
    }
  }
  else  if (bitRead(state, wander)) {
    
    Serial.println("nothing to see here, I need to look for a wall");
    if (lastState == 0) {  // If you have tried to corner but couldn't find the wall again
      stop();
      delay(500);
      //reverse(half_rotation);
      spin(half_rotation, 0);
      forward(one_rotation);
      pivot(quarter_rotation, 1);
      if (bitRead(flag, obFront)){
        spin(half_rotation, 0);
      }
    }else if (lastState < 0) { // If the last state was right wall following
      forward(quarter_rotation);  //Move foward to clear the turn
      spin(half_rotation, 1);  //Turn right 90 degrees
      forward(two_rotation);  // Move forward 2 rotations
      lastState++;  //Reset to random wander
    }else {  // If the last state was left wall following
      forward(quarter_rotation); //Move forward to clear the turn
      spin(half_rotation, 0);  // turn left 90 degrees
      forward(two_rotation); // Mover forward 2 rotations
      lastState--; //Reset to random wander
    }
  }
}


/*

  Description: This function updates sensor values and determines what the new state should be
  Inputs:

           VOID

  Outputs:

           VOID

*/
void updateSensors() {
  test_state = !test_state;             //LED to test the heartbeat of the timer interrupt routine
  digitalWrite(test_led, test_state);   //flash the timer interrupt LED
  flag = 0;                             //clear all sensor flags
  state = 0;                            //clear all state flags
  updateIR();                           //update IR readings and update flag variable and state machine
  updateSonar2();                     //there are 2 ways to read sonar data, this is the 2nd option, use whichever one works best for your hardware
  updateError();                        //update sensor current, previous, change in error
  updateState();                        //update State Machine based upon sensor readings
}

/*

  Description: This function takes in the current value of the Front IR Sensor and computes the actual distance
               to the obstacle.

  Inputs:

           Value: the current value of the Front IR Sensor



  Outputs:

           The actual distance to the obstacle in inches

*/
int getFrontDistance(long value) {

  double value2 = 10 * value + 9847;

  value2 = value2 / (153 + 10 * value);

  if (value2 >= 20 || value2 < 0) { //Get rid of junk or invalid signals
    return 0;
  }

  return value2;

}


/*

  Description: This function takes in the current value of the Right IR Sensor and computes the actual distance
               to the obstacle.

  Inputs:

           Value: the current value of the Right IR Sensor



  Outputs:

           The actual distance to the obstacle in inches

*/
int getRightDistance(long value) {

  double value2 = -46 * value + 48884.5;

  value2 = value2 / (485 + 20 * value);

  return value2;

}


/*

  Description: This function takes in the current value of the Rear IR Sensor and computes the actual distance
               to the obstacle.

  Inputs:

           Value: the current value of the Rear IR Sensor



  Outputs:

           The actual distance to the obstacle in inches

*/
int getRearDistance(long value) {

  double value2 = -5 * value + 5249;

  value2 = value2 / (-249 + 5 * value);

  if (value2 > 20 || value2 < 0) { //Get rid of junk or invalid signals
    return 0;
  }

  return value2;
}


/*

  Description: This function takes in the current value of the Left IR Sensor and computes the actual distance
               to the obstacle.

  Inputs:

           Value: the current value of the Left IR Sensor



  Outputs:

           The actual distance to the obstacle in inches

*/
int getLeftDistance(long value) {

  double value2 = -10 * value + 9814;

  value2 = value2 / (93 + 5 * value);

  return value2;

}

/*

  Description: This function updates the current IR values of all four IR sensors

  Inputs:

           VOID



  Outputs:

           VOID

*/
void updateIR() {
  int front, back, left, right;         //declare IR variables
  front = getFrontDistance(analogRead(irFront));          //read front IR sensor
  back = getRearDistance(analogRead(irRear));            //read back IR sensor
  li_curr = getLeftDistance(analogRead(irLeft));            //read left IR sensor
  ri_curr = getRightDistance(analogRead(irRight));          //read right IR sensor
  Serial.print("ri_curr = ");
  Serial.println(ri_curr);

  if (ri_curr < 12 && ri_curr > 0) {
    bitSet(flag, obRight);            //set the right obstacle
  } else {
   
    bitClear(flag, obRight);          //clear the right obstacle
  }

  if (li_curr < 12 && li_curr > 0) {
    bitSet(flag, obLeft);
  } else {
    bitClear(flag, obLeft);           //clear the left obstacle
  }

  li_curr += 2;  //Adjust for left sensor error
  if (front < 12 && front > 0) {
    bitSet(flag, obFront);            //set the front obstacle
  } else
    bitClear(flag, obFront);          //clear the front obstacle

  

  ///////////////////////update variables

  if ((ri_curr < irMax)) {
    ri_cerror = ri_curr - irMax;//calculate current error (too far positive, too close negative)
  } else if ((ri_curr > irMin)) {
    ri_cerror = ri_curr - irMin; //calculate current error (too far positive, too close negative)
  } else
    ri_cerror = 0;                  //set error to zero if robot is in dead band
  ri_derror = ri_cerror - ri_perror; //calculate change in error
  ri_perror = ri_cerror;            //log current error as previous error [left sonar]

  if (li_curr < irMax) {
    li_cerror = li_curr - irMax;   //calculate current error (too far positive, too close negative)
  } else if (li_curr > irMin) {
    li_cerror = li_curr - irMin;   //calculate current error (too far positive, too close negative)
  } else
    li_cerror = 0;                  //error is zero if in deadband
  li_derror = li_cerror - li_perror; //calculate change in error
  li_perror = li_cerror;                //log reading as previous error
}

/*

  Description: This function updates the current IR values of all two Sonar sensors

  Inputs:

           VOID

  Outputs:

           VOID

*/
void updateSonar2() {
  srRightAvg =  sonarRt.ping_in();//read right sonar in inches
  delay(50);                      //delay 50 ms
  srLeftAvg = sonarLt.ping_in();  //reaqd left sonar in inches

  ///////////////////////update variables
  rs_curr = srRightAvg;             //log current sensor reading [right sonar]
  if ((rs_curr > snrMax) || (rs_curr < snrMin))
    rs_cerror = rs_curr - snrMax;    //calculate current error (too far positive, too close negative)
  else
    rs_cerror = 0;                  //set error to zero if robot is in dead band
  rs_derror = rs_cerror - rs_perror; //calculate change in error
  rs_perror = rs_cerror;            //log current error as previous error [left sonar]

  ls_curr = srLeftAvg;                   //log current sensor reading [left sonar]
  if ((ls_curr > snrMax) || (ls_curr < snrMin))
    ls_cerror = ls_curr - snrMax;     //calculate current error
  else
    ls_cerror = 0;                  //error is zero if in deadband
  ls_derror = ls_cerror - ls_perror; //calculate change in error
  ls_perror = ls_cerror;                //log reading as previous error

}

/*

  Description: This function updates the errors of each for every wall (right error, left error, distance from center)

  Inputs:

           VOID

  Outputs:

           VOID

*/
void updateError() {
  left_derror = ls_cerror - li_cerror; //difference between left front and back sensor, use threshold for robot mostly parallel to wall
  right_derror = rs_cerror - ri_cerror; //difference between right front and back sensor, use threshold for robot mostly parallel to wall

  derror = li_cerror - ri_cerror; //use IR data for difference error

}

/*

  Description: This function updates the state of the robot

  Inputs:

           VOID



  Outputs:

           VOID

*/
void updateState() {
  if (!(flag)) { //no sensors triggered
    //set random wander bit
    Serial.println("\tset random wander state");
    bitSet(state, wander);//set the wander state
    //clear all other bits
    bitClear(state, fright);//clear follow wall state
    bitClear(state, fleft);//clear follow wall state
    bitClear(state, center);//clear follow wall state
  }
  else if (bitRead(flag, obRight) && !bitRead(flag, obLeft) ) {
    Serial.println("\tset follow right state");
    bitSet(state, fright);    //set RIGHT WALL state
    //clear all other bits
    bitClear(state, wander);  //clear wander state
    bitClear(state, fleft);   //clear follow wall state
    bitClear(state, center);  //clear follow wall state
  }
  else if (bitRead(flag, obLeft) && !bitRead(flag, obRight) ) {
    Serial.println("\tset follow left state");
    bitSet(state, fleft);     //set left wall state
    //clear all other bits
    bitClear(state, fright);  //clear follow wall state
    bitClear(state, wander);  //clear wander state
    bitClear(state, center);  //clear follow wall state
  }
    else if (bitRead(flag, obLeft) && bitRead(flag, obRight) ) {
      Serial.println("\tset follow hallway state");
      bitSet(state, center);      //set the hallway state
      //clear all other bits
      bitClear(state, fright);    //clear follow wall state
      bitClear(state, wander);    //clear wander state
      bitClear(state, fleft);     //clear follow wall state
    }
}


/*robot move forward function */
void forward(int rot) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  positions[0] = stepperRight.currentPosition() + rot;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot;   //left motor absolute position

  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*robot move reverse function */
void reverse(int rot) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  positions[0] = stepperRight.currentPosition() - rot;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() - rot;   //left motor absolute position

  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*robot pivot function */
void pivot(int rot, int dir) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  if (dir > 0) {//pivot right
    positions[0] = stepperRight.currentPosition();    //right motor absolute position
    positions[1] = stepperLeft.currentPosition() + rot ; //left motor absolute position
  }
  else//pivot left
  {
    positions[0] = stepperRight.currentPosition() + rot ; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() ;     //left motor absolute position
  }
  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*robot spin function */
void spin(int rot, int dir) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  if (dir > 0) {//spin right
    positions[0] = stepperRight.currentPosition() - rot; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() + rot; //left motor absolute position
  }
  else//spin left
  {
    positions[0] = stepperRight.currentPosition() + rot; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() - rot;  //left motor absolute position
  }
  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*robot stop function */
void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

/*This function, runToStop(), will run the robot until the target is achieved and
  then stop it
*/
void runToStop ( void ) {
  int runNow = 1;

  while (runNow) {
    if (!stepperRight.run()) {
      bitClear(state, movingR);  // clear bit for right motor moving
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      bitClear(state, movingL);   // clear bit for left motor moving
      stepperLeft.stop();//stop left motor
    }//
    if (!bitRead(state, movingR) & !bitRead(state, movingL))
      runNow = 0;
  }
}


