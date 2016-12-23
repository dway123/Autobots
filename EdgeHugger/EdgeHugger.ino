/*
 * NOTE: Libraries can be found at:
 * Adafruit_GFX.h       https://github.com/adafruit/Adafruit-GFX-Library
 * Adafruit_SSD1306.h   https://github.com/adafruit/Adafruit_SSD1306
 * Ultrasonic.h         https://www.itead.cc/wiki/Ultrasonic_Ranging_Module_HC-SR04        
*/

#include "Ultrasonic.h"

//libraries for OLED display
#include <SPI.h>
#include <Wire.h>
#include <limits.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

//initializations for OLED display
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

//launcher pin
#define LAUNCHERPIN                   39  // pin with launcher
#define IRUPPIN1                      7
#define IRUPPIN2                      8

//initializations for ultrasonics
//TODO: MAKE SURE THESE PINS ARE CORRECT
Ultrasonic ultrasonic4(26,27);  // right front
Ultrasonic ultrasonic3(28,29);  // front left
Ultrasonic ultrasonic2(30,31);  // front mid
Ultrasonic ultrasonic1(32,33);  // front right
Ultrasonic ultrasonic5(34,35);  // right back (was left)

//direction indicators
#define MOTOR_FORWARD                 0
#define MOTOR_TURN_RIGHT              1
#define MOTOR_TURN_LEFT               2
#define MOTOR_BRAKE                   3

// USER INPUT

//Thresholds to tweak
#define ULTRASONIC_RIGHT_LOW_THRESH   9
#define ULTRASONIC_RIGHT_HIGH_THRESH  11
#define ULTRASONIC_FRONT_LOW_THRESH   13

#define IR_LOW_THRESHOLD              38
#define IR_LOW_DELTA_THRESHOLD_UP     20

#define MOTOR_LEFT_ADJUST_TIME      200  
#define MOTOR_RIGHT_ADJUST_TIME     100 
#define MOTOR_LEFT_90_TIME          500  // What delay time will generate a ~90 degree turn?

#define LAUNCHERSPINTIME            1500   // experimentally determined to provide ~45 degrees spin

//Thresholds added after flowchart was created  
#define MOTOR_INCH_FWD_TIME         50
#define MOTOR_TRACK_TURN_TIME       200

//Global Variables
int currDir = 0;      // 0 indicates forward, 1 indicates turning right, 2 indicates turning left, 3 indicates stopped (ex. launching ball)
int IRPins[7];        // 7 IRpins, leftmost is 0, rightmost is 6

//startup values
bool hasBall = true;  // true indicates going away from home, false indicates returning home
int init_up_1_val;
int init_up_2_val;

void setup() {
  Serial.begin(19200);          // Arduino Mega valid baud rates: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
  setup_display();
  pinMode(LAUNCHERPIN, OUTPUT);
  // 5 - 13 are motor pins
  for(int i = 5; i < 13; i++) {
    pinMode(i, OUTPUT);
  }
  init_up_IR();  
}

// ACTIVE CODE

void loop() {

  if(hasBall){
    if(foundIRTarget()){
      trackIRAndShoot();
    }
    
    if(foundUpIRTarget()){
      brake();
      shoot();
    }
  }
  
  //check Ultrasonics
  //check Right
  
  if(rightTooClose()){  //too close 
    smallTurnLeft();
//    moveBackwards();
//    delay(200);
  }
  else if(rightTooFar()){ //too far
    smallTurnRight();

  }
  //check Front
  if(frontTooClose()){  //too close
    turn90Left();
    inchForward();
  }
  else{ //ok
    inchForward();
  }
  refresh_display();
}

/**********
ULTRASONIC SENSORS
**********/

bool USBetween(Ultrasonic us, int lowThresh, int highThresh){
//  refresh_display();
  int read = us.Ranging(CM);
  if(read < highThresh && read > lowThresh){
    return true;
  }
  return false;
}

bool frontTooClose(){
  int lowThresh = INT_MIN;
  int highThresh = ULTRASONIC_FRONT_LOW_THRESH;
  return USBetween(ultrasonic1, lowThresh, highThresh) || USBetween(ultrasonic2, lowThresh, highThresh); // TODO: fix ultrasonic 3 and put the check for it back
}

bool rightTooFar(){
  int lowThresh = ULTRASONIC_RIGHT_HIGH_THRESH;
  int highThresh = INT_MAX;
  return USBetween(ultrasonic3, lowThresh, highThresh) || USBetween(ultrasonic4, lowThresh, highThresh)|| USBetween(ultrasonic5, lowThresh, highThresh);
}

bool rightTooClose(){
  int lowThresh = INT_MIN;
  int highThresh = ULTRASONIC_RIGHT_LOW_THRESH;
  return USBetween(ultrasonic3, lowThresh, highThresh) || USBetween(ultrasonic4, lowThresh, highThresh);
}

/**********
IR SENSORS & LAUNCHER
**********/

// function to run through analog pins; returns IR with max value if past threshold
int getMaxIR(){
//  refresh_display();
  int maxPin = 0;
  int maxIndex = 0;
  for (int j = 0; j < 7; j++) {
    int dummy = 0;
    for (int i = 0; i < 3; i++) {
      dummy = dummy + analogRead(j);
    }
    dummy = dummy / 3;
    IRPins[j] = dummy;
    if(IRPins[j] > maxPin) {
      maxPin = IRPins[j];
      maxIndex = j;
    }
  }
  if(maxPin > IR_LOW_THRESHOLD) {
    return maxIndex;
  }
  return -1;
}

int init_up_IR(){
  int r1 = 0;
  int r2 = 0;
  int trials = 10;
  for(int i = 0; i < trials; i++){
      r1 += analogRead(IRUPPIN1);
      r2 += analogRead(IRUPPIN2);
      delay(10);
  }
  init_up_1_val = r1/trials;
  init_up_2_val = r2/trials;
}

bool foundUpIRTarget(){
  int read1 = analogRead(IRUPPIN1);
  int read2 = analogRead(IRUPPIN2);
  return ((abs(read1 - init_up_1_val) > IR_LOW_DELTA_THRESHOLD_UP) || (abs(read2 - init_up_2_val) > IR_LOW_DELTA_THRESHOLD_UP));
}

//whether the target has been seen
bool foundIRTarget(){
  if(getMaxIR() != -1){
    return 1;
  }
  return 0;
}

// function tracks IR signal; stops when it is centered
int trackIRAndShoot(){
  brake();
  int stime, etime, toRight, maxIR, timeMovingRight; //to undo time
  while(1){
//    brake();
//    delay(100);
//    refresh_display();
    maxIR = getMaxIR();
    if(maxIR < 0){    //fuck it
      brake();
      break;
    }
    else if(maxIR == 4) {//brake and shoot
      brake();
      shoot();
      break;
    }
    stime = millis();
    if(maxIR < 4){
      toRight = 1;
      turnRight();
      delay(MOTOR_TRACK_TURN_TIME);
    }
    else if(maxIR > 4) {
      toRight = -1;
      turnLeft();
      delay(MOTOR_TRACK_TURN_TIME);
    }
    brake();
    etime = millis();
    timeMovingRight += toRight * (etime - stime);
  }
  brake();
//  undoTrackingRotate(timeMovingRight);
  return 0;
}

//resets angle of attack to that ideally before trackIRAndShoot.
//error acceptable due to movement strategy tolerating it.
void undoTrackingRotate(int milliseconds){
  if(milliseconds > 0){
    turnLeft();
    delay(milliseconds);
  }
  else{   
    turnRight();
    delay(milliseconds);
  }
 brake();
}

// after IR is detected, shoot a gun, turn, and go home
void shoot() {
  refresh_display();
  digitalWrite(LAUNCHERPIN, HIGH);
  delay(LAUNCHERSPINTIME);
  digitalWrite(LAUNCHERPIN, LOW);
  hasBall = false;
}

/**********
MOTORS
**********/

void brake()  {
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
//  currDir = MOTOR_BRAKE;
//  refresh_display();
}

void moveForward() {
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  currDir = MOTOR_FORWARD;
//  refresh_display();
}

//moveBackwards is not used, so this can safely be deleted
//however, this function is kept for bookkeeping purposes.
//in case of using moveBackwards, please add a new #define MOTOR_BACKWARD
void moveBackwards() {
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  //currDir = MOTOR_BACKWARD;
//  refresh_display();
}

void turnRight(){
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  currDir = MOTOR_TURN_RIGHT;
//  refresh_display();
}

void turnRight2(int x){
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  delayMicroseconds(x);
//  currDir = MOTOR_TURN_RIGHT;
//  refresh_display();
}

void turnLeft(){
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);
  currDir = MOTOR_TURN_LEFT;
//  refresh_display();
}

/**********
MOVEMENT (using motors)
**********/

void smallTurnLeft(){
  turnLeft();
  delay(MOTOR_LEFT_ADJUST_TIME);
}

void smallTurnRight(){
  turnRight();
  delay(MOTOR_RIGHT_ADJUST_TIME);
}

void turn90Left(){
  turnLeft();
  delay(MOTOR_LEFT_90_TIME);
}

void inchForward(){
  moveForward();
  delay(MOTOR_INCH_FWD_TIME);
}
  
/**********
OLED DISPLAY
**********/

// setup for the display
void setup_display(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize oled with the I2C addr 0x3D (for the 128x64)
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(250);
  // Clear the buffer.
  display.clearDisplay();
}

// refreshes the display
void refresh_display(){
  //text setup
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  
  //text
  displayAllRawUltrasonicValues();
//  displayAllHIGHRawIRvalues();
  displayAllRawIRvalues();
  
  display.setTextSize(2);
  display.println("BUMBLEBEE");
  display.print("Dir:  ");
  
  if(currDir == MOTOR_FORWARD){display.println("FWD");}
  else if(currDir == MOTOR_TURN_RIGHT){display.println("RGT");}
  else if(currDir == MOTOR_TURN_LEFT){display.println("LFT");}
  else if(currDir == MOTOR_BRAKE){display.println("STP");}
  else{display.println("ERR");}
  
  display.print("Mode: ");
  if(hasBall){
    display.println("ATK");
  }
  else{
    display.println("DEF");
  }
  display.display();
}

void displayAllRawUltrasonicValues(){
  display.print("US:");
  display.print(int_format(ultrasonic1.Ranging(CM)));
  display.print("|");
  display.print(int_format(ultrasonic2.Ranging(CM)));
  display.print("|");
  display.print(int_format(ultrasonic3.Ranging(CM)));
  display.print("|");
  display.print(int_format(ultrasonic4.Ranging(CM)));
  display.print("|");
  display.print(int_format(ultrasonic5.Ranging(CM)));
  //display.print("|IR:");  //NOTE: IR printed with Ultrasonic values due to lack of display space
  display.print("\n");
}

void displayAllRawIRvalues(){
//  display.print("IR:"); //NOTE: IR printed with Ultrasonic values due to lack of display space
  for (int j = 0; j < 7; j++) {
    IRPins[j] = analogRead(j);
    display.print(IRPins[j]);
    if(j != 7){
      display.print("|");
    }
  }
  display.println();
}

void displayAllHIGHRawIRvalues(){
  display.print("IR:");
  display.print(analogRead(7));
  display.print("|");
  display.print(analogRead(8));
  display.print("|");
  display.print(init_up_1_val);
  display.print("|");
  display.print(init_up_2_val);
  display.println();
}


String int_format(int i){
  if(i > 99){
    return "99";
  }
  else if(i < 10){
    return "0" + String(i);
  }
  else{
    return String(i);
  }
}
