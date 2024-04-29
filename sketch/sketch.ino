/*

************************************************************************************
* MIT License
*
* Copyright (c) 2023 Crunchlabs LLC (IRTurret Control Code)
* Copyright (c) 2020-2022 Armin Joachimsmeyer (IRremote Library)

* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is furnished
* to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
* CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
* OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
************************************************************************************
*/

//////////////////////////////////////////////////
              //  LIBRARIES  //
//////////////////////////////////////////////////
#include <Arduino.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <IRremote.hpp>
#include "Definitions.h"

#define DECODE_NEC  //defines the type of IR transmission to decode based on the remote. See IRremote library for examples on how to decode other types of remote

/*
** if you want to add other remotes (as long as they're on the same protocol above):
** press the desired button and look for a hex code similar to those below (ex: 0x11)
** then add a new line to #define newCmdName 0x11,
** and add a case to the switch statement like case newCmdName: 
** this will let you add new functions to buttons on other remotes!
** the best remotes to try are cheap LED remotes, some TV remotes, and some garage door openers
*/

//defines the specific command code for each button on the remote
#define left 0x8
#define right 0x5A
#define up 0x52
#define down 0x18
#define ok 0x1C
#define cmd1 0x45
#define cmd2 0x46
#define cmd3 0x47
#define cmd4 0x44
#define cmd5 0x40
#define cmd6 0x43
#define cmd7 0x7
#define cmd8 0x15
#define cmd9 0x9
#define cmd0 0x19
#define star 0x16
#define hashtag 0xD

// Sensor pins
#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define BUZZER_PIN   6  // Arduino pin tied to the buzzer.

//////////////////////////////////////////////////
          //  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
//this is where we store global variables!
Servo yawServo; //names the servo responsible for YAW rotation, 360 spin around the base
Servo pitchServo; //names the servo responsible for PITCH rotation, up and down tilt
Servo rollServo; //names the servo responsible for ROLL rotation, spins the barrel to fire darts

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN); // Defines the ultrasonic sensor

int yawServoVal; //initialize variables to store the current value of each servo
int pitchServoVal = 100;
int rollServoVal;

int pitchMoveSpeed = 8; //this variable is the angle added to the pitch servo to control how quickly the PITCH servo moves - try values between 3 and 10
int yawMoveSpeed = 90; //this variable is the speed controller for the continuous movement of the YAW servo motor. It is added or subtracted from the yawStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Try values between 10 and 90;
int yawStopSpeed = 90; //value to stop the yaw motor - keep this at 90
int rollMoveSpeed = 90; //this variable is the speed controller for the continuous movement of the ROLL servo motor. It is added or subtracted from the rollStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Keep this at 90 for best performance / highest torque from the roll motor when firing.
int rollStopSpeed = 90; //value to stop the roll motor - keep this at 90

int yawPrecision = 150; // this variable represents the time in milliseconds that the YAW motor will remain at it's set movement speed. Try values between 50 and 500 to start (500 milliseconds = 1/2 second)
int rollPrecision = 158; // this variable represents the time in milliseconds that the ROLL motor with remain at it's set movement speed. If this ROLL motor is spinning more or less than 1/6th of a rotation when firing a single dart (one call of the fire(); command) you can try adjusting this value down or up slightly, but it should remain around the stock value (160ish) for best results.

int pitchMax = 175; // this sets the maximum angle of the pitch servo to prevent it from crashing, it should remain below 180, and be greater than the pitchMin
int pitchMin = 10; // this sets the minimum angle of the pitch servo to prevent it from crashing, it should remain above 0, and be less than the pitchMax

bool reverseMode = false; // this variable is used to reverse the direction of the yaw motor when the turret is asked to fire. 
int safeDistance = 30; // this variable is used to set the distance in centimeters that the ultrasonic sensor will check for objects before firing. If an object is detected within this distance, the turret will not fire.


//////////////////////////////////////////////////
              //  S E T U P  //
//////////////////////////////////////////////////
void setup() {
    Serial.begin(9600); // initializes the Serial communication between the computer and the microcontroller

    yawServo.attach(10); //attach YAW servo to pin 10
    pitchServo.attach(11); //attach PITCH servo to pin 11
    rollServo.attach(12); //attach ROLL servo to pin 12

    pinMode(BUZZER_PIN, OUTPUT); //set the buzzer pin as an output
    // Just to know which program is running on my microcontroller
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(9, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println(F("at pin " STR(9)));


    homeServos(); //set servo motors to home position
}

////////////////////////////////////////////////
              //  L O O P  //
////////////////////////////////////////////////

void loop() {

    /*
    * Check if received data is available and if yes, try to decode it.
    */
    if (IrReceiver.decode()) {

        /*
        * Print a short summary of received data
        */
        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.printIRSendUsage(&Serial);
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) { //command garbled or not recognized
            Serial.println(F("Received noise or an unknown (or not yet enabled) protocol - if you wish to add this command, define it at the top of the file with the hex code printed below (ex: 0x8)"));
            // We have an unknown protocol here, print more info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
        }
        Serial.println();

        /*
        * !!!Important!!! Enable receiving of the next value,
        * since receiving has stopped after the end of the current received data packet.
        */
        IrReceiver.resume(); // Enable receiving of the next value


        /*
        * Finally, check the received data and perform actions according to the received command
        */

        switch(IrReceiver.decodedIRData.command){ //this is where the commands are handled

            case up://pitch up
              upMove(1);
              break;
            
            case down://pitch down
              downMove(1);
              break;

            case left://fast counterclockwise rotation
              leftMove(1);
              break;
            
            case right://fast clockwise rotation
              rightMove(1);
              break;
            
            case ok: //firing routine 
              if (reverseMode) 
                  reverse();

              if (safeToFire())
                  fire();
              else {
                  shakeHeadNo();
              }
              break;
              
            case star:
               if (safeToFire())
                  fireAll();
              else {
                  shakeHeadNo();
              }
              delay(50);
              break;

            case cmd1:
              reverseMode = !reverseMode;
              if (reverseMode)
                shakeHeadYes();
              else
                shakeHeadNo();
              break;

            case cmd2:
              playMusic();
              break;

            case cmd8:
              figureEight();
              break;

        }
    }
    delay(5);
}


void playMusic(){
  Serial.println("PLAY MUSIC");
  //tone(BUZZER_PIN, 90);
  for (int i=0; i<=30;i++) {
      digitalWrite(BUZZER_PIN, LOW);
      delay(10);
      //noTone(BUZZER_PIN);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(10);
  }
}

bool safeToFire() {
    // Check if the ultrasonic sensor detects an object within 10 cm.

    // store ultrasonic read distance in variable
    int distance = ultrasonic.read();
    Serial.println("Distance: " + String(distance) + " cm");

    if (distance > safeDistance) {
        Serial.println("SAFE TO FIRE");
        return true;
    }
    else {
        Serial.println("NOT SAFE TO FIRE");
        return false;
    }

}



void reverse(){

    Serial.println("REVERSE");

    yawServo.write(180);
    delay(450); // Adjust delay for smoother motion
    yawServo.write(yawStopSpeed);
    delay(50);
}

void smoothPitchMove(int startAngle, int endAngle, int smoothDelay){

    if (endAngle >= startAngle) {
        for (int angle = startAngle; angle <= endAngle; angle++) {
            pitchServo.write(angle);
            delay(smoothDelay); // Adjust delay for smoother motion
        }
    }
    else {
        for (int angle = startAngle; angle >= endAngle; angle--) {
            pitchServo.write(angle);
            delay(smoothDelay); // Adjust delay for smoother motion
        }
    }
}

void figureEight(){
    Serial.println("FIGURE EIGHT");

    int moves = 3; // Number of moves
    int startAngle = pitchServoVal; // Current position of the pitch servo
    int lowAngle = startAngle - 30; // Angle for nodding motion
    int highAngle = startAngle + 30; // Angle for nodding motion
    int yawSpeed = 45; // Speed of yaw servo.
    int pitchDelay = 16; // Delay to smooth pitch movements.

    for (int i=0; i < moves; i++) {
        // Move to top left
        yawServo.write(yawStopSpeed + yawSpeed);
        smoothPitchMove(startAngle, highAngle, pitchDelay );
        yawServo.write(yawStopSpeed);
        delay(50);
        // Move to bottom left
        smoothPitchMove(highAngle, lowAngle, 5);
        delay(50);
        // Move to top right
        yawServo.write(yawStopSpeed - yawSpeed);
        smoothPitchMove(lowAngle, highAngle, pitchDelay);
        yawServo.write(yawStopSpeed);
        delay(50);
        // Move to bottom right
        smoothPitchMove(highAngle, lowAngle, 5);
        delay(50);
        // Move to middle
        yawServo.write(yawStopSpeed + yawSpeed);
        smoothPitchMove(lowAngle, startAngle, pitchDelay);
        yawServo.write(yawStopSpeed);

    }
    pitchServo.write(startAngle);
    delay(50);
}

void shakeHeadYes(int moves = 3) {
    Serial.println("YES");
    int startAngle = pitchServoVal; // Current position of the pitch servo
    int nodAngle = startAngle + 20; // Angle for nodding motion

    for (int i = 0; i < moves; i++) { // Repeat nodding motion three times
        // Nod up
        for (int angle = startAngle; angle <= nodAngle; angle++) {
            pitchServo.write(angle);
            delay(7); // Adjust delay for smoother motion
        }
        delay(50); // Pause at nodding position
        // Nod down
        for (int angle = nodAngle; angle >= startAngle; angle--) {
            pitchServo.write(angle);
            delay(7); // Adjust delay for smoother motion
        }
        delay(50); // Pause at starting position
    }
}

void shakeHeadNo(int moves = 3) {
    Serial.println("NO");

    for (int i = 0; i < moves; i++) { // Repeat nodding motion three times
        // rotate right, stop, then rotate left, stop
        yawServo.write(140);
        delay(190); // Adjust delay for smoother motion
        yawServo.write(yawStopSpeed);
        delay(50);
        yawServo.write(40);
        delay(190); // Adjust delay for smoother motion
        yawServo.write(yawStopSpeed);
        delay(50); // Pause at starting position
    }
}

void leftMove(int moves){
    for (int i = 0; i < moves; i++){
        yawServo.write(yawStopSpeed + yawMoveSpeed); // adding the servo speed = 180 (full counterclockwise rotation speed)
        delay(yawPrecision); // stay rotating for a certain number of milliseconds
        yawServo.write(yawStopSpeed); // stop rotating
        delay(5); //delay for smoothness
        Serial.println("LEFT");
  }

}

void rightMove(int moves){ // function to move right
  for (int i = 0; i < moves; i++){
      yawServo.write(yawStopSpeed - yawMoveSpeed); //subtracting the servo speed = 0 (full clockwise rotation speed)
      delay(yawPrecision);
      yawServo.write(yawStopSpeed);
      delay(5);
      Serial.println("RIGHT");
  }
}

void upMove(int moves){
  for (int i = 0; i < moves; i++){
      if(pitchServoVal > pitchMin){//make sure the servo is within rotation limits (greater than 10 degrees by default)
        pitchServoVal = pitchServoVal - pitchMoveSpeed; //decrement the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("UP");
      }
  }
}

void downMove (int moves){
  for (int i = 0; i < moves; i++){
        if(pitchServoVal < pitchMax){ //make sure the servo is within rotation limits (less than 175 degrees by default)
        pitchServoVal = pitchServoVal + pitchMoveSpeed;//increment the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("DOWN");
      }
  }
}

/**
 * fire does xyz
 */
void fire() { //function for firing a single dart
    rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
    delay(rollPrecision);//time for approximately 60 degrees of rotation
    rollServo.write(rollStopSpeed);//stop rotating the servo
    delay(5); //delay for smoothness
    Serial.println("FIRING");
}

void fireAll() { //function to fire all 6 darts at once
    rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
    delay(rollPrecision * 6); //time for 360 degrees of rotation
    rollServo.write(rollStopSpeed);//stop rotating the servo
    delay(5); // delay for smoothness
    Serial.println("FIRING ALL");
}

void homeServos(){
    yawServo.write(yawStopSpeed); //setup YAW servo to be STOPPED (90)
    delay(20);
    rollServo.write(rollStopSpeed); //setup ROLL servo to be STOPPED (90)
    delay(100);
    pitchServo.write(100); //set PITCH servo to 100 degree position
    delay(100);
    pitchServoVal = 100; // store the pitch servo value
    Serial.println("HOMING");
}
   
