/*****************************************************************************
 * Rob Sveinson
 * Comp 444 final project
 * Line following roomba
 * version 2: follow a line without any controls in place
 * 
 * Using a Sparkfun line foolower array and a sonar sensor get a roomba to follow
 * a dark line on a light bachground. This will be developed in 5 stages as follows;
 * 
 * Stage 1 Sonar: Get the roomba to move in a relatively straight line and stop 
 * when approaching an obstacle. This is necessary to protect the line follower
 * array that sticks out in front of the roomba.
 * 
 * Stage 2 No controller: Get the roomba to follow the line without applying
 * any control mechanisms. Simply apply additional power to the appropriate
 * wheel when the roombo starts to go off course.
 * 
 * Stage 3 Proportional control: Add proportional control so the power is applied
 * proportional to the error. This should eliminate most of the oscilation
 * form the no control version
 * 
 * Stage 4 Integral control: Add integral control so that the application of power
 * is affected by the total error. This will hasten the system's stabalization and account
 * for any steady state error.
 * 
 * Staghe 5 Derivative control: Add derivative control so that the application of power
 * is affected by the instantaneous slope of the line tangent to the error curve. This will 
 * allow the system to settle more quickly by eliminating, or reducing, overshoot, meaning the 
 * robot will start to correct before it passes the set point.
 * 
 * This is stage 2: follow aline without any control in place. Should produce large, consisten 
 * oscillations, oscillations will not diminish
 */

#include<SoftwareSerial.h>
#include"Wire.h"              // for I2C serial interface
#include "sensorbar.h"        // libraries for line follower array
#define SIG A0

// global scope

const int BASESPEED = 200;            // base speed of wheel motors
const byte LFAADDRESS = 0x3E;  // assign an address to the line follower array
const float PIE = 3.14159;        // value of pi used in calculating angles
const int SAFEDIST = 20;          // 20cm safety zone, used for sonar
const float CORRECTION = 1.25;     // increase wheel speed by this factor

// create a new sensor bar object
SensorBar lfa(LFAADDRESS);     // lfa stands for line follower array
//delay(20);                    // delay for a tic

// pin assignments for arduino to roomba 7 mini-din port

int rxPin = 9;        // from arduino(9) to roomba pin(4) white wire
int txPin = 11;       // to arduino(11) from roomba pin(3) red wire
int ddPin = 5;        // device detect used to wake up roomba
                      // from arduino(5) to roomba pin(5) yellow wire
int buttonPin = 12;   // hold operation until button is pressed

// sonar variables
unsigned long rxTime;   // return time of sonar pulse
float distance;         // distance calculated form return time

SoftwareSerial mySerial(rxPin, txPin);  // new port with 9rx and 11tx
                                  // this means pin 4 on roomba connects to pin 9 on arduino
                                  // pin 3 on roomba connects to pin 11 on arduino


/*****************************************************************
 * start the serial port to print to serial monitor
 * print opening message to serial monitor
 * 
 * start mySerial at 19200 baud
 * 
 * put roomba into passive mode opcode 128
 * put roomba into safe mode opcode 130 or 131, they seem to do the same thing
 ******************************************************************************/
void setup(){
  Serial.begin(9600);       // start serial port for monitor
  //Serial.println("hello dave");

  mySerial.begin(19200);       // start software serial for communicating with roomba
                                // use software serial because roomba's tx doesn't always
                                // carry enough voltage to meet arduino's requirements
  delay(10);              // set a short delay to allow the command to propegate

/******************************************************************
 * set up the line follower array
 * clearBarStrobe();
 * set indicator lights to be on while sensor is detecting a dark line
 * alternative is setBarStrobe(); sets indicators to blink each time it reads
 * so you get blinking light when a sensor is detecting a line
 * 
 * clearInvertBits();
 * sets array to detect dark line on light background
 * alternative is setInvertBits(); sets array to detect light line on dark background
 *****************************************************************************/

  lfa.clearBarStrobe();
  lfa.clearInvertBits();
  delay(20);              // delay a bit

/*****************************************************************************
 * start lfa and play tone to indicate successful start
 *****************************************************************************/
 uint8_t lfaStatus = lfa.begin();

 if(lfaStatus){
  Serial.println("lfa ok");
  playBeep2();        // change tone later so it's unique to lfa start
 } // end if, lfa began normally
 else{
  
  // do something else here maybe set mofe to off
  while(1);       // trap exectution
 } // end else, lfa didn't begin properly
  
  pinMode (ddPin, OUTPUT);            // set ddPin(5) to output, that's from arduino to roomba
  pinMode(buttonPin, INPUT_PULLUP);   // set the buttonPin(12) as in pullup input pin
  
  // *** wake up roomba
  wakeUp(); // 2 second delay built into the wakeUp() function

  // ** set roomba mode
  setMode(128);           // put roomba into passive mode
  setMode(131);           // put roomba into safe mode
                          // roomba is now ready to receive commands
                              
  //readyDance();         // jiggle to signal ready to go
  playBeep();             // beep once to indicate waiting for button press
  waitForButton();        // wait for the button to be pushed
  
  //Serial.println("roomba initialized into safe mode, ready to go");
} // end setup

void loop(){
  
  // send a high and low ping from the sonar sensor
  ping();
  distance = calcDistance();    // calculate the distance to an object
  

// drive forward until an obstacle is detected
  while((int)distance > SAFEDIST){
    Serial.print("distance to obstacle: ");
    Serial.println(distance);
    
   follow(); 
   ping();
   distance = calcDistance();
   //Serial.print("distance to obstacle: ");
   //Serial.println(distance);
  } // end while
  stopDrive();
} // end loop

// ************** my functions *****************
/*****************************************************
 * main line following funciton
 * will get error from the sensor
 * calcuate the power that should be sent to each wheel
 * drive the roomba with the calculated power
 * 
 * Interface:
 * in: no parameters
 * out: nothing out
 ********************************************************/
 void follow(void){
  int error;            // error value read from sensor
  int leftSpeed;        // speed of left wheel motor
  int rightSpeed;       // speed of right wheel motor
  
  error = lfa.getPosition();// get error reading from sensor
  Serial.println(error);
  // calculate left and right motor speeds
  if(error > 0){
    leftSpeed = BASESPEED + (int)BASESPEED * CORRECTION;     // increase left wheel speed by 25%
    rightSpeed = BASESPEED;         // set right wheel speed to base
  } // end > 0 veering left
  else
  if(error < 0){
    leftSpeed = BASESPEED;         // set left wheel speed to base 
    rightSpeed = BASESPEED + (int)BASESPEED * CORRECTION;   // increase right wheel speed by 25%     
  } // end < 0 veering right
  else
  {
    leftSpeed = leftSpeed;
    rightSpeed = rightSpeed;
  } // END == 0 going straight
  
  // drive motors
  driveWheels(rightSpeed, leftSpeed);
  //driveWheels(0, 0);
 } // end follow
 
/*************************************************
* receive somar echo and calculate distance
***************************************************/
float calcDistance(void){
  float d;
  // reset pinmode to input to receive the echo
  pinMode(SIG, INPUT);
  
  // wait for SIG to go high, first echo, start timing
  rxTime = pulseIn(SIG, HIGH);
  
  // calculate distance based on signal return time
  d = (float)rxTime * 34 / 2000.0;    // convert using mach 1 
 return d; 
} // end ping

/*************************************************
* send a high and low ping from the sonar sensor
***************************************************/
void ping(void){
  pinMode(SIG, OUTPUT);
  digitalWrite(SIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SIG, LOW);
} // end ping
  
/************************************************
 * beep to indicate roomba is waiting for a button press
 ***************************************************/
 void playBeep(void){
   mySerial.write("\x8c\x01\x04\x42\x20\x3e\x20\x42\x20\x3e\x20"); // [140] [1] [4] [68] [32]
   mySerial.write("\x8d\x01"); // [141] [1] play it (in slot 1)
 } // end playBeed

 /************************************************
 * beep to indicate lfa has been started
 ***************************************************/
 void playBeep2(void){
   mySerial.write("\x8c\x02\x04\x4c\x10\x4a\x10\x48\x10\x47\x10"); // [140] [1] [4] [68] [32]
   mySerial.write("\x8d\x02"); // [141] [2] play it (in slot 2)
 } // end playBeed

/**********************************************
 * do a little dance to signal that roomba is in safe mode
 * and is ready to start reciving and transmitting
 **********************************************/
 void readyDance(void){
  driveWheels(50, -50);
  delay(250);
  driveWheels(-50, 50);
  delay(500);
  driveWheels(50, -50);
  delay(250);
  stopDrive();
 } // end readyDance()

/********************************************
 * wait for button push to continue
 ********************************************/
void waitForButton(void){
  while(digitalRead(buttonPin)==HIGH){
    //Serial.print("button state: ");
    //Serial.println(digitalRead(buttonPin));
    //Serial.println("push button to proceed");
  } // end while button pin
} // end of waitForButton()

/*******************************************
 * send code 137 to roomba to stop all motors
 ********************************************/
void stopDrive(void){
  mySerial.write(byte(137));
  mySerial.write(byte(0));
  mySerial.write(byte(0));
  mySerial.write(byte(0));
  mySerial.write(byte(0));

// shut down the oi, roomba won't respond to any commands
// cycle the power to re-set mode.
  //mySerial.write(byte(173)); 

// put roomba into passive mode so it won't 
// respond to oi commands
  setMode(128);
                                
} // end stopdrive()

/***********************************************
 * direct drive wheel motors
 * 4 bytes are
 * right velocity high byte
 * right velocity low byte
 * left velocity high byte
 * left velocitn low byte
 * 
 * constrain is used to ensure that the parameter values are within 
 * range for driving roomba's wheels
 ******************************************************/
void driveWheels(int right, int left)
{
  // compensate for left pull
  //float comp = left * 0.1;              // compensate for a left pull in my roomba
  //left += (int)comp;
  //Serial.print("left comp speed ");
  //Serial.println(left);
  constrain(right, -500, 500);
  constrain(left, -500, 500);
  
  mySerial.write(145);            // send code for dirct drive
  mySerial.write(right >> 8);     // send right velocity high byte
  mySerial.write(right);          // send right velocity low byte
  mySerial.write(left >> 8);      // send left velocity high byte
  mySerial.write(left);           // send left velocity low byte
 } // end drive wheels

 void wakeUp (void)
{
  //setWarningLED(ON);
  //Serial.println("wake up, wake up.");
  digitalWrite(ddPin, HIGH);
  delay(100);
  digitalWrite(ddPin, LOW);
  delay(500);
  digitalWrite(ddPin, HIGH);
  delay(2000);
}

void setMode(int modeCode){
  mySerial.write(modeCode);       // put roomba into passive mode
  delay(20);                      // wait for mode change to propegate
} // end setMode

void getInfo(void){
  if (mySerial.available())   //First order of business: listen to Roomba
    Serial.println(mySerial.read());   //writes to USB input from soft serial if connected to laptop serial monitor
  
} // end getInfo

