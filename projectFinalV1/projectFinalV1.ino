/*****************************************************************************
 * Rob Sveinson
 * Comp 444 final project
 * Line following roomba
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
 * This is stage 0: run the roomba in a straight line and stop when it gets to withing
 * about 20cm of an obstacle. This safety zone may need some adjustment later but that will
 * be a pretty simple matter.
 */

#include<SoftwareSerial.h>
#define SIG A0

// global scope

const float PIE = 3.14159;        // value of pi used in calculating angles
const int SAFEDIST = 10;          // 20cm safety zone, used for sonar

// variables for reading and responding to sensor data

int inByte = 0;
int n = 0;              // simple counter for loop()
int rightEncoderDistance = 0;
int leftEncoderDistance = 0;
float turnAngle = 0.0;

float angleDegrees = 0;

byte encoderData[4];

// pin assignments

int rxPin = 9;
int txPin = 11;
int ddPin = 5;
int buttonPin = 12;           // hold operation until button is pressed

// sonar variables
unsigned long rxTime;
float distance;

SoftwareSerial mySerial(rxPin, txPin);  // new port with 10rx and 11tx
                                  // this means pin 4 on roomba connects to pin 10 on arduino
                                  // pin 3 on roomba connects to pin 11 on arduino

/*****************************************************************
 * start the serial port to print to serial monitor
 * print opening message to serial monitor
 * 
 * start mySerial at 19200 baud
 * 
 * put roomba into passive mode opcode 128
 * put roomba into safe mode opcode 130 or 131, they seem to do the same thing
 */
void setup(){
  Serial.begin(9600);       // start serial port for monitor
  //Serial.println("hello dave");

  mySerial.begin(19200);       // start software serial for communicating with roomba
                                // use software serial because roomba's tx doesn't always
                                // carry enough voltage to meet arduino's requirements
  delay(10);              // set a short delay to allow the command to propegate
  
  pinMode (ddPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  
  // *** wake up roomba
  //Serial.println("PUSH BUTTON TO WAKE UP ROOMBA");
  //waitForButton();
  //delay(2000);
  wakeUp(); // 2 second delay built into the wakeUp() function

  // ** set roomba mode
  mySerial.write(128);        // put roomba into passive mode
  delay(20);                  // clean light should come on

  mySerial.write(131);        // put roomba into safe mode
  delay(20);                  // clean light should go out
                              // roomba is now ready to receive commands through the din port
                              
  //readyDance();               // jiggle to signal ready to go
  playBeep();                 // beep once to indicate waiting for button press
  waitForButton();            // wait for the button to be pushed
  
  //Serial.println("roomba initialized into safe mode, ready to go");
/*
  mySerial.write(byte(137));
  mySerial.write(byte(0));
  mySerial.write(byte(100));
  mySerial.write(byte(0x80));
  mySerial.write(byte (0));    // go forward
*/
  //goForward();
  //driveWheels(150, 250);
  //delay(2000);
  //stopDrive();
  //Serial.println("after forward and stop");



  //driveWheels(250,150);
  //delay(2000);   // for 2 seconds
  //stopDrive();
  //delay(500);

  //driveWheels(150, 150);
  //delay(2000);

  //driveWheels(-250, -250);
  //delay(2000);
  //Serial.println("after reverse");
/*  
  mySerial.write(byte(137));
  mySerial.write(byte(0));
  mySerial.write(byte(0));
  mySerial.write(byte(0));
  mySerial.write(byte(0));
  delay(1000);
*/

  

  //goForward();
  //delay(2000);

  //Serial.println("shutting down, (off mode, opcode 173).");
  //mySerial.write(173);        // change to off mode.

  //float tempAngle = getAngle();      // make sure sensor packet 20 is zeroed out
                                
} // end setup

void loop(){
  float tempAngle = 0.0;
  // send a high and low ping from the sonar sensor
  ping();
  distance = calcDistance();
  
// drive forward until an obstacle is detected
  while((int)distance > 10){
   driveWheels(150, 150); 
   ping();
   distance = calcDistance();
   Serial.print("distance to obstacle: ");
   Serial.println(distance);
  } // end while
  stopDrive();
  delay(2000);
  
  // turn CW about 180 degrees
  turnCCW(183);

  ping();
  distance = calcDistance();
// drive forward until an obstacle is detected
  while((int)distance > 10){
   driveWheels(150, 150); 
   ping();
   distance = calcDistance();
   Serial.print("distance to obstacle: ");
   Serial.println(distance);
  } // end while
  stopDrive();
  delay(2000);
  
  // turn CW about 180 degrees
  turnCCW(183);
  
  //stopDrive();

/*  
  while((int)distance < 100){
    ping();
    distance = calcDistance();
    driveWheels(-100, 100);
  }
*/  
  //stopDrive();
  //mySerial.write(173);      // go to off mode
  
    
} // end loop

// ************** my functions *****************
/*************************************************
* receive somar echo and calculate distance
***************************************************/
void turnCCW(int limit){
  float totalRotation = 0.0;
  float temp = getAngle();             // make sure angle sensor is zeroed out

  driveWheels(100, -100);  
  
  while(totalRotation <= limit){
    delay(15);
    Serial.print("total rotation since last update: ");
    Serial.println(totalRotation);
    totalRotation += getAngle();
  }
} // end turnCW

float getAngle(void){

  int i = 0;
  float rotation = 0.0;
  
  
  mySerial.write(142);
  mySerial.write(20);
  //delay(10);

// 2 bytes, angle high byte, angle low byte
  while(mySerial.available()){
    encoderData[i] = mySerial.read();
    //Serial.println("angle data read");
    i++;
  } // end while
/*
  for(int j = 0; j < i; j++){
    Serial.print("data byte # ");
    Serial.print(j);
    Serial.print("  ");
    Serial.println(encoderData[j]);
  }  // end for
*/
  
  //angleDegrees = (int)(encoderData[0] << 8)|(int)(encoderData[1]&0xFF);
  rotation = (int)word((int)encoderData[0], (int)encoderData[1]);
  rotation /= 0.324056; 
  //Serial.print("degrees turned = ");
  //Serial.println(angleDegrees);

   i = 0;
   return rotation;
} // end get Angle

void getInfo(void){
  if (mySerial.available())   //First order of business: listen to Roomba
    Serial.println(mySerial.read());   //writes to USB input from soft serial if connected to laptop serial monitor
  
} // end getInfo

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
* send a high and low pint from the sonar sensor
***************************************************/
void ping(void){
  pinMode(SIG, OUTPUT);
  digitalWrite(SIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SIG, LOW);
} // end ping
  
/************************************************
 * beep to indicate roomba is waiting for a button press
 */
 void playBeep(void){

   mySerial.write("\x8c\x01\x04\x42\x20\x3e\x20\x42\x20\x3e\x20"); // [140] [1] [4] [68] [32] ... place "start sound" in slot 1
   mySerial.write("\x8d\x01"); // [141] [1] play it (in slot 1)
 
 // byte song3[5]={140, 3, 1, 64, 16};
  //byte playSong3[2]={141, 3};
  //mySerial.write(song3, 5);
  //mySerial.write(playSong3, 2);
/*
  mySerial.write(140);
  mySerial.write(3);
  mySerial.write(1);
  mySerial.write(64);
  mySerial.write(16);

  mySerial.write(141);
  mySerial.write(3);
*/  
  
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
    Serial.print("button state: ");
    Serial.println(digitalRead(buttonPin));
    Serial.println("push button to proceed");
  } // end while button pin
} // end of waitForButton()

/*******************************************
 * wake up roomba by pulsing dd pin 5 high low high
 ********************************************/
void stopDrive(void){
  mySerial.write(byte(137));
  mySerial.write(byte(0));
  mySerial.write(byte(0));
  mySerial.write(byte(0));
  mySerial.write(byte(0));  
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
 */
void driveWheels(int right, int left)
{
  // compensate for left pull
  float comp = left * 0.1;
  left += (int)comp;
  Serial.print("left comp speed ");
  Serial.println(left);
  constrain(right, -500, 500);
  constrain(left, -500, 500);

  Serial.println("in driveWheels");
  
  mySerial.write(145);
  mySerial.write(right >> 8);
  mySerial.write(right);
  mySerial.write(left >> 8);
  mySerial.write(left);
 } // end drive wheels

 /* goForward sends the opcode 137 drive 
  *  the 4 bytes are
  *  velocity high byte, low byte
  *  radius high byte, low byte
  *  this op code doesn't drive the wheel motors directly
  */
 void goForward(void)
 {
  //Serial.println("in goForward");
    mySerial.write(byte(137));
    mySerial.write(byte(0));
    mySerial.write(byte(100));
    mySerial.write(byte(0x80));
    mySerial.write(byte (0));    // go forward
 } // end goForward
 
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
 

