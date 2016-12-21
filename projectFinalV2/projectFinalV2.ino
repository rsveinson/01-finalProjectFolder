/*****************************************************************************
 * Rob Sveinson
 * Comp 444 final project
 * Line following roomba
 * version 2: follow a line
 * 
 * Using a Sparkfun line foolower array  get a roomba to follow
 * a dark line on a light bachground. This will be developed in 5 stages as follows;
 * 
 * Stage 1 Sonar: Get the roomba to move in a relatively straight line and stop 
 * when approaching an obstacle. This is necessary to protect the line follower
 * array that sticks out in front of the roomba.
 * 
 * sonar functionality later removed
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
 * This is stage 3-5: now that the roomba is following the line it's time to implement and tune the 
 * pid controller. Sart wtith Kp, Ki, and Kd all = 0. add a p value, then an i vlaue, then a d value
 */

#include<SoftwareSerial.h>
#include"Wire.h"              // for I2C serial interface
#include "sensorbar.h"        // libraries for line follower array
//#define SIG A0

// global scope

const int BASESPEED = 175;            // base speed of wheel motors
const byte LFAADDRESS = 0x3E;        // assign an address to the line follower array
//const float PIE = 3.14159;        // value of pi used in calculating angles
//const int SAFEDIST = 20;          // 20cm safety zone, used for sonar
const float CORRECTION = 1.75;     // increase error

// ****** PID constants *****
/* i have left some of the intermediate settings in the code
 *  as comments  to give a sense of the process, it wasn't until
 *  i added the correction constant to increase the error
 *  that i was able to work with sufficiently small Kp to 
 *  allow the oscillations to settle out of the system
 */
//const float Kp = 1.5;
//const float Ki = 0;
//const float Kd = 0;
const float Kp = 0.9;         // proportional coefficient
//const float Kp = 2.41;
const float Ki = 0.0015;         // integral coefficient
//const float Ki = 0.0065;
const float Kd = 0.8;         // derivative coefficient
//const float Kd = .57;

// ***** PID variables *****

int error = 0;            // error reading at time t
int totalError = 0;       // sum of all error readings
int lastError = 0;        // previous error reading
int outputV = 0;        // value output by the PID equation

int leftSpeed;        // speed of left wheel motor
int rightSpeed;       // speed of right wheel motor

int lls = BASESPEED;              // previous left speed
int lrs = BASESPEED;              // previous right speed

// create a new sensor bar object
SensorBar lfa(LFAADDRESS);     // lfa stands for line follower array
//delay(20);                    // delay for a tic
// when things were'nt going according to plan i thought
// that some processing migh be happening before the bar was reay
// hence the delay, as it turns out, not needed. i leave it in to 
// demonstrate a failed problem solving arc

// pin assignments for arduino to roomba 7 mini-din port

int rxPin = 9;        // from arduino(9) to roomba pin(4) white wire
int txPin = 11;       // to arduino(11) from roomba pin(3) red wire
int ddPin = 5;        // device detect used to wake up roomba,
                      // from arduino(5) to roomba pin(5) yellow wire
int buttonPin = 12;   // hold operation until button is pressed

// sonar variables
//unsigned long rxTime;   // return time of sonar pulse
//float distance;         // distance calculated form return time

SoftwareSerial mySerial(rxPin, txPin);  // new port with 9rx and 11tx
                                  // this means pin 4 on roomba connects to pin 9 on arduino
                                  // pin 3 on roomba connects to pin 11 on arduino


/*****************************************************************
 * start the serial port to print to serial monitor
 * print opening message to serial monitor
 * 
 * start mySerial at 19200 baud
 * Serial monitor was/is used in development and debugging so 
 * sll serial monitor commands are commented out for the final 
 * version, I leave them in so you can see how they were used to
 * verify, and debug
 * 
 * put roomba into passive mode opcode 128
 * put roomba into safe mode opcode 130 or 131, they seem to do the same thing
 ******************************************************************************/
void setup(){
  //Serial.begin(9600);       // start serial port for monitor
  //Serial.println("hello dave");

  pinMode (ddPin, OUTPUT);            // set ddPin(5) to output, that's from arduino to roomba
  pinMode(buttonPin, INPUT_PULLUP);   // set the buttonPin(12) as in pullup input pin
  
  mySerial.begin(19200);       // start software serial for communicating with roomba
                                // use software serial because roomba's tx doesn't always
                                // carry enough voltage to meet arduino's requirements
                                // not really an issue since we're not using any sensor
                                // data form the roomba, but better safe...
  delay(10);              // set a short delay to allow the command to propegate
  
  // *** wake up roomba
  
  wakeUp(); // 2 second delay built into the wakeUp() function

  // ** set roomba mode
  setMode(128);           // put roomba into passive mode
  setMode(131);           // put roomba into safe(131) or full(132) mode
                          // roomba is now ready to receive commands
                              
  //readyDance();         // jiggle to signal ready to go
  playBeep();             // beep once to indicate waiting for button press
  waitForButton();        // wait for the button to be pushed
  delay(1000);
  //Serial.println("roomba initialized into safe mode, ready to go");


/******************************************************************
 * set up the line follower array
 * clearBarStrobe();
 * set indicator lights to be on while sensor is detecting a dark line
 * alternative is setBarStrobe(); sets indicators to blink each time it reads
 * so you get blinking light when a sensor is detecting a line
 * 
 * I changed to setBarStrobe() because keeping the lights on all the time was 
 * consuming too much battery power, live and learn
 * 
 * clearInvertBits();
 * sets array to detect dark line on light background
 * alternative is setInvertBits(); sets array to detect light line on dark background
 *****************************************************************************/

  //lfa.clearBarStrobe();
  lfa.setBarStrobe();
  lfa.clearInvertBits();
  delay(200);              // delay a bit

/*****************************************************************************
 * start lfa and play tone to indicate successful start
 *****************************************************************************/
   bool lfaStatus = lfa.begin();
  
   if(lfaStatus){
    Serial.println("lfa ok");
    playBeep2();        // play a beep to indicate sensor bar ready
   } // end if, lfa began normally
   else{
    
    // do something else here maybe set mode to off
    while(1);   // trap exectution
    //Serial.println("oops");       // print message if sensor bar fails to initialize
   } // end else, lfa didn't begin properly

  waitForButton();      // wait for a button press after all initialization has succeded
  delay(250);           //delay so pin 12 can re-set to HIGH
} // end setup

/*********************************************************************
 *********************************************************************
 *********************************************************************/
void loop(){
 /* I have left the code for the sonar sensor in so you can see
  *  how it would have worked...
  *  
  // send a high and low ping from the sonar sensor
  //ping();
  //distance = calcDistance();    // calculate the distance to an object

  if(distance < SAFEDIST){
    stopDrive();    // stop motors and put roomba into passive mode
    Serial.println("proximity alert: STOP!");
    while(1){}    // trap processing when proximity alarm goes off
  }
*/
// drive forward following a dark line on a light background
    
   follow(); 


  //  stop roomba on button press
  if(digitalRead(buttonPin)==LOW){
    //Serial.println("stopping");
    stopDrive();              // stop wheel motors
    setMode(128);             // put roomba into passive mode
    // putting roomba into passive mode is critical so it will charge 
    // the battery
  }
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
  
  lastError = error;            // keep track of the previous erro for d term  
  error = lfa.getPosition();    // get error reading from sensor
  error *= CORRECTION;     // increase error by CORRECTION so bigger errors will cause more significant correction
  totalError += error;          // sum all errors for i term
  
  /*****************************************************************
   * the PID equation calculates the system output for a given sensor
   * reading by summing the product of the p coefficient and the current
   * error, plus the product of the i coefficient and the sum of all error
   * from the time the system started, plus the d coefficient times the
   * difference between current error and the last error. since the elapsed
   * time between readings is constant there is no need to divide, this gives 
   * the instantaneous slope of the line tangent to the curve of the error
   *************************************************************************/
  outputV = (int)(Kp * error + Ki * totalError + Kd * (error - lastError));
  //Serial.print("output power ");
  //Serial.println(outputV);
  
  // calculate left and right motor speeds
  // split the power increase/decrease btween left and rigt wheels
  if(outputV  > 0){
    leftSpeed = BASESPEED  + (outputV / 2);     // increase left wheel speed by  half outputV
    rightSpeed = BASESPEED - (outputV / 2);     // decrease right wheel by half outputV
  } // end > 0 veering left
  else
  if(outputV < 0){
    leftSpeed = BASESPEED - ((abs(outputV)) / 2);     // decrease right wheel by half outputV 
    rightSpeed = BASESPEED  + ((abs(outputV)) / 2);   // increase right wheel speed by outputV     
  } // end < 0 veering right
 /* I found that when error == 0 nothing need be done, the 
  *  system will continue with the last right and left speeds
  *  this will usually cause an error to be introduced so the 
  *  system will continue with its corrections
  *  else
  {
    leftSpeed = lls;
    rightSpeed = lrs;
    Serial.println("zero error");
  } // END == 0 going straight
 */
  // drive motors
  lls = leftSpeed;        // record last left wheel motor speed
  lrs = rightSpeed;       // record last right wheel motot speed

  // constrain the motor speed, roomba's max speed is 500 mm/sec
  constrain(leftSpeed, 50, 500); 
  constrain(rightSpeed, 50, 500);

  //Serial.print("current error: ");
  //Serial.println(error);
  
  //Serial.print("total error: ");
  //Serial.println(totalError);

  //Serial.print("left motor speed: ");
  //Serial.println(leftSpeed);

  //Serial.print("right motor speed: ");
  //Serial.println(rightSpeed);
  
  driveWheels(rightSpeed, leftSpeed);
  //driveWheels(0, 0);
 } // end follow
 
/*************************************************
* receive somar echo and calculate distance
***************************************************
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
*/
/*************************************************
* send a high and low ping from the sonar sensor
***************************************************
void ping(void){
  pinMode(SIG, OUTPUT);
  digitalWrite(SIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SIG, LOW);
} // end ping
*/  
/************************************************
 * beep to indicate roomba is waiting for a button press
 * the first line stores the sound in roomba's memory
 * in this case it's position 1, 1 note long, play tone 68(converted to hex) for half second
 ***************************************************/
 void playBeep(void){
   mySerial.write("\x8c\x01\x01\x42\x20"); // [140] [1] [1] [68] [32]
   mySerial.write("\x8d\x01"); // [141] [1] play the beep stored in position 1 of roomba's memory
 } // end playBeed

 /************************************************
 * beep to indicate lfa has been started
  * the first line stores the sound in roomba's memory
 * in this case it's position 2, 1 note long, play tone 76(converted to hex for quarter second
 ***************************************************/
 void playBeep2(void){
   mySerial.write("\x8c\x02\x01\x4c\x10\x4a\x10\x48\x10\x47\x10"); // [140] [2] [1] [76] [16]
   mySerial.write("\x8d\x02"); // [141] [2] play the bood in postition 2 of roomba's memory
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
  float comp = left * 0.1;              // compensate for a left pull in my roomba
  left += (int)comp;
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
  //Serial.println("i'm awake.");
}

void setMode(int modeCode){
  mySerial.write(modeCode);       // put roomba into passive mode
  delay(20);                      // wait for mode change to propegate
  //Serial.print("mode set to: ");
  //Serial.println(modeCode);
} // end setMode



