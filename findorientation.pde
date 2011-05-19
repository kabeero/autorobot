/*
  FindOrientation procedure to return car back to charging station using a beacon as a checkpoint
 */
#include <IRremote.h>
#include <Servo.h>

const int ANGLE_PIN = 8; // controls receiver servo
const int TURNING_PIN = 9; // controls car's turning
const int MOTOR_PIN = 10; // controls car's speed
const int RECV_PIN_FRONT = 11; // receiver facing rear
//const int RECV_PIN_BACK = 12; // receiver facing front
const int ULTRA_PIN = 12;
const int ADV_LED = 13;

const int motorReverse = 50;
const int motorStop = 90;
const int motorForward = 103;
const int motorBurst = 120;
const int receiverLeft = 0;
const int receiverCenter = 90;
const int receiverRight = 180;
const int angleLeft = 120;
const int angleCenter = 82;
const int angleRight = 40;

const int angleAdjust = 0;
const int angleAdjustAlt = angleAdjust + 11;
const int adjustedTurn = receiverCenter - angleCenter;

int angleFirstDetect1 = -1;
int angleFirstDetect2 = -1;
int angleLastDetect1 = -1;
int angleLastDetect2 = -1;
int angleBeacon1 = -1;
int angleBeacon2 = -1;
int angleBeacon1Prev = -1;
int angleBeacon2Prev = -1;

// Phase in finding dock
//  0: stopped (charging)
//  1: going to checkpoint 1 (beacon)
//  2: going to checkpoint 2 (dock)
int phase = 1;
char pos = 'n'; // right left or null

int sweepAlt=0; // needed to alternate scan directions (left or right)

unsigned long milliseconds = 0;
unsigned long highAdvancePulseDelay = 15;  // time to set torque to high
unsigned long reverseDelay = 50; // braking power
unsigned long minAdvanceDelay = 150; // minimum advance time
unsigned long maxAdvanceDelay = 350; // maximum advance time
unsigned long minAdvanceDelayDock = 100;
unsigned long maxAdvanceDelayDock = 125;
unsigned long advanceDelay = 200; // time to advance car forward between sweeps

// stop distances for ultrasonic, in inches
unsigned const int distanceCheckpoint = 36, distanceDockStop = 3.75;

Servo receiverServo, motorServo, turningServo;
IRrecv ir_front(RECV_PIN_FRONT);// ir_back(RECV_PIN_BACK);
decode_results resultsFront, resultsBack;

void setup() {
  Serial.begin(38400);
  pinMode(ADV_LED, OUTPUT);

  receiverServo.attach(ANGLE_PIN);
  motorServo.attach(MOTOR_PIN);
  turningServo.attach(TURNING_PIN);

  receiverServo.write(receiverLeft);
  motorServo.write(motorStop);
  turningServo.write(angleCenter);

  ir_front.enableIRIn();
}

void loop() {
  Serial.println("Enter command: [R]eset, [D]ock, [C]harging, [?]Scan");
  char recv = Serial.read();
  while(recv < 0){
    delay(10);
    recv = Serial.read();
  }; // && digitalRead(HOME_BTN) == LOW){};

  if(recv == 'R' || recv == 'r'){
    maxAdvanceDelay = 400;
    minAdvanceDelay = 250;
    pos = 'n';
    phase = 1;
    Serial.println("Reset Phase: Scan for Checkpoint");
  }
  else if(recv == 'C' || recv == 'c'){
    phase = 0;
    Serial.println("Phase: Charging on dock");
  }
  else if(recv == 'D' || recv == 'd'){
    phase = 2;
    Serial.println("Phase: Scan for Dock");
  }
  else {
    for(int z=0;z<2;z++){
      if(phase != 0)
        motorServo.write(motorStop);
      if(maxAdvanceDelay != maxAdvanceDelayDock)
        turningServo.write(angleCenter);

      checkUltrasonic();

      if(phase != 0) {
        if(sweep())
          advance();
      }
    }
  }

  Serial.println("-----------------------------------");
}

int sweep() {
  angleFirstDetect1 = -1;
  angleFirstDetect2 = -1;
  angleLastDetect1 = -1;
  angleLastDetect2 = -1;
  angleBeacon1 = -1;
  angleBeacon2 = -1;
  delay(50);  // delay 50 ms to account for detection @ 0
  if(sweepAlt==receiverRight){     // scan right to left
    for(sweepAlt=receiverRight;sweepAlt>receiverLeft;sweepAlt--){
      receiverServo.write(sweepAlt);
      scan(sweepAlt+angleAdjustAlt);
    }
  }
  else{     // scan left to right
    for(sweepAlt=receiverLeft;sweepAlt<receiverRight;sweepAlt++){
      receiverServo.write(sweepAlt);
      scan(sweepAlt+angleAdjust);
    }
  }
  if(angleFirstDetect1 > -1 && angleLastDetect1 > -1) {
    angleBeacon1 = (int)((angleLastDetect1 - angleFirstDetect1) / 2) + angleFirstDetect1;
    Serial.print("Beacon1 calculated to be @ ");
    Serial.println(angleBeacon1);
  }
  if(angleFirstDetect2 > -1 && angleLastDetect2 > -1) {
    angleBeacon2 = (int)((angleLastDetect2 - angleFirstDetect2) / 2) + angleFirstDetect2;
    Serial.print("Beacon2 calculated to be @ ");
    Serial.println(angleBeacon2);
    if(pos == 'n'){
     if(angleBeacon2 > 90){
       pos = 'l';
     }
     else
       pos = 'r';
    }
  }

  delay(50);

  if(angleBeacon1 < 0 && angleBeacon2 < 0) {
    Serial.println("No signal detected");
    return 0;
  }
  else
    return 1;
}

void scan(int scanningAngle) {

  if (ir_front.decode(&resultsFront)) {  // if the 0-180 receiver found a signal
    double result = resultsFront.value;
    digitalWrite(ADV_LED, HIGH);
    delay(20);
    digitalWrite(ADV_LED, LOW);

    if(result == 0xA90) {
      if(angleFirstDetect1 < 0) {
        angleFirstDetect1 = scanningAngle;
      }
      angleLastDetect1 = scanningAngle; 
    }
    else if(result == 0xB80) {
      if(angleFirstDetect2 < 0) {
        angleFirstDetect2 = scanningAngle;
      }
      angleLastDetect2 = scanningAngle;
    }
    else if(result == 0xC70) {
    }
    else if(result == 0xE50) {
      if(angleFirstDetect1 < 0) {
        angleFirstDetect1 = scanningAngle;
      }
      angleLastDetect1 = scanningAngle; 
    }

    ir_front.resume();
  }

  /*
  if (ir_back.decode(&resultsBack)) {  // if the 180-360 receiver found a signal
   double result = resultsBack.value;
   digitalWrite(ADV_LED, HIGH);
   delay(20);
   digitalWrite(ADV_LED, LOW);
   
   if(result == 0xA90) {
   if(angleFirstDetect1 < 0) {
   angleFirstDetect1 = scanningAngle + 180;
   }
   angleLastDetect1 = scanningAngle + 180; 
   }
   else if(result == 0xB80) {
   if(angleFirstDetect2 < 0) {
   angleFirstDetect2 = scanningAngle + 180;
   }
   angleLastDetect2 = scanningAngle + 180;
   }
   else if(result == 0xC70) {
   }
   else if(result == 0xE50) {
   if(angleFirstDetect1 < 0) {
   angleFirstDetect1 = scanningAngle + 180;
   }
   angleLastDetect1 = scanningAngle + 180;
   }
   
   ir_back.resume(); // Receive the next value
   }
   */

  delay(10);

}

void advance() {
  if(phase==0) {
    Serial.println("Car is charging");
    checkUltrasonic();
  }
  else if(phase==1){     // go to checkpoint 1 
    if(angleBeacon1 > 0) {
      Serial.println("Advancing car towards Checkpoint...");
      digitalWrite(ADV_LED, HIGH);
      turningServo.write(180 - (angleBeacon1+adjustedTurn)); // head towards Beacon1
      Serial.print("Heading set to ");
      Serial.println(angleBeacon1);
      motorServo.write(motorBurst); // drive at burst speed
      delay(highAdvancePulseDelay); // burst for only a short time
      motorServo.write(motorForward); // drive at forward speed
      Serial.print("Motors @ ");
      Serial.println(motorForward);
      Serial.print("Angle difference squared: ");
      Serial.println(abs(((float)(angleCenter - (angleBeacon1+adjustedTurn)))*(float)(angleCenter - (angleBeacon1+adjustedTurn))));
      Serial.print("Delay: ");
      float motorDelay = 100*advanceDelay/(abs(((float)(angleCenter - (angleBeacon1+adjustedTurn)))*(float)(angleCenter - (angleBeacon1+adjustedTurn))));
      if(motorDelay > maxAdvanceDelay)
        motorDelay = maxAdvanceDelay;
      else if(motorDelay < minAdvanceDelay)
        motorDelay = minAdvanceDelay;
      Serial.println(motorDelay);
      delay(motorDelay - highAdvancePulseDelay);
      motorServo.write(motorReverse);
      delay(reverseDelay);
      motorServo.write(motorStop);
      digitalWrite(ADV_LED, LOW);

      if(checkUltrasonic() < (double)distanceCheckpoint){
        phase = 2;
        rotate();
      }

    }
  }
  else if(phase==2){      // go to dock (checkpoint 2)
    if(angleBeacon2 > 0) {
      Serial.println("Advancing car towards Dock...");
      digitalWrite(ADV_LED, HIGH);
      turningServo.write(180 - (angleBeacon2+adjustedTurn)); // head towards Beacon2 
      Serial.print("Heading set to ");
      Serial.println(angleBeacon2);
      motorServo.write(motorBurst); // drive at burst speed
      delay(highAdvancePulseDelay); // burst for only a short time
      motorServo.write(motorForward); // drive at forward speed       Serial.print("Motors @ ");
      Serial.println(motorForward);
      Serial.print("Angle difference squared: ");
      Serial.println(abs(((float)(angleCenter - (angleBeacon2+adjustedTurn)))*(float)(angleCenter - (angleBeacon2+adjustedTurn))));
      Serial.print("Delay: ");
      float motorDelay = 100*advanceDelay/(abs(((float)(angleCenter - (angleBeacon2+adjustedTurn)))*(float)(angleCenter - (angleBeacon2+adjustedTurn))));
      if(motorDelay > maxAdvanceDelay)
        motorDelay = maxAdvanceDelay;
      else if(motorDelay < minAdvanceDelay)
        motorDelay = minAdvanceDelay;
      Serial.println(motorDelay);
      delay(motorDelay - highAdvancePulseDelay);       
      motorServo.write(motorReverse);
      delay(reverseDelay);
      motorServo.write(motorStop);
      digitalWrite(ADV_LED, LOW);

      double dist = checkUltrasonic();

      if(dist < (double)distanceDockStop){
        phase = 0;
      }
      else if(dist < 4 * (double)distanceDockStop){
        turningServo.write(angleCenter);
        maxAdvanceDelay = maxAdvanceDelayDock;
        minAdvanceDelay = minAdvanceDelayDock;
      }

    }
  }
}

void rotate() {
   // turn the car when it goes from phase 1 to 2
  if(pos == 'n'){
   
  }
  else if(pos == 'r'){
   Serial.println("Turning car right to find Dock");
   turningServo.write(180 - angleRight);
   motorServo.write(motorForward);
   delay(minAdvanceDelay);
   motorServo.write(motorReverse);
   delay(reverseDelay);
   motorServo.write(motorStop);
   
   sweep();

   if(angleBeacon2 == -1){
     Serial.println("Turning car right to find Dock");
     turningServo.write(180 - angleRight);
     motorServo.write(motorForward);
     delay(minAdvanceDelay);
     motorServo.write(motorReverse);
     delay(reverseDelay);
     motorServo.write(motorStop);
     sweep();
  
     if(angleBeacon2 == -1){
       Serial.println("Turning car right to find Dock");
       turningServo.write(180 - angleRight);
       motorServo.write(motorForward);
       delay(minAdvanceDelay);
       motorServo.write(motorReverse);
       delay(reverseDelay);
       motorServo.write(motorStop);
      }
      else
        advance();
    }
    else
      advance();
  }
  else if(pos == 'l'){
   Serial.println("Turning car left to find Dock");
   turningServo.write(180 - angleLeft);
   motorServo.write(motorForward);
   delay(minAdvanceDelay);
   motorServo.write(motorReverse);
   delay(reverseDelay);
   motorServo.write(motorStop);

   sweep();

   if(angleBeacon2 == -1){
     Serial.println("Turning car left to find Dock");
     turningServo.write(180 - angleLeft);
     motorServo.write(motorForward);
     delay(minAdvanceDelay);
     motorServo.write(motorReverse);
     delay(reverseDelay);
     motorServo.write(motorStop);

     sweep();
  
     if(angleBeacon2 == -1){
       Serial.println("Turning car left to find Dock");
       turningServo.write(180 - angleLeft);
       motorServo.write(motorForward);
       delay(minAdvanceDelay);
       motorServo.write(motorReverse);
       delay(reverseDelay);
       motorServo.write(motorStop);
      }
      else
        advance();
    }
    else
      advance();
  } 
}

double checkUltrasonic(){ 
  double inches, duration;

  digitalWrite(ADV_LED, HIGH);
  pinMode(ULTRA_PIN, OUTPUT);
  digitalWrite(ULTRA_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(ULTRA_PIN, LOW);

  pinMode(ULTRA_PIN, INPUT);
  duration = pulseIn(ULTRA_PIN, HIGH);
  
  digitalWrite(ADV_LED, LOW);
  inches = double(duration/130);
  Serial.print(inches);  Serial.print(" in (");  Serial.print(duration);  Serial.println(" us)");

  return inches;
}

