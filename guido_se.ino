//imports
#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

//declaring objects
//DC motor objects
AF_DCMotor rf(1);
AF_DCMotor lf(2);
AF_DCMotor rb(4);
AF_DCMotor lb(3);
AF_DCMotor fan(5);

//Servo motor object
Servo myServo;

//initializing variables
//controlling speed
byte motorSpeed = 80;
int motorOffset = 15;
int turnSpeed = 70;

//controlling sensor signals, distance, and time
uint8_t trig = A1;                                               //pin that sends the signal from ultrasonic sensor
uint8_t echo = A0;                                              //pin that recieves the signal from ultrasonic sensor
byte maxDist = 150;                                          //objects whose distance is more than maxDist will not be detected
byte stopDist = 50;                                          //minimum distance from an object to stop
byte stopGarbageDistance = 25;
float maxTime = 2 * (maxDist + 10) / 100 / 340 * 1000000;    //time between signal is sent and received

//for garbage detection
uint8_t garbage_trig = A4;                                               //pin that sends the signal from ultrasonic sensor
uint8_t garbage_echo = A5;                                              //pin that recieves the signal from ultrasonic sensor

//put your setup code here, to run once:
void setup() {

  Serial.begin(9600);

  //setting speed of motors
  rf.setSpeed(motorSpeed);
  rb.setSpeed(motorSpeed);  
  lf.setSpeed(motorSpeed);
  lb.setSpeed(motorSpeed);

  //pauses after some time
  rf.run(RELEASE);
  rb.run(RELEASE);
  lf.run(RELEASE);
  lb.run(RELEASE);

  //setup servo
  myServo.attach(10);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(garbage_trig, OUTPUT);
  pinMode(garbage_echo, INPUT);

  //forward();
  //delay(1000);
}

//calculates the distance to an object
int getDistance() {
  //Serial.print(10);
  unsigned long pulseTime;                      //time taken for the pulse to go back and forth
  int dist;                                     //stores distance

  digitalWrite(trig, HIGH);                     //activates the trig echo pin (input) 
  delayMicroseconds(10);                        //10 microsecond pulse
  digitalWrite(trig, LOW);                      //deactivate the trig echo pin (input)

  pulseTime = pulseIn(echo, HIGH, maxTime);     //reads a pulse (either HIGH or LOW) on echo. Returns the length of the pulse in microseconds
  dist = (float)pulseTime * 340 / 2 / 10000;    //calculate the object distance based on the pulse time

  return dist;
}

//calculates the distance to an object (from garbage sensor)
int getGarbageDistance() {
  //Serial.print(10);
  unsigned long pulseTime;                      //time taken for the pulse to go back and forth
  int dist;                                     //stores distance

  digitalWrite(garbage_trig, HIGH);                     //activates the trig echo pin (input) 
  delayMicroseconds(10);                        //10 microsecond pulse
  digitalWrite(garbage_trig, LOW);                      //deactivate the trig echo pin (input)

  pulseTime = pulseIn(garbage_echo, HIGH, maxTime);     //reads a pulse (either HIGH or LOW) on echo. Returns the length of the pulse in microseconds
  dist = (float)pulseTime * 340 / 2 / 10000;    //calculate the object distance based on the pulse time

  return dist;
}

//checks and returns left and right object distances
int checkDirection() {
  int distances[2] = {0, 0};     //index  0: left distance, index 1; right distance
  int turnDirection = 1;         //turning left: 0, going backward/reverse: 1; turning right: 2

  myServo.write(180);            //servo turns by 180 degrees to look left
  delay(500);
  distances[0] = getDistance();  //get distance of object on the left

  myServo.write(0);              //servo turns to 0 degrees to look right
  delay(1000);
  distances[1] = getDistance();  //get distance of object on the right

  //if obstacles on both directions are too far, turn left
  if (distances[0] >= 200 && distances[1] >= 200)
    turnDirection = 0;
  //if both left and right directions are blocked with an obstacle, reverse
  else if (distances[0] <= stopDist && distances[1] <= stopDist)  
    turnDirection = 1;
  //if right side is blocked with an obstacle, turn left
  else if (distances[0] >= distances[1])                        
    turnDirection = 0;
  //if left side is blocked with an obstacle, turn right
  else if (distances[0] < distances[1])                           
    turnDirection = 2;

  return turnDirection;
}

//moves forward
void forward() {
  //setting all motors to run forward
  rf.run(FORWARD);
  rb.run(FORWARD);
  lf.run(FORWARD);
  lb.run(FORWARD);
}

//moves backward
void backward() {
  //setting all motors to run backward
  rf.run(BACKWARD);
  rb.run(BACKWARD);
  lf.run(BACKWARD);
  lb.run(BACKWARD);
}

//turns right
void turnRight(int duration) {
  rf.setSpeed(motorSpeed + turnSpeed);
  rb.setSpeed(motorSpeed + turnSpeed);
  lf.setSpeed(motorSpeed + motorOffset + turnSpeed);
  lb.setSpeed(motorSpeed + motorOffset + turnSpeed);

  rf.run(BACKWARD);
  rb.run(BACKWARD);
  lf.run(FORWARD);
  lb.run(FORWARD);

  delay(duration);

  rf.setSpeed(motorSpeed);
  rb.setSpeed(motorSpeed);
  lf.setSpeed(motorSpeed + motorOffset);
  lb.setSpeed(motorSpeed + motorOffset);

  rf.run(RELEASE);
  rb.run(RELEASE);
  lf.run(RELEASE);
  lb.run(RELEASE);
}

//turns left
void turnLeft(int duration) {
  rf.setSpeed(motorSpeed + turnSpeed);                 
  rb.setSpeed(motorSpeed + turnSpeed);
  lf.setSpeed(motorSpeed + motorOffset + turnSpeed);
  lb.setSpeed(motorSpeed + motorOffset + turnSpeed);

  rf.run(FORWARD);
  rb.run(FORWARD);
  lf.run(BACKWARD);
  lb.run(BACKWARD);

  delay(duration);

  rf.setSpeed(motorSpeed);                           
  rb.setSpeed(motorSpeed);
  lf.setSpeed(motorSpeed + motorOffset);
  lb.setSpeed(motorSpeed + motorOffset);

  rf.run(RELEASE);
  rb.run(RELEASE);
  lf.run(RELEASE);
  lb.run(RELEASE);
}

//stops all motors
void stopMove() {
  rf.run(RELEASE);
  rb.run(RELEASE);
  lf.run(RELEASE);
  lb.run(RELEASE);
}

void collectGarbage() {
  fan.setSpeed(motorSpeed);
}

void collectGarbage() {
  fan.run(RELEASE);
}

//put your main code here, to run repeatedly:
void loop() {
  //forward();
  // NewPing sonar(A1, A0, 100);
  
  myServo.write(90);          //servo set at 90 degrees (points straight ahead)
  delay(750);
  //Serial.print(4);
  int dist = getDistance();   //get current distance from obstacle
  int garbageDist = getGarbageDistance();
  //Serial.print(6);
  //Serial.print(dist);
  printf("distance: %d\n", dist);
  // Serial.println(sonar.ping_cm());

  //moves forward if no objects are within stopping distance
  if (dist >= stopDist) {
    forward();

    //collect garbage if garbage found within garbage collection radar
    while (garbageDist <= stopGarbageDistance)) {
         collectGarbage();
    }

    //otheriwise, stop collection garbage
    stopGarbageCollection();

  }

  //check until the minimum distance is reached
  while (dist >= stopDist) {
    dist = getDistance();
    delay(250);
  }

  //stop motots if obstacle is found
  stopMove();

  //checks right and left object distances before turning
  int turnDirection = checkDirection();

  switch(turnDirection) {
    case 0:
      turnLeft(1500); break;
    case 1:
      turnLeft(1500); break;
    case 2:
      turnRight(1500); break;
  }
}
