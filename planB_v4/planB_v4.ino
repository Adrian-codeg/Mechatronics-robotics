#include <Servo.h>

// Motor pin definitions
#define STBY1 28
#define PWMA1 10
#define PWMB1 9
#define AI1_1 26
#define AI2_1 24
#define BI1_1 30
#define BI2_1 32

#define STBY2 38
#define PWMA2 8
#define PWMB2 7
#define AI1_2 36
#define AI2_2 34
#define BI1_2 40
#define BI2_2 42

// Ultrasonic sensor pin definitions
#define TRIG_PIN 11
#define ECHO_PIN 12

// Line sensor pin definitions
#define LEFT_SENSOR A0
#define CENTER_SENSOR A1
#define RIGHT_SENSOR A2

// Right color sensor
#define S0R 53
#define S1R 51
#define S2R 49
#define S3R 47
#define sensorOutR 45

// Left color sensor
#define S0L 43
#define S1L 41
#define S2L 39
#define S3L 37
#define sensorOutL 35

// Define servo motors
Servo motor1; // Motor 1
Servo motor2; // Motor 2
Servo motor3; // Motor 3
Servo motor4; // Motor 4

// Motor speed definitions
const int motorSpeed = 150;
const int turnSpeed = 120;

// Threshold definitions
const int blackThreshold = 400; // Black line threshold

// IR sensor pins
int LeftIRPin = 4;
int RightIRPin = 5;

// IR calibration values
float LuT_1_xElements[] = {54, 57, 65, 73, 77, 86, 98, 106, 110, 121, 147, 184, 242, 366, 554, 591, 632, 633};
float LuT_1_yElements[] = {80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 9, 8, 7};
float LuT_1_xSize = 0;
// Function prototypes for the lookup table functions
float LookUpTable1D(float inVal, float elementsArraySize, float xElements[], float yElements[]);
void CheckLuTArrays (float xArraySize, float yArraySize, float xElements[], float yElements[]);

// Optimized servo motor movement code, moving from the current position to the target position
void moveMotorFromCurrentPos(Servo &motor, int targetAngle, int speed) {
  int currentAngle = motor.read();  // Read current angle
  
  if (currentAngle < targetAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      motor.write(angle);
      delay(speed);
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      motor.write(angle);
      delay(speed);
    }
  }
}

// Control servo motor movement (this code makes the motor move first to the start, then to the target), 
// an optimized version is written above
void moveMotor(Servo &motor, int startAngle, int targetAngle, int speed) {
  if (startAngle == targetAngle) { 
    return;
  }
  if (startAngle < targetAngle) {
    for (int angle = startAngle; angle <= targetAngle; angle++) {
      motor.write(angle);
      delay(speed);
    }
  } else {
    for (int angle = startAngle; angle >= targetAngle; angle--) {
      motor.write(angle);
      delay(speed);
    }
  }
}

// Initialize servo motors to default positions
void initializeServoMotors() {
  Serial.println("Initializing servo motors...");
  moveMotorFromCurrentPos(motor1, 45, 20);   // motor1 moves to 45°
  delay(500);
  moveMotorFromCurrentPos(motor2, 0, 20);   // motor2 moves to 0°
  delay(500);
  moveMotorFromCurrentPos(motor3, 90, 20); // motor3 moves to 90°
  delay(500);
  moveMotorFromCurrentPos(motor4, 0, 20);   // motor4 moves to 0°
  delay(500);
  // moveMotor(motor1, 45, 45, 50);   // motor1 moves to 45°
  // delay(500);
  // moveMotor(motor2, 0, 0, 50);   // motor2 moves to 0°
  // delay(500);
  // moveMotor(motor3, 90, 90, 50); // motor3 moves to 90°
  // delay(500);
  // moveMotor(motor4, 0, 0, 50);   // motor4 moves to 0°
  // delay(500);
  Serial.println("Servo motor initialization complete");
}

void setup() {
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(STBY1, OUTPUT);
  pinMode(PWMA1, OUTPUT);
  pinMode(PWMB1, OUTPUT);
  pinMode(AI1_1, OUTPUT);
  pinMode(AI2_1, OUTPUT);
  pinMode(BI1_1, OUTPUT);
  pinMode(BI2_1, OUTPUT);
  pinMode(STBY2, OUTPUT);
  pinMode(PWMA2, OUTPUT);
  pinMode(PWMB2, OUTPUT);
  pinMode(AI1_2, OUTPUT);
  pinMode(AI2_2, OUTPUT);
  pinMode(BI1_2, OUTPUT);
  pinMode(BI2_2, OUTPUT);

  // Activate motor module
  digitalWrite(STBY1, HIGH);
  digitalWrite(STBY2, HIGH);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize line sensor pins
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(CENTER_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  // Initialize color sensor pins
  pinMode(S0L, OUTPUT);
  pinMode(S1L, OUTPUT);
  pinMode(S2L, OUTPUT);
  pinMode(S3L, OUTPUT);
  pinMode(sensorOutL, INPUT);
  pinMode(S0R, OUTPUT);
  pinMode(S1R, OUTPUT);
  pinMode(S2R, OUTPUT);
  pinMode(S3R, OUTPUT);
  pinMode(sensorOutR, INPUT);

  // Attach servo motors
  motor2.attach(5);
  motor3.attach(4);
  motor4.attach(3);
  motor1.attach(6);

  // Initialize servo motors
  initializeServoMotors();

  // Setup lookup table for the IR distance sensor
  LuT_1_xSize = sizeof(LuT_1_xElements) / sizeof(LuT_1_xElements[0]);
  float LuT_1_ySize = sizeof(LuT_1_yElements) / sizeof(LuT_1_yElements[0]); // Note this is a local variable
  // Check the LuT_1 arrays for size and increasing xElement values
  CheckLuTArrays(LuT_1_xSize, LuT_1_ySize, LuT_1_xElements, LuT_1_yElements);
}

// Control a single motor
void runMotor(int dir1, int dir2, int pwmPin, int speed, bool forward) {
  digitalWrite(dir1, forward ? HIGH : LOW);
  digitalWrite(dir2, forward ? LOW : HIGH);
  analogWrite(pwmPin, speed);
}

// Stop all motors
void stopAllMotors() {
  runMotor(AI1_1, AI2_1, PWMA1, 0, true);
  runMotor(AI1_2, AI2_2, PWMA2, 0, true);
  runMotor(BI1_1, BI2_1, PWMB1, 0, true);
  runMotor(BI1_2, BI2_2, PWMB2, 0, true);
}

// Ultrasonic distance measurement function
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return (duration / 2.0) * 0.0343; // Distance = (time/2) × speed of sound
}

// Rotate 180° in place
void rotate180(int duration) {
  Serial.println("Rotating 180° in place...");
  runMotor(AI1_1, AI2_1, PWMA1, 150, true);
  runMotor(BI1_1, BI2_1, PWMB1, 150, true);
  runMotor(AI1_2, AI2_2, PWMA2, 150, false);
  runMotor(BI1_2, BI2_2, PWMB2, 150, false);
  delay(duration); // Rotation duration
  stopAllMotors();
}

// Turn left until the center sensor detects the specified color line
void turnUntilLine(int threshold) {
  while (analogRead(CENTER_SENSOR) < threshold ) {
    runMotor(AI1_1, AI2_1, PWMA1, turnSpeed, true);
    runMotor(BI1_1, BI2_1, PWMB1, turnSpeed, true);
    runMotor(AI1_2, AI2_2, PWMA2, turnSpeed, false);
    runMotor(BI1_2, BI2_2, PWMB2, turnSpeed, false);
  }
  stopAllMotors();
}

// Follow the line using differential control (8 states)
void followLine(int maxDuration, int threshold) {
  unsigned long startTime = millis();
  while (millis() - startTime < maxDuration) {
    int leftValue = analogRead(LEFT_SENSOR);
    int centerValue = analogRead(CENTER_SENSOR);
    int rightValue = analogRead(RIGHT_SENSOR);

    bool leftOnLine = leftValue > threshold;
    bool centerOnLine = centerValue > threshold;
    bool rightOnLine = rightValue > threshold;

    if (centerValue >  threshold) { // Center sensor on line
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed, true);
    } else if (leftValue > threshold) { // Drifting left
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed / 2, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed / 2, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed, true);
    } else if (rightValue > threshold) { // Drifting right
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed / 2, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed / 2, true);
    }

    if( (leftOnLine && centerOnLine && rightOnLine) || ((!leftOnLine && centerOnLine && !rightOnLine)) ) {
      // Go straight
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed, true);
    } else if ( (!leftOnLine && centerOnLine && rightOnLine) ) {
      // Turn right slightly
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed/4, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed/4, true);
    } else if ( (!leftOnLine && !centerOnLine && rightOnLine) ) {
      // Turn right
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed, true);
      runMotor(BI1_1, BI2_1, PWMB1, 0, true);
      runMotor(BI1_2, BI2_2, PWMB2, 0, true);
    } else if ( (leftOnLine && centerOnLine && !rightOnLine) ){
      // Turn left slightly
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed/4, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed/4, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed, true);
    } else if ( (leftOnLine && !centerOnLine && !rightOnLine) ){
      // Turn left
      runMotor(AI1_1, AI2_1, PWMA1, 0, true);
      runMotor(AI1_2, AI2_2, PWMA2, 0, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed, true);
    } else if( (!leftOnLine && !centerOnLine && !rightOnLine) ){
      // Stop
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed/2, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed/2, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed/2, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed/2, true);
    } else {
      // Error, stop
      // runMotor(AI1_1, AI2_1, PWMA1, 0, true);
      // runMotor(BI1_1, BI2_1, PWMB1, 0, true);
      // runMotor(AI1_2, AI2_2, PWMA2, 0, true);
      // runMotor(BI1_2, BI2_2, PWMB2, 0, true);
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed/2, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed/2, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed/2, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed/2, true);
    }

    // Detect obstacles
    float distance = getFilteredDistance();
    Serial.print("Current distance: ");
    Serial.println(distance);

    if (distance < 13.5) { // If distance is less than 12cm
      Serial.println("Obstacle detected, stopping...");
      stopAllMotors();
      return;
    }
  }
  stopAllMotors();
}
// Modified straight driving with obstacle detection function
void moveStraightWithObstacleDetection(int maxDuration) {
  unsigned long startTime = millis();
  while (millis() - startTime < 15000) {
    // Motors drive straight
    runMotor(AI1_1, AI2_1, PWMA1, 150, true);
    runMotor(BI1_1, BI2_1, PWMB1, 150, true);
    runMotor(AI1_2, AI2_2, PWMA2, 150, true);
    runMotor(BI1_2, BI2_2, PWMB2, 150, true);

    // Detect obstacles
    float distance = getFilteredDistance();
    Serial.print("Current distance: ");
    Serial.println(distance);

    if (distance < 12) { // If distance is less than 12cm
      Serial.println("Obstacle detected, stopping...");
      stopAllMotors();
      return;
    }

    delay(50); // Delay for smooth operation
  }
  stopAllMotors();
}

// Filtered ultrasonic distance measurement function
float getFilteredDistance() {
  const int sampleCount = 5;
  float totalDistance = 0;
  int validSamples = 0;

  for (int i = 0; i < sampleCount; i++) {
    float distance = getDistance();
    if (distance > 2 && distance < 400) {
      totalDistance += distance;
      validSamples++;
    }
    delay(10);
  }

  return validSamples > 0 ? totalDistance / validSamples : 400;
}

// Turn left function
void turnLeft(int duration, int speed) {
  Serial.println("Turning left...");
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    runMotor(AI1_1, AI2_1, PWMA1, speed, true);
    runMotor(BI1_1, BI2_1, PWMB1, speed, true);
    runMotor(AI1_2, AI2_2, PWMA2, speed, false);
    runMotor(BI1_2, BI2_2, PWMB2, speed, false);
  }
  stopAllMotors();
}

// Does left color sensor detect red?
bool leftDetectRed(){
  return detectRed(1);
}

// Does right color sensor detect red?
bool rightDetectRed(){
  return detectRed(2);
}

// Helper function for leftDetectRed() and rightDetectRed() to detect if the color sensor sees red
bool detectRed(int LorR){
  int redFrequency = 0;
  int greenFrequency = 0;
  int blueFrequency = 0;
  int S0, S1, S2, S3, S4, sensorOut;
  if(LorR == 1){
    S0 = S0L;
    S1 = S1L;
    S2 = S2L;
    S3 = S3L;
    sensorOut = sensorOutL;
  }else{
    S0 = S0R;
    S1 = S1R;
    S2 = S2R;
    S3 = S3R;
    sensorOut = sensorOutR;
  }
  // Setting frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);

  delay(50);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  greenFrequency = pulseIn(sensorOut, LOW);

  delay(50);
 
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);

  // Reading the output frequency
  blueFrequency = pulseIn(sensorOut, LOW);

  Serial.print(" R = ");
  Serial.print(redFrequency);
  Serial.print(" G = ");
  Serial.print(greenFrequency);
  Serial.print(" B = ");
  Serial.print(blueFrequency);

  if (blueFrequency > 30 || greenFrequency > 30){ // !!! This threshold may need to be adjusted
    // Serial.println(". It is RED!");
    return true;
  }else{
    // Serial.println(" It is WHITE!");
    return false;
  }
}

// Follow red line function
void followRedLine(int duration){

  double speed = motorSpeed*0.6;
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    Serial.print("Left: ");
    bool leftOnLine = leftDetectRed();
    Serial.print(", Right: ");
    bool rightOnLine = rightDetectRed();

    Serial.print(" ");
    Serial.print(leftOnLine);
    Serial.print(" ");
    Serial.print(rightOnLine);
    Serial.println();

    if (!leftOnLine && rightOnLine) { // Only right detects the line
      // Turn right
      runMotor(AI1_1, AI2_1, PWMA1, speed, true);
      runMotor(AI1_2, AI2_2, PWMA2, speed, true);
      runMotor(BI1_1, BI2_1, PWMB1, speed/4, true);
      runMotor(BI1_2, BI2_2, PWMB2, speed/4, true);
    } else if (leftOnLine && !rightOnLine) { // Only left detects the line
      // Turn left
      runMotor(AI1_1, AI2_1, PWMA1, speed/4, true);
      runMotor(AI1_2, AI2_2, PWMA2, speed/4, true);
      runMotor(BI1_1, BI2_1, PWMB1, speed, true);
      runMotor(BI1_2, BI2_2, PWMB2, speed, true);
    } else if (!leftOnLine && !rightOnLine) {
      // Go forward
      Serial.print(" gogo");
      runMotor(AI1_1, AI2_1, PWMA1, speed, true);
      runMotor(AI1_2, AI2_2, PWMA2, speed, true);
      runMotor(BI1_1, BI2_1, PWMB1, speed, true);
      runMotor(BI1_2, BI2_2, PWMB2, speed, true);
      delay(1000);
    } else {
      // ERROR, STOP!
      Serial.print("ohhh ");
      runMotor(AI1_1, AI2_1, PWMA1, 0, true);
      runMotor(AI1_2, AI2_2, PWMA2, 0, true);
      runMotor(BI1_1, BI2_1, PWMB1, 0, true);
      runMotor(BI1_2, BI2_2, PWMB2, 0, true);
    }
  }
  stopAllMotors();
}

// Read left IR distance
float leftIRDistance(){
  int irL = analogRead(LeftIRPin);
  float disL = LookUpTable1D(irL, LuT_1_xSize, LuT_1_xElements, LuT_1_yElements);
  return disL;
}

// Read right IR distance
float rightIRDistance(){
  int irR = analogRead(RightIRPin);
  float disR = LookUpTable1D(irR, LuT_1_xSize, LuT_1_xElements, LuT_1_yElements);
  return disR;
}

void turnToWall(){
  float disL = leftIRDistance();
  float disR = rightIRDistance();
  while (abs(disL - disR) > 15){
    if (disL > disR){ 
      // Turn right
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed/4, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed/4, true);
    }else{
      // Turn left
      runMotor(AI1_1, AI2_1, PWMA1, motorSpeed/4, true);
      runMotor(AI1_2, AI2_2, PWMA2, motorSpeed/4, true);
      runMotor(BI1_1, BI2_1, PWMB1, motorSpeed, true);
      runMotor(BI1_2, BI2_2, PWMB2, motorSpeed, true);
    }
  }
}

void loop() {

  // while(1){ // Used for testing infinite loop

  //   // int d = getFilteredDistance();
  //   // Serial.println(d);

  //   int leftValue = analogRead(LEFT_SENSOR);
  //   int centerValue = analogRead(CENTER_SENSOR);
  //   int rightValue = analogRead(RIGHT_SENSOR);
  //   Serial.println("---------------------");
  //   Serial.print(leftValue);
  //   Serial.print(" ");
  //   Serial.print(centerValue);
  //   Serial.print(" ");
  //   Serial.print(rightValue);
  //   Serial.println("---------------------");


  //   // Delay the next iteration of the loop
  //   delay(200);
  // }

  digitalWrite(23, LOW);
  digitalWrite(25, LOW);
 
  Serial.println("Task start: Rotating 180° in place");
  rotate180(4050*0.8);

  // Serial.println("Calibrating position, aligning with wall");
  // turnToWall();

  Serial.println("Driving straight with obstacle detection...");
  moveStraightWithObstacleDetection(20000); // Maximum straight driving duration is 20 seconds

  Serial.println("Turning left to find black line...");
  turnUntilLine(300);

  Serial.println("Following black line...");
  // followLine(6600, blackThreshold);
  followLine(20000, 300); // Maximum line following duration is 20 seconds, with distance detection, exit if 12cm

  Serial.println("Servo motors start working...");
  moveMotor(motor2, 0, 135, 30);
  delay(500);
  moveMotor(motor3, 90, 60, 30);
  delay(500);
  moveMotor(motor2, 135, 0, 30);
  delay(500);
  moveMotor(motor3, 60, 90, 30);

  Serial.println("Servo motor actions complete, turning left again for 5 seconds...");
  turnLeft(2800*0.8, 120);

  // Serial.println("Following red line");
  // followRedLine(5000);

  Serial.println("Left turn complete, continue driving straight for 5 seconds...");
  moveStraightWithObstacleDetection(4000*0.8); // Finally drive straight for 5 seconds

  Serial.println("Turning left 90°...");
  turnLeft(2800*0.8, 120); // Turn left 90°

  Serial.println("Driving straight for 5 seconds then stopping...");
  moveStraightWithObstacleDetection(6000*0.8);

  Serial.println("Servo motors start working...");
  // Motor1 moves from 45° to 135°
  moveMotor(motor1, 45, 135, 30);
  delay(500);
  // Motor2 moves from 0° to 90°
  moveMotor(motor2, 0, 90, 30);
  delay(500);
  // Motor4 moves from 0° to 120°
  moveMotor(motor4, 0, 120, 30);
  delay(500);
  // Motor3 moves from 90° to 5°
  moveMotor(motor3, 90, 5, 30);
  delay(500);
  // Motor3 moves from 5° to 90°
  moveMotor(motor3, 5, 90, 30);
  delay(500);
  Serial.println("Servo motor actions complete");

  Serial.println("Task complete");
  while (true);
}

// Helper function for IR sensor
float LookUpTable1D (float inVal, float elementsArraySize, float xElements[], float yElements[]) {

  // declare and initialise local variables
  float fraction = 0.0;

  // Scan through the gaps between the elements in the xElements array, and find where the inVal is
  // located. Perform a simple linear interpolation between indexes n and n+1, of xElements
  // and yElements, to calculate the lookup output value relating to the input value

  for (int n = 0; n <= elementsArraySize - 2; n++) {
    if ((inVal >= xElements[n]) && (inVal <= xElements[n + 1])) {
      fraction = (inVal - xElements[n]) / (xElements[n + 1] - xElements[n]);

      // Return the output value and exit the function
      return (fraction * (yElements[n + 1] - yElements[n])) + yElements[n];
    }

  }
  // Return an overflow value if the input value is not within the input range of the xElements array
  return 0xFFFFFFFF;
}

// Another helper function for IR sensor
void CheckLuTArrays (float xArraySize, float yArraySize, float xElements[], float yElements[]) {

  // Check for array size mismatch between the xElements and yElements arrays
  // If found, display an error message, then block the execution of the remainder
  // of the code
  if (xArraySize != yArraySize) { // check for array size mismatch
    Serial.println();
    Serial.println("Error: Number of elements in xElements array is not equal to the number of");
    Serial.println("elements in the yElements array.");
    Serial.println();
    Serial.println("This program will not execute further, and sit in an infinite loop");
    while (1); // Hold the execution at this point
  }

  // Check that the values in xElements are increasing as the index values increase
  // If not, display an error message then block the execution of the remainder of the code
  for (int n = 0; n <= xArraySize - 2; n++) {
    // Scan through the xElements indexes
    if (LuT_1_xElements[n] >= LuT_1_xElements[n + 1]) {
      Serial.println();
      Serial.println("Error: The values in xElements are not all increasing with increasing");
      Serial.println("index values");
      Serial.println();
      Serial.println("This program will not execute further, and sit in an infinite loop");
      while (1); // Hold the execution at this point
    }
  }
}
