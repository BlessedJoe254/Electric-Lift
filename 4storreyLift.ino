#include <NewPing.h>

// Motor pins
const int in1 = 8;
const int in2 = 9;
const int en  = 10;

// Ultrasonic pins
#define TRIG_PIN 6
#define ECHO_PIN 7
#define MAX_DIST 200

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

// Floor distances (cm)
const int floorHeights[4] = {2, 11, 21, 37}; // Ground = 2cm, Top = 32cm
const int tolerance = 2;

// Buttons
const int upButton = 4;    // goes to Ground floor
const int downButton = 5;  // goes to Top floor

// Buzzer
const int buzzer = 11;

// Motor speed settings
const int maxSpeed = 115;
const int minSpeed = 40;
const int softMinSpeed = 60; // for soft stop, ensure lift reaches top
const int accelStep = 5;
const int accelDelay = 30;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en, OUTPUT);

  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);

  pinMode(buzzer, OUTPUT);

  stopMotor();
  digitalWrite(buzzer, LOW);
  Serial.begin(9600);
}

void loop() {
  // Check UP button → go to Ground
  if (digitalRead(upButton) == LOW) {
    beepOnce();
    moveToFloor(0); // Ground floor
    beepBuzzer();
    while(digitalRead(upButton) == LOW); // wait until released
    delay(50); // debounce
  }

  // Check DOWN button → go to Top
  if (digitalRead(downButton) == LOW) {
    beepOnce();
    moveToFloor(3); // Top floor
    beepBuzzer();
    while(digitalRead(downButton) == LOW); // wait until released
    delay(50); // debounce
  }
}

// ---------- Move lift with soft stop ----------
void moveToFloor(int floor) {
  int targetDistance = floorHeights[floor];
  int distance = getDistance();

  if (distance < targetDistance) { // moving up
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    smoothMoveUp(targetDistance);
  }
  else if (distance > targetDistance) { // moving down
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    smoothMoveDown(targetDistance);
  }

  stopMotor();
}

// ---------- Smooth movement UP ----------
void smoothMoveUp(int targetDistance) {
  while (true) {
    int distance = getDistance();
    int diff = targetDistance - distance;

    int speed;
    if (diff <= 8) { // start deceleration
      speed = map(diff, 0, 8, softMinSpeed, maxSpeed);
    } else {
      speed = maxSpeed;
    }
    speed = constrain(speed, softMinSpeed, maxSpeed);
    analogWrite(en, speed);

    if (abs(distance - targetDistance) <= tolerance) break;
    delay(accelDelay);
  }
}

// ---------- Smooth movement DOWN ----------
void smoothMoveDown(int targetDistance) {
  while (true) {
    int distance = getDistance();
    int diff = distance - targetDistance;

    int speed;
    if (diff <= 8) { // start deceleration
      speed = map(diff, 0, 8, softMinSpeed, maxSpeed);
    } else {
      speed = maxSpeed;
    }
    speed = constrain(speed, softMinSpeed, maxSpeed);
    analogWrite(en, speed);

    if (abs(distance - targetDistance) <= tolerance) break;
    delay(accelDelay);
  }
}

// ---------- Distance ----------
int getDistance() {
  delay(50);
  int d = sonar.ping_cm();
  if (d == 0) return getDistance();
  Serial.print("Distance: "); Serial.println(d);
  return d;
}

// ---------- Stop motor ----------
void stopMotor() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(en, 0);
}

// ---------- Buzzer ----------
void beepBuzzer() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(200);
  }
}

void beepOnce() {
  digitalWrite(buzzer, HIGH);
  delay(150);
  digitalWrite(buzzer, LOW);
}
