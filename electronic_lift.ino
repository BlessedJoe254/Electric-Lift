// Blessed Joe's One-Floor Lift with Ultrasonic Sensor + Soft Start/Stop + Buzzer
#include <NewPing.h>

// Motor pins
const int in1 = 8;
const int in2 = 9;
const int en  = 10;

// Ultrasonic pins
#define TRIG_PIN 6
#define ECHO_PIN 7
#define MAX_DIST 200  // cm

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

// Target floor distance (from ground in cm)
const int floorHeight = 34; // <-- Distance to 1st floor
const int tolerance   = 2;  // +/- tolerance cm

// Buttons
const int upButton = 4;
const int downButton = 5;

// Buzzer
const int buzzer = 11;

// Motor speed settings
const int maxSpeed = 115;  // max speed (0–255)
const int minSpeed = 40;  // starting speed
const int accelStep = 5;   // increase per step
const int accelDelay = 30; // ms delay between steps

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
  bool upPressed = digitalRead(upButton) == LOW;
  bool downPressed = digitalRead(downButton) == LOW;

  int distance = getDistance();
  Serial.print("Lift Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // --- UP button pressed ---
  if (upPressed) {
    beepOnce();          // alert before moving
    accelerateUp();
    while (true) {
      distance = getDistance();
      if (distance >= floorHeight - tolerance && distance <= floorHeight + tolerance) break;
      if (digitalRead(downButton) == LOW) break; // allow cancel
    }
    decelerate();
    stopMotor();
    beepBuzzer();
  }

  // --- DOWN button pressed ---
  if (downPressed) {
    beepOnce();
    accelerateDown();
    while (true) {
      distance = getDistance();
      if (distance <= 5) break; // near ground
      if (digitalRead(upButton) == LOW) break;  // allow cancel
    }
    decelerate();
    stopMotor();
    beepBuzzer();
  }
}

// ---------- Distance ----------
int getDistance() {
  delay(50);
  int d = sonar.ping_cm();
  if (d == 0) return getDistance(); // retry on bad read
  return d;
}

// ---------- Motor Control ----------
void accelerateUp() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  for (int speed = minSpeed; speed <= maxSpeed; speed += accelStep) {
    analogWrite(en, speed);
    delay(accelDelay);
  }
}

void accelerateDown() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  for (int speed = minSpeed; speed <= maxSpeed; speed += accelStep) {
    analogWrite(en, speed);
    delay(accelDelay);
  }
}

void decelerate() {
  for (int speed = maxSpeed; speed >= minSpeed; speed -= accelStep) {
    analogWrite(en, speed);
    delay(accelDelay);
  }
}

void stopMotor() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(en, 0);
}

// ---------- Buzzer ----------
void beepBuzzer() {
  for (int i = 0; i < 3; i++) {  // Beep 3 times
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
