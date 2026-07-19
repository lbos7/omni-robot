#include <ESP32Encoder.h>

const int encoderA = 36;
const int encoderB = 39;
const int pwm = 18;
const int dir = 19;
const int pwmFreq = 20000;
const int pwmRes = 10;
const int maxDutyCnt = pow(2, pwmRes) - 1 ;
const int encoderCPR = 64;
const float gearRatio = 18.75;
const float countsPerRev = ((float)encoderCPR) * gearRatio;

ESP32Encoder encoder;
int duty = 0;
int64_t currentCount = 0;
int64_t prevCount = 0;
float currentVel = 0.0;
float prevVel = 0.0;
float kp = 0.0;
float ki = 0.0;
float kd = 0.0;
float currentTime, prevTime;

void setup() {
  Serial.begin(115200);
  encoder.attachFullQuad(encoderA, encoderB);
  encoder.setCount(0);

  pinMode(dir, OUTPUT);
  digitalWrite(dir, LOW);
  ledcAttach(pwm, pwmFreq, pwmRes);
  ledcWrite(pwm, duty);

  currentTime = micros() / 1e6f;
  prevTime = currentTime;
}

void loop() {
  if (Serial.available()) {
    char input = Serial.read();

    switch (input) {
      case 'w':
        duty += 10;
        break;

      case 's':
        duty -= 10;
        break;
    }
  }

  duty = constrain(duty, -1023, 1023);

  if (duty >= 0) {
    digitalWrite(dir, LOW);     // forward
    ledcWrite(pwm, duty);
  } else if (duty < 0) {
    digitalWrite(dir, HIGH);    // reverse
    ledcWrite(pwm, 1023 + duty);
  }

  currentCount = encoder.getCount();
  currentTime = micros() / 1e6f;
  float dAngle = ((currentCount - prevCount) / countsPerRev) * TWO_PI;
  float dTime = currentTime - prevTime;
  currentVel = dAngle / dTime;
  prevCount = currentCount;
  prevTime = currentTime;
  prevVel = currentVel;
  Serial.println("Current_Velocity: " + String(currentVel));
  delay(100);
}