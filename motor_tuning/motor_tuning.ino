#include <ESP32Encoder.h>

enum GainTuningState {
  KP,
  KI,
  KD,
};

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
const float alpha = 0.2;
const float vel2DutyScale = 1.0 / 57.01364;
const float vel2DutyOffset = -1.805 * vel2DutyScale;

ESP32Encoder encoder;
int commandedDuty = 0;
int64_t currentCount = 0;
int64_t prevCount = 0;
float currentVel = 0.0;
float prevVel = 0.0;
float kp = 0.22;
float ki = 0.2;
float kd = 0.002;
float currentTime, prevTime, stepStart;
enum GainTuningState gainState = KP;
bool doStep = false;
float setpoint = 0.0;
float desiredVel = 6.0;
float velIntError = 0.0;
float prevVelError = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.println("DesiredVel:,ActualVel:");
  encoder.attachFullQuad(encoderA, encoderB);
  encoder.setCount(0);

  pinMode(dir, OUTPUT);
  digitalWrite(dir, LOW);
  ledcAttach(pwm, pwmFreq, pwmRes);
  ledcWrite(pwm, commandedDuty);

  currentTime = micros() / 1e6f;
  prevTime = currentTime;
}

void loop() {
  if (Serial.available()) {
    char input = Serial.read();
    float adjustment = 0.0;
    switch (input) {
      case 'w':
        adjustment += 0.01;
        break;

      case 's':
        adjustment -= 0.01;
        break;

      case 'p':
        gainState = KP;
        Serial.println("gainState set to KP");
        break;

      case 'i':
        gainState = KI;
        Serial.println("gainState set to KI");
        break;

      case 'd':
        gainState = KD;
        Serial.println("gainState set to KD");
        break;

      // Increase desired speed
      case 'r':
        desiredVel += 1.0;
        Serial.print("Desired speed: ");
        Serial.println(desiredVel);
        break;


      // Decrease desired speed
      case 'f':
        desiredVel -= 1.0;
        Serial.print("Desired speed: ");
        Serial.println(desiredVel);
        break;


      // // Increase acceleration
      // case 'a':
      //   accel += 0.5;
      //   Serial.print("Acceleration: ");
      //   Serial.println(accel);
      //   break;


      // // Decrease acceleration
      // case 'z':
      //   accel -= 0.5;
      //   if (accel < 0) accel = 0;
      //   Serial.print("Acceleration: ");
      //   Serial.println(accel);
      //   break;


      // Activate speed command
      case 'g':
        setpoint = desiredVel;
        stepStart = micros() / 1e6f;
        prevTime = stepStart;
        Serial.println("Step command activated");
        break;
    }

    if (gainState == KP) {
      kp += adjustment;
      Serial.print("Kp = ");
      Serial.println(kp, 4);

    } else if (gainState == KI) {
      ki += adjustment;
      Serial.print("Ki = ");
      Serial.println(ki, 4);

    } else if (gainState == KD) {
      kd += adjustment;
      Serial.print("Kd = ");
      Serial.println(kd, 4);
    }
  }

  currentCount = encoder.getCount();
  currentTime = micros() / 1e6f;
  float dAngle = ((currentCount - prevCount) / countsPerRev) * TWO_PI;
  float dTime = currentTime - prevTime;
  if (dTime >= 0.005f) {
    currentVel = alpha * (dAngle / dTime) + (1.0f - alpha) * prevVel;

    float velError = setpoint - currentVel;
    float pTerm = kp * velError;

    if (abs(setpoint) < 0.5) {
      velIntError = 0.0;
    } else {
      velIntError += velError * dTime;
    }
    float iTerm = ki * velIntError;
    iTerm = constrain(iTerm, -0.45, 0.45);

    float velDerError = (velError - prevVelError) / dTime;
    float dTerm = kd * velDerError;

    float duty = pTerm + iTerm + dTerm;
    commandedDuty = (int)(duty * maxDutyCnt);
    commandedDuty = constrain(commandedDuty, -1023, 1023);
    if (commandedDuty >= 0) {
      digitalWrite(dir, LOW);     // forward
      ledcWrite(pwm, commandedDuty);
    } else {
      digitalWrite(dir, HIGH);    // reverse
      ledcWrite(pwm, 1023 + commandedDuty);
    }
    Serial.print(desiredVel);
    Serial.print(",");
    Serial.print(currentVel);
    Serial.print(",");
    Serial.print(kp);
    Serial.print(",");
    Serial.print(ki);
    Serial.print(",");
    Serial.println(kd,4);

    if (currentTime - stepStart > 3.0) {
      setpoint = 0.0;
      digitalWrite(dir, LOW);
      ledcWrite(pwm, 0);
    }

    prevCount = currentCount;
    prevTime = currentTime;
    prevVel = currentVel;
    prevVelError = velError;
  }
}