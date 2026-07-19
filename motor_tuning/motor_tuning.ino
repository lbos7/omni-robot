#include <ESP32Encoder.h>

enum GainTuningState {
  KP,
  KI,
  KD,
};

enum TestState {
  STEP,
  RAMP,
  SIN,
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
float currentTime, prevTime, stepStart, rampStart;
enum GainTuningState gainState = KP;
enum TestState testState = STEP;
bool doStep = false;
float setpoint = 0.0;
float desiredVel = 6.0;
float velIntError = 0.0;
float prevVelError = 0.0;
bool negSetpoint = false;
bool useRamp = false;
bool useSin = false;
float rampTime = 3.0;

void setup() {
  Serial.begin(115200);
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
        if (gainState == KD) {
          adjustment = 0.001;
        } else {
          adjustment = 0.01;
        }
        break;

      case 's':
        if (gainState == KD) {
          adjustment = -0.001;
        } else {
          adjustment = -0.01;
        }
        break;

      case 'p':
        gainState = KP;
        break;

      case 'i':
        gainState = KI;
        break;

      case 'd':
        gainState = KD;
        break;

      case 't':
        testState = STEP;
        break;

      case 'y':
        testState = RAMP;
        break;

      case 'u':
        testState = SIN;
        break;

      // Increase desired speed
      case 'r':
        desiredVel += 1.0;
        break;


      // Decrease desired speed
      case 'f':
        desiredVel -= 1.0;
        break;


      // Increase ramp time
      case 'a':
        rampTime += 0.1;
        break;


      // Decrease acceleration
      case 'z':
        rampTime -= 0.1;
        break;


      // Activate speed command
      case 'g':
        if (testState == STEP) {
          setpoint = desiredVel;
          stepStart = micros() / 1e6f;
          prevTime = stepStart;
        } else if (testState == RAMP)  {
          setpoint = 0.0;
          rampStart = micros() / 1e6f;
          prevTime = rampStart;
        }
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

    if (testState == STEP) {
      float dStepTime = currentTime - stepStart;
      if (dStepTime > 6.0) {
        setpoint = 0.0;
        negSetpoint = false;
        digitalWrite(dir, LOW);
        ledcWrite(pwm, 0);
      } else if (!negSetpoint && dStepTime > 3.0) {
        setpoint = -setpoint;
        negSetpoint = true;
      }
    } else if (testState == RAMP) {
      float dRampTime = currentTime - rampStart;
      if (dRampTime > rampTime + 3.0) {
        setpoint = 0.0;
        digitalWrite(dir, LOW);
        ledcWrite(pwm, 0);
      } else if (dRampTime < rampTime) {
        setpoint = desiredVel * (dRampTime / rampTime);
      }
    }

    currentVel = alpha * (dAngle / dTime) + (1.0f - alpha) * prevVel;
    float velError = setpoint - currentVel;
    float pTerm = kp * velError;

    if (abs(setpoint) < 1.5) {
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
    Serial.print(-desiredVel);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(currentVel);
    Serial.print(",");
    Serial.print(kp);
    Serial.print(",");
    Serial.print(ki);
    Serial.print(",");
    Serial.print(kd,4);
    Serial.print(",");
    Serial.print(gainState);
    Serial.print(",");
    Serial.println(testState);

    prevCount = currentCount;
    prevTime = currentTime;
    prevVel = currentVel;
    prevVelError = velError;
  }
}