#include <ESP32Encoder.h>

ESP32Encoder encoder;
const int encoderA = 36;
const int encoderB = 39;
const int pwm = 18;
const int dir = 19;
int duty = 0;

void setup() {
  Serial.begin(115200);
  // ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(encoderA, encoderB);
  encoder.setCount(0);

  delay(5000);

  pinMode(dir, OUTPUT);
  digitalWrite(dir, LOW);
  // pinMode(pwm, OUTPUT);
  // digitalWrite(pwm, LOW);
  ledcAttach(pwm, 20000, 10);
  ledcWrite(pwm, duty);
  
}

void loop() {
  if (Serial.available()) {
    char input = Serial.read();

    switch (input) {
      case 'w': // forward
        duty += 10;
        break;

      case 's': // reverse
        duty -= 10;
        break;
    }
  }

  duty = constrain(duty, -1023, 1023);

  if (duty >= 0) {
    digitalWrite(dir, LOW);     // forward
    ledcWrite(pwm, duty);
  }
  else if (duty < 0) {

    digitalWrite(dir, HIGH);    // reverse
    ledcWrite(pwm, 1023 + duty);
  }

  // Serial.println("Duty Cycle = " + String(duty / 1023.0));
  double degrees = (encoder.getCount() / (64.0 * 18.75)) * 360.0;
  Serial.println("Encoder degrees = " + String(degrees));
  delay(100);
}