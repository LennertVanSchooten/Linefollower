const int IN1 = 5;   // Motor A pin 1
const int IN2 = 6;   // Motor A pin 2
const int IN3 = 9;   // Motor B pin 1
const int IN4 = 10;  // Motor B pin 2

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  Serial.println("Twee motoren snelheidsverloop test start");
}

void loop() {
  // snelheid van -255 (achteruit) naar 255 (vooruit)
  for (int snelheid = -255; snelheid <= 255; snelheid++) {
    stelMotorSnelheidIn(IN1, IN2, snelheid);  // Motor A
    stelMotorSnelheidIn(IN3, IN4, snelheid);  // Motor B
    Serial.println(snelheid);
    delay(10); // bepaalt hoe snel de verandering verloopt
  }

  // eventueel even stoppen
  stelMotorSnelheidIn(IN1, IN2, 0);
  stelMotorSnelheidIn(IN3, IN4, 0);
  delay(1000);
}

// ---------- Functie om motorrichting en PWM in te stellen ----------
void stelMotorSnelheidIn(int pin1, int pin2, int snelheid) {
  if (snelheid > 0) {
    analogWrite(pin1, snelheid);
    analogWrite(pin2, 0);
  } else if (snelheid < 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, -snelheid); // negatief omzetten naar positief
  } else {
    // motor uit
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}
