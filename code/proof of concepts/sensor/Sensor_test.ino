const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int IR = 10;

void setup() {
  Serial.begin(9600);
  pinMode(IR, OUTPUT);
  digitalWrite(IR, HIGH);
  Serial.println("HY-S301 Analoge Sensor Test Start...");
}

void loop() {
  Serial.print("Sensorwaarden: ");

  for (int i = 0; i < 8; i++) {
    int waarde = analogRead(sensorPins[i]);
    Serial.print(waarde);
    Serial.print("\t");
  }
  Serial.println();

  delay(200);
}
