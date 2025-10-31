#include <SoftwareSerial.h>

SoftwareSerial BTserial(10, 11); // TX, RX van de HC-05
const int led1Pin = 9;           // LED 1 (ON/OFF)
const int led2Pin = 6;          // LED 2 (PWM)

int led2Value = 128;    // Basiswaarde voor LED2 (0-255)

void setup() {
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);

  Serial.begin(9600);
  BTserial.begin(9600);

  Serial.println("Bluetooth Dual LED control ready.");
}

void loop() {
  analogWrite(led2Pin, led2Value);
  if (BTserial.available()) {
    String command = BTserial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("ON")){
      digitalWrite(led1Pin, HIGH);
    }

    if (command.equalsIgnoreCase("OFF")){
      digitalWrite(led1Pin, LOW);
    }

    if (command.startsWith("PWM")) {
      int val = command.substring(4).toInt();
      val = constrain(val, 0, 255);
      led2Value = val; // sla waarde op zodat hij behouden blijft
    }
  }
}