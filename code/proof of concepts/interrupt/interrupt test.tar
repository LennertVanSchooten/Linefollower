const int knop = 13;
const int led = 12;
bool run;

void setup() {

  pinMode(led, OUTPUT);
  pinMode(knop, INPUT);
  run = LOW;

}

void loop() {
 if (run == false) {
    if (digitalRead(knop) == 1) {
      run = true;
      digitalWrite(led, HIGH);
    }
    else(digitalWrite(led, LOW);)
  }
  delay(500);
  if (run == true) {
    if (digitalRead(knop) == 1) {
      run = false;
      digitalWrite(led, LOW);
    }
  }

}
