#include "SerialCommand.h"
#include "EEPROMAnything.h"

#define SerialPort Serial
#define Baudrate 9600

SerialCommand sCmd(SerialPort);
bool debug;
int status = 0;  // 0 = uit, 1 = aan
unsigned long previous, calculationTime;

const int ledPin = 12;
const int button = 13;
const int sensor[] = { A7, A6, A5, A4, A3, A2, A1, A0 };
bool lastButtonState = LOW;
long normalised[8];
float position;
float debugPosition;
const int M1_IN1 = 5;
const int M1_IN2 = 3;
const int M2_IN3 = 6;
const int M2_IN4 = 11;

struct param_t {
  unsigned long cycleTime;
  int speed;
  int black[8];
  int white[8];
  float diff;
  float kp;
} params;

void setup() {
  ADCSRA &= ~((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)); // testen, geeft extra ruis
  ADCSRA |= (1<<ADPS2); // testen, geeft extra ruis // geeft stabiele sensor waarde maar een calc time tijdens run van 792µs
  //ADCSRA = (ADCSRA & 0b11111000) | 0b010; //geeft calc tijd van 712µs maar iets minder stabiele sensor waarden 
  SerialPort.begin(Baudrate);
  pinMode(ledPin, OUTPUT);
  pinMode(button, INPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);

  sCmd.addCommand("set", onSet);
  sCmd.addCommand("run", onStatus);
  sCmd.addCommand("debug", onDebug);
  sCmd.addCommand("calibrate", onCalibrate);
  sCmd.setDefaultHandler(onUnknownCommand);

  EEPROM_readAnything(0, params);

  SerialPort.println("ready");
}

void loop() {

  Aanzetten();
  sCmd.readSerial();
  unsigned long current = micros();
  if (current - previous >= params.cycleTime)
  {
    previous = current;
    // hier je cyclische code, bv. sensoren uitlezen
    //sensor
    for (int i = 0; i < 8; i++) 
    {
      normalised[i] = map(analogRead(sensor[i]), params.black[i], params.white[i], 0, 1024);
    }
    //interpolatie
    int index = 0;
    for (int i = 1; i < 8; i++) if (normalised[i] < normalised[index]) index = i;

    if (normalised[index] > 500) status = 0;

    if (index == 0) position = -20;
    else if (index == 7) position = 20;
    else {
      int sNul = normalised[index];
      int sMinEen = normalised[index - 1];
      int sPlusEen = normalised[index + 1];

      float b = sPlusEen - sMinEen;
      b = b / 2;

      float a = sPlusEen - b - sNul;

      position = -b / (2 * a);
      position += index;
      position -= 3.5;
      position *= 15;
    }
    debugPosition = position;

    //kp waarde
    float error = -position;
    float output = error * params.kp;

    output = constrain(output, -510, 510);

    int powerLeft = 0;
    int powerRight = 0;

    //status on
      if (status == 1) if (output >= 0)
      {
        powerLeft = constrain(params.speed + params.diff * output, -255, 255);
        powerRight = constrain(powerLeft - output, -255, 255);
        powerLeft = powerRight + output;
      }
      else 
      {
        powerRight = constrain(params.speed - params.diff * output, -255, 255);
        powerLeft = constrain(powerLeft + output, -255, 255);
        powerRight = powerRight - output;
      }
      analogWrite(M1_IN1, powerLeft > 0 ? powerLeft : 0);
      analogWrite(M1_IN2, powerLeft < 0 ? -powerLeft : 0);
      analogWrite(M2_IN3, powerRight > 0 ? powerRight : 0);
      analogWrite(M2_IN4, powerRight < 0 ? -powerRight : 0);
      }
    if (status == 0) {
      digitalWrite (ledPin,LOW);
    }
  unsigned long difference = micros() - current;
  if (difference > calculationTime) calculationTime = difference;
}

// --- SerialCommand handlers ---
void onUnknownCommand(char* command) {
  SerialPort.print("unknown command: \"");
  SerialPort.print(command);
  SerialPort.println("\"");
}

void onSet() {
  char* param = sCmd.next();
  char* value = sCmd.next();

  if (strcmp(param, "cycle") == 0) params.cycleTime = atol(value);
  else if (strcmp(param, "speed") == 0) params.speed = atol(value);
  else if (strcmp(param, "status") == 0) {
    if (strcasecmp(value, "on") == 0) status = 1;
    else if (strcasecmp(value, "off") == 0) status = 0;
  } else if (strcmp(param, "diff") == 0) params.diff = atof(value);
  else if (strcmp(param, "kp") == 0) params.kp = atof(value);

  if (strcmp(param, "cycle") == 0)
  {
    long newCycleTime = atol(value);
    float ratio = ((float) newCycleTime) / ((float) params.cycleTime);
    
    params.cycleTime = newCycleTime;
  }

  EEPROM_writeAnything(0, params);
}

void onCalibrate() {
  char* param = sCmd.next();

  if (strcmp(param, "black") == 0) {
    SerialPort.print("start calibrating black... ");
    for (int i = 0; i < 8; i++) params.black[i] = analogRead(sensor[i]);
    SerialPort.println("done");
  } else if (strcmp(param, "white") == 0) {
    SerialPort.print("start calibrating white... ");
    for (int i = 0; i < 8; i++) params.white[i] = analogRead(sensor[i]);
    SerialPort.println("done");
  }
}

void Aanzetten()
{
  if (status == 0)
  {
    if (digitalRead (button) == 1) 
    {
      iTerm = 0;
     status = 1;
      digitalWrite(ledPin, HIGH);
   }
  }
}

void onStatus()
{
  char* parameter = sCmd.next();
  char* value = sCmd.next();
  if (strcmp(parameter, "on") == 0 )
  {
    iTerm = 0;
   status = 1;
   digitalWrite (ledPin,HIGH);
  }
  else if (strcmp(parameter, "off") == 0 )
  {
   status = 0;
   digitalWrite (ledPin,HIGH);
  }
}

void onDebug() {
  SerialPort.print("cycle time: ");
  SerialPort.println(params.cycleTime);

  SerialPort.print("black: ");
  for (int i = 0; i < 8; i++) {
    SerialPort.print(params.black[i]);
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("white: ");
  for (int i = 0; i < 8; i++) {
    SerialPort.print(params.white[i]);
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("normalised: ");
  for (int i = 0; i < 8; i++) {
    SerialPort.print(normalised[i]);
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("position: ");
  SerialPort.println(debugPosition);

  SerialPort.print("speed: ");
  SerialPort.println(params.speed);

  SerialPort.print("status: ");
  SerialPort.println(status);

  SerialPort.print("diff: ");
  SerialPort.println(params.diff);

  SerialPort.print("kp: ");
  SerialPort.println(params.kp);

  SerialPort.print("calculation time: ");
  SerialPort.println(calculationTime);
  calculationTime = 0;
}

