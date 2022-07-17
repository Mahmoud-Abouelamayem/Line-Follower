#include <QTRSensors.h>                        //Bibliothek des Sensors hinzufügen
QTRSensors qtr;
const uint8_t SensorCount = 8;                 //Anzahl der Sensore
uint16_t sensorValues[SensorCount];            //Wert jedes Sensors
float Kp = 0.09;                               //Proportionalitätsfaktor
float Ki = 0.0000088;                          //Intergralfaktor
float Kd = 0.85;                               //Differenzialfaktor
int P;                                         //P Anteil
int I;                                         //I Anteil
int D;                                         //D Anteil
int lastError = 0;
boolean onoff = false;
int mode = 8;                                   //Pins des Motortreibers
int aphase = 9;                             
int aenbl = 6;
int bphase = 5;
int benbl = 3;
int buttoncalibrate = 17;                       //Kalibrierungsknopf (pin A3)
int buttonstart = 2;                            //Startknopf (Pin D2)

const uint8_t maxspeeda = 100;                  //Maximale Geschwindigkeit des Motors A
const uint8_t maxspeedb = 100;                  //Maximale Geschwindigkeit des Motors A
const uint8_t basespeeda = 40;                  // Basis-Geschwindigkeit des Motors A
const uint8_t basespeedb = 40;                  // Basis-Geschwindigkeit des Motors B

int motorspeeda;                                //Aktuelle Geschwindigkeit des Motors A
int motorspeedb;                                //Aktuelle Geschwindigkeit des Motors A

void setup()
{
  Serial.begin(9600);                           //BR Monitor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10, 11, 12, 14, 15, 16, 18, 19}, SensorCount);      //Sensor Pins D10, D11, D12, A0, A1, A2, A4, A5
  qtr.setEmitterPin(7);                         //LEDON PIN
  pinMode(mode, OUTPUT);                        //Konfiguieren der Pins als Output
  digitalWrite(mode, HIGH);                     
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  digitalWrite(mode, HIGH);
  
  delay(500);                                   //0.5s verschieben
  pinMode(LED_BUILTIN, OUTPUT);

  boolean Ok = false;
  while (Ok == false)                          //Die Schleife startet nicht solange der Sensor Array nicht kalibriert ist
  { 
    if(digitalRead(buttoncalibrate) == HIGH) 
    {  
      calibration();                        //calibrate the robot for 10 seconds
      Ok = true;
    }
    }
 forward_brake(0, 0);
}
void calibration()                          //Funktion der Kalibrierung
{
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 100; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}
void loop()                               
{
  if(digitalRead(buttonstart) == HIGH) {    //Wenn der Startknopf gedrückt wird
    onoff =! onoff;
    if(onoff = true) {
      delay(1000);              //a delay when the robot starts
    }
    else {
      delay(50);
    }
  }
  if (onoff == true) {
    PID_control();
  }
  else {
    forward_brake(0,0);
  }
}

void forward_brake(int posa, int posb)          //Funktion zum Geben der PWM Signale an Motoren
{
  analogWrite(aphase, LOW);
  analogWrite(bphase, HIGH);
  analogWrite(aenbl, posa);
  analogWrite(benbl, posb);
}
void PID_control()                                           //Der PID Regler
{
  uint8_t soll = 3500;
  uint16_t position = qtr.readLineBlack(sensorValues);       //Lesen des Sensorwertes
  int error = 3500 - position;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki+ D*Kd;                       //Motor-Geschwindigkeit Addition von P, I und D Anteile
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  }
  forward_brake(motorspeeda, motorspeedb);
  
    Serial.print("position : ");     Serial.print(position);     Serial.print(" | ");     
    Serial.print("Gesch.Motor A : ");     Serial.print(motorspeeda);Serial.print(" | ");    Serial.print("Gesch.Motor B : ");     Serial.println(motorspeedb);
   Serial.println();
}
