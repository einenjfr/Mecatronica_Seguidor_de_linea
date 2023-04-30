// Eric Torrontera Ruiz i Juan  
// Versió V1

#include <QTRSensors.h>

// Control motors
int E1 = 6;
int M1 = 7;
int E2 = 5;
int M2 = 4;

// Offset Roda esquerra (Una roda va més ràpida que l'altra)
int Offset = 1.4;
int velocitat = 150;

// Control sensors
#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, 
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// PID
int Error;
int ErrorAnt;
float Kp;
float Kd;
float Ki;
float VD;
float VE;

void setup()
{
  // Pins motors
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // Lectura sensors
  delay(500);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);

  // Valors de la formula PID
  Kp = 0.035;
  Ki = 0.25;
}

void loop()
{
  // read raw sensor values
  qtra.read(sensorValues);
  
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
  // 1023 means minimum reflectance
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }

  //Calculem marge d'error.
  ErrorAnt = Error;
  Error = + sensorValues[0] + sensorValues[1] + sensorValues[2] - sensorValues[3] - sensorValues[4] - sensorValues[5];

  // Mostra el marge d'error entre sensors
  Serial.print("Error: ");
  Serial.print(Error);
  Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor

  // Mostra el valor de correcció de KP
  Serial.print("Kp x Error: ");
  Serial.print((Kp * Error));
  Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor

  // Mostra el valor de l'error anteterior (Derivatiu)
  Serial.print("Error Anterior: ");
  Serial.print((ErrorAnt));
  Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor

  // Mostra el valor de les rodes
  VD = 170 - (Kp * Error);
  VE = 189 + (Kp * Error);

  Serial.print("VD: ");
  Serial.print(VD);
  Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor

  Serial.print("VE: ");
  Serial.print(VE);
  Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor

  Serial.println();

  // Control velocitat i direcció dels motors
  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);

  // Control dels extrems (Igual no fa falta o map)
  if(VD > 255)
  {
    analogWrite(E1, 255); //PWM Speed Control
  }
  else if (VD < 0)
  {
    analogWrite(E1, 0); //PWM Speed Control
  }
  else if (0 <= VD <= 255)
  {
     analogWrite(E1, VD); //PWM Speed Control
  }

  if(VE > 255)
  {
    analogWrite(E2, 255); //PWM Speed Control
  }
  else if (VE < 0)
  {
    analogWrite(E2, 0); //PWM Speed Control
  }
  else if (0 <= VE <= 255)
  {
     analogWrite(E2, VE); //PWM Speed Control
  } 

  //delay(30);

}
