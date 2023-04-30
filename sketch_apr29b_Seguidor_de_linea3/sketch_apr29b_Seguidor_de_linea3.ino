#include <QTRSensors.h> //Pololu QTR Sensor
  
// Pins dels motors del MD1.3 2A Dual Motor Controller (SKU: DRI0002)
int E1 = 6;
int M1 = 7;
int E2 = 5;
int M2 = 4;

// Corrección del desvío
int Error_Anterior = 0;

// Constants fórmula PID
#define KP 0.1 
#define KI 0 
#define KD 0.2 

// Valors límits dels motors
#define M1_velocitat_minima 180 // Mínima velocitat Motor1 E
#define M2_velocitat_minima 180  // Mínima velocitat Motor2 D
#define M1_velocitat_max 255 // Máxima velocitat Motor 1 E
#define M2_velocitat_max 255 // Máxima velocitat Motor 1 D

// Control sensors
#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading // Si pujem major presició si entren valors extrems no desitjats.
#define EMITTER_PIN  QTR_EMITTERS_OFF   // L'he posat off (Llum ambient)

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
  
void setup()
{
  // Configuració sortida Pins motors
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  delay(1500); // Espera + calibració manual (Per defecte esta entre valors de 30 i 700 mes o menys els possarem entre 0 i 1000)

  //Calibració manual, podriem mirar de fer autocalibració (Crec que dona voltes o algo aixi a un punt T)
  int i;
  for (i = 0; i < 255; i++) // 7,65 segons
  {
    qtra.calibrate(QTR_EMITTERS_ON); // Apunts Pololu - Arduino Library for the Pololu QTR Reflectance Sensors (Pàgina 7)
    delay(30);
  }
}

void loop()
{
  unsigned int sensors[NUM_SENSORS];
  int posicio = qtra.readLine(sensors); // Apunts Pololu - Arduino Library for the Pololu QTR Reflectance Sensors (Pàgina 7)
  int error = posicio - 2500; // Funcionament explicat a la pàgina 7.

  // Fórmula Integral - Derivativa (Falta desenvolupar la part integrativa)
  int velocitatMotor = KP * error + KD * (error - Error_Anterior);
  Error_Anterior = error;

  int velocitatMotorDreta = M2_velocitat_minima - velocitatMotor;
  int velocitatMotorEsquerra = M1_velocitat_minima + velocitatMotor;

  // Mètode que fixa la velocitat als motors
  Ajustar_Velocitat_Motors(velocitatMotorEsquerra, velocitatMotorDreta);
  }
    
  void Ajustar_Velocitat_Motors(int velocitatMotorEsquerra, int velocitatMotorDreta)
{
  // Condicionals cassos extrems (Test caixa blanca)
  if (velocitatMotorEsquerra > M1_velocitat_max ) velocitatMotorEsquerra = M1_velocitat_max; 
  if (velocitatMotorDreta > M2_velocitat_max ) velocitatMotorDreta = M2_velocitat_max;
  if (velocitatMotorEsquerra < 0) velocitatMotorEsquerra = 0; 
  if (velocitatMotorDreta < 0) velocitatMotorDreta = 0; 

  // Control velocitat i direcció dels motors
  digitalWrite(M1,HIGH); // Direcció cap endevant
  digitalWrite(M2,HIGH); // Direcció cap endevant
  analogWrite(E1, velocitatMotorEsquerra); //PWM Speed Control
  analogWrite(E2, velocitatMotorDreta); //PWM Speed Control
}