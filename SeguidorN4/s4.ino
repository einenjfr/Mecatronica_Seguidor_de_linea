#include <QTRSensors.h> //Pololu QTR Sensor

/*
  DEFINICIO DE LES CONSTANTS
*/
// Pins dels motors del MD1.3 2A Dual Motor Controller (SKU: DRI0002)
#define E1 6
#define M1 7
#define E2 5
#define M2 4

// Pins dels leds
#define L_DRETA 8
#define L_ESQUERRA 9


// Constants formula PID
#define KP 0.1 
#define KI 0 
#define KD 0.2 

// Valors limits dels motors
#define VEL_MIN 180 // Minima velocitat
#define VEL_MAX 255 // Máxima velocitat

// Control sensors
#define NUM_SENSORS             6  // Quantitat de sensors utilitzats
#define NUM_SAMPLES_PER_SENSOR  4  // Cada sensor en pren 4 mostres a cada lectura. Si incrementessim major precicio si entren valors extrems no desitjats.
#define EMITTER_PIN  QTR_EMITTERS_OFF   // OFF => Llum ambient

// Variables PID
int ajustat = 0;
int err;
int errDar = 0; // Darrer error
int errTot;


// Pel loop
int velDre;
int velEsq;


// Els sensors (valors 2-7) es corresponen als pins 0-5 a la placa de l'arduino
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensors[NUM_SENSORS];
  
void setup() {
  // GESTIO LLUMS
  Serial.begin(9600);
  pinMode(L_ESQUERRA , OUTPUT);
  pinMode(L_DRETA , OUTPUT);

  // GESTIO MOTORS
  // Configuracio sortida Pins motors
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  delay(1500); // Espera + calibracio manual (Per defecte esta entre valors de 30 i 700 mes o menys els possarem entre 0 i 1000)

  //Calibracio manual, podriem mirar de fer autocalibracio (Crec que dona voltes o algo aixi a un punt T)
  for (int i = 0; i < 255; i++) // 7,65 segons
  {
    qtra.calibrate(QTR_EMITTERS_ON); // Apunts Pololu - Arduino Library for the Pololu QTR Reflectance Sensors (Pagina 7)
    delay(30);
  }
  
}

void loop() {
  // LUMS
  digitalWrite(L_ESQUERRA , HIGH);    // poner el Pin en LOW
  digitalWrite(L_DRETA , HIGH);   // poner el Pin en HIGH
  
  // MOTORS
  //unsigned int sensors[NUM_SENSORS];
  //int posicio = qtra.readLine(sensors); // Apunts Pololu - Arduino Library for the Pololu QTR Reflectance Sensors (Pagina 7)

  // PART PID  
  err = qtra.readLine(sensors) - 2500; // corregim la posicio (funcionament explicat a la pagina 7)

  // Calculem integrativa
  if (ajustat < VEL_MIN && ajustat > VEL_MAX)
    errTot += err;
  // Formula PID
  ajustat = KP * err + KD * (err - errDar) + KI * errTot;
  errDar = err;

  // Ajustem velocitat
  velDre = VEL_MIN - ajustat;
  velEsq = VEL_MIN + ajustat;

  // Metode que fixa la velocitat als motors
  ajustarVel(velEsq, velDre);
  
}
    
  void ajustarVel(int velEsq, int velDre) {
  // Condicionals cassos extrems (Test caixa blanca)
  if (velEsq > VEL_MAX)
    velEsq = VEL_MAX;
  else if (velEsq < 0) {
    velDre = velDre * 1;         // Reduim velocitat a un 60%
    velEsq = 0;
    digitalWrite(L_ESQUERRA , LOW);  // Llum dreta fora
    intermitents(L_DRETA);     // Activacio intermitent
  }
  if (velDre > VEL_MAX)
    velDre = VEL_MAX;
  else if (velDre < 0) {
    velDre = 0; 
    velEsq = velEsq * 1;
    digitalWrite(L_DRETA , LOW);
    intermitents(L_ESQUERRA);
  }


  // Control velocitat i direccio dels motors
  // Direccio cap endevant
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  // PWM => potencia que regulara la velocitat
  analogWrite(E1, velEsq);
  analogWrite(E2, velDre);
}

void intermitents (int dir) {
  // Rafaga on
  digitalWrite(dir, HIGH);
  delay(150);
  // Rafaga off
  digitalWrite(dir, LOW);
  delay(100); 
}
