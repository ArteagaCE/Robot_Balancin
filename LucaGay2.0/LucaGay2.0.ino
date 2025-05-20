#include <Wire.h>

// Pines de conexión al driver L298N
const int enA = 25;   // Pin Enable A (control de velocidad PWM)
const int in1 = 26;   // Pin de entrada 1 para el motor A
const int in2 = 27;   // Pin de entrada 2 para el motor A
const int enB = 13;   // Pin Enable A (control de velocidad PWM)
const int in3 = 33;   // Pin de entrada 1 para el motor A
const int in4 = 32;   // Pin de entrada 2 para el motor A

// Dirección I2C del MPU6050
const int MPU6050_ADDR = 0x68;

// Valores de offset y factor de escala obtenidos en la calibración
float accelOffsetX = 339.92;
float accelOffsetY = -48.66;
float accelOffsetZ = 1072.10;
float accelScaleFactorX = -1.00;
float accelScaleFactorY = -1.00;
float accelScaleFactorZ = -1.01;

// Variables para almacenar las lecturas del sensor (calibradas)
float accelXCal, accelYCal, accelZCal;
float gyroX, gyroY, gyroZ;

// Variables para el filtro de Kalman (simplificado para el ángulo de cabeceo - pitch)
float pitch = 0;       // Ángulo de cabeceo estimado
float pitchRate = 0;   // Velocidad angular de cabeceo medida
float kalmanGain = 0.5; // Ganancia del filtro de Kalman (ajustar este valor)

// Tiempo transcurrido entre lecturas
unsigned long previousTime = 0;
float deltaTime;

// Variables globales para la implentación
float P = 0; // Acción Proporcional
float I = 0; // Acción Integral
float D = 0; // Acción Derivativa
float r = 0.0; // Set point (ángulo objetivo: vertical = 0 grados)
float y = 0;   // Estado actual (ángulo filtrado)
float u = 0;   // Salida del PID
int uPWM = 0;  // Valor PWM para el motor
float anguloPitchFiltrado = 0;
int velocidad = 0;

long TsMicroSegundos = 10000;
float TsSegundos = 0.01; // Corrección: 10000 microsegundos = 0.01 segundos

// ... (Funciones initMPU6050, leerSensoresCalibrados, Blink, obtenerPitchFiltradoCalibrado, MovingAverange) ...
// Función para inicializar el MPU6050
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Registro de Power Management 1
  Wire.write(0);    // Desactivar el modo de sleep
  if (Wire.endTransmission() != 0) {
    Serial.println("Error al inicializar el MPU6050");
    return false;
  }
  // Configurar el giroscopio (opcional)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); // Registro de Configuración del Giroscopio
  Wire.write(0x00); // Rango de ±250 °/s
  Wire.endTransmission();

  // Configurar el acelerómetro (opcional)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // Registro de Configuración del Acelerómetro
  Wire.write(0x00); // Rango de ±2g
  Wire.endTransmission();

  delay(100);
  return true;
}
// Función para leer los datos del MPU6050 y aplicar la calibración
void leerSensoresCalibrados() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Registro del primer byte de la aceleración
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  int16_t rawAccelX = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelY = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelZ = (Wire.read() << 8) | Wire.read();

  accelXCal = (rawAccelX - accelOffsetX) / 16384.0 * accelScaleFactorX;
  accelYCal = (rawAccelY - accelOffsetY) / 16384.0 * accelScaleFactorY;
  accelZCal = (rawAccelZ - accelOffsetZ) / 16384.0 * accelScaleFactorZ;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43); // Registro del primer byte de la velocidad angular
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroZ = (Wire.read() << 8) | Wire.read();

  gyroX = rawGyroX / 131.0;
  gyroY = rawGyroY / 131.0;
  gyroZ = rawGyroZ / 131.0;
}

// Funcion Parpadeo
void Blink(unsigned long interval)
{
	static unsigned long previousMillis = 0;        // will store last time LED was updated
	//const long interval = 1000;           // interval at which to blink (milliseconds)
	unsigned long currentMillis = millis();
	static bool estadoPin=false;
	
	if(currentMillis - previousMillis > interval) 
	{
		previousMillis = currentMillis;
		estadoPin==false? estadoPin=true : estadoPin=false;
		digitalWrite(2, estadoPin);
	}
}

// Función para leer los datos del MPU6050 y aplicar un filtro de Kalman simplificado (con lecturas calibradas)
float obtenerPitchFiltradoCalibrado() {
  leerSensoresCalibrados();

  unsigned long currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  pitch = pitch + gyroY * deltaTime;

  float accelPitch = atan2(accelXCal, sqrt(accelYCal * accelYCal + accelZCal * accelZCal)) * 180 / PI;

  pitch = kalmanGain * (pitch + gyroY * deltaTime) + (1 - kalmanGain) * accelPitch;

  return pitch;
}

float MovingAverange(float nuevoValor){
	// Filtro Moving Averange implementado con buffer circular

	const long buffersize=11;					// Tama o del Buffer. El tama o utilizable es n-1, por los  ndices.
	static float buffer[buffersize]={0};			// Buffer
	static long indexentrante=1;				// Indice que apunta al valor entrante
	static long indexsaliente=indexentrante+1;	// Indice que apunta al valor saliente. Debe estar adelante de indexentrante
	static float suma=0;						// valor donde se suma el contenido del buffer.
	
	
	// Carga del nuevo valor y actualizaci n del  ndice.
	buffer[indexentrante]=nuevoValor;
	suma=suma+buffer[indexentrante]-buffer[indexsaliente];
	
	indexentrante==(buffersize-1)? indexentrante=0 : indexentrante++;
	indexsaliente==(buffersize-1)? indexsaliente=0 : indexsaliente++;
	return suma/(buffersize-1);

}

void PID() {
  float Kp = 40;
  float Ki = 2;
  float Kd = 0;
  float ek = 0;
  static float yk_1 = 0;
  static float Ikplus1 = 0;

  ek = r - y; // Error = Setpoint - Estado Actual

  // Acción proporcional
  P = Kp * ek;

  // Acción integral
  I = Ikplus1;

  // Acción derivativa
  D = -Kd * (y - yk_1) / TsSegundos;
  D = MovingAverange(D);
  D = constrain(D, -10, 10);

  // Acción de control
  u = P + I + D;

  // Actualización de la acción integral
  if (abs(ek) > 0.1) {
    Ikplus1 = Ikplus1 + Ki * ek * TsSegundos;
    Ikplus1 = constrain(Ikplus1, -4, 4);
  }

  // Actualización del estado anterior
  yk_1 = y;
}

void moverMotores(float controlSalida) {
  int pwmValue = 0;
  //int direction = 0;

  // Mapear la salida del PID al rango de velocidad PWM (ajusta estos rangos)
  pwmValue = map(abs(controlSalida), 0, 50, 0, 255); // Ejemplo de mapeo: salida PID de -50 a 50 a PWM 0 a 255
  pwmValue = constrain(pwmValue, 0, 255);

  // Determinar la dirección del motor según el signo de la salida del PID
  if (controlSalida > 0) {
    // Necesita corregir hacia adelante (ajusta la dirección según tu robot)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (controlSalida < 0) {
    // Necesita corregir hacia atrás (ajusta la dirección según tu robot)
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    // No se necesita corrección
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    pwmValue = 0;
  }


  analogWrite(enA, pwmValue);
  analogWrite(enB, pwmValue);
  velocidad = pwmValue; // Actualiza la variable global de velocidad para el serial print
}


void setup() {

    // Configura los pines como salidas
  pinMode(2,OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Inicializa el motor detenido
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0); // 0% de velocidad

    // Inicializa el motor detenido
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0); // 0% de velocidad

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  if (initMPU6050()) {
    Serial.println("MPU6050 inicializado correctamente");
  } else {
    Serial.println("Falló la inicialización del MPU6050");
    while (1);
  }

  previousTime = millis();
}
void loop() {
  Blink(250);
  anguloPitchFiltrado = obtenerPitchFiltradoCalibrado();
  y = anguloPitchFiltrado; // El ángulo filtrado es el estado actual para el PID

  PID(); // Calcula la salida del PID

  moverMotores(u); // Usa la salida del PID para controlar los motores

  Serial.print("Ángulo de Cabeceo Filtrado (Calibrado): ");
  Serial.print(anguloPitchFiltrado);
  Serial.print(" | AccelX: ");
  Serial.print(accelXCal);
  Serial.print(" | AccelY: ");
  Serial.print(accelYCal);
  Serial.print(" | AccelZ: ");
  Serial.println(accelZCal);
  Serial.print(" | Salida PID (u): ");
  Serial.print(u);
  Serial.print(" | Velocidad motor: ");
  Serial.println(velocidad);

  delay(10);
}
