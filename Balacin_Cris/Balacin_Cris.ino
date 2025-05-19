#include <Wire.h>

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
float pitch = 0;         // Ángulo de cabeceo estimado
float pitchRate = 0;     // Velocidad angular de cabeceo medida
float kalmanGain = 0.5;  // Ganancia del filtro de Kalman (ajustar este valor)

// Tiempo transcurrido entre lecturas
unsigned long previousTime = 0;
float deltaTime;

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


void setup() {
  pinMode(2,OUTPUT);
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
  float anguloPitchFiltrado = obtenerPitchFiltradoCalibrado();
  Serial.print("Ángulo de Cabeceo Filtrado (Calibrado): ");
  Serial.print(anguloPitchFiltrado);
  Serial.print(" | AccelX: "); Serial.print(accelXCal);
  Serial.print(" | AccelY: "); Serial.print(accelYCal);
  Serial.print(" | AccelZ: "); Serial.println(accelZCal);
  delay(100);
}