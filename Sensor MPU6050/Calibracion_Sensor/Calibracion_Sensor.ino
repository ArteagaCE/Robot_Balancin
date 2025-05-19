#include <Wire.h>

// Dirección I2C del MPU6050
const int MPU6050_ADDR = 0x68;

// Variables para almacenar las lecturas brutas del acelerómetro
int16_t rawAccelX, rawAccelY, rawAccelZ;

// Variables para almacenar las sumas de las lecturas en cada orientación
long sumAccelX[6] = {0, 0, 0, 0, 0, 0};
long sumAccelY[6] = {0, 0, 0, 0, 0, 0};
long sumAccelZ[6] = {0, 0, 0, 0, 0, 0};

// Número de lecturas a tomar en cada orientación
const int numReadings = 200;

// Función para inicializar el MPU6050
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Registro de Power Management 1
  Wire.write(0);    // Desactivar el modo de sleep
  if (Wire.endTransmission() != 0) {
    Serial.println("Error al inicializar el MPU6050");
    return false;
  }
  // Configurar el acelerómetro (opcional)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // Registro de Configuración del Acelerómetro
  Wire.write(0x00); // Rango de ±2g
  Wire.endTransmission();

  delay(100);
  return true;
}

// Función para leer los datos brutos del acelerómetro
void readRawAccel() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Registro del primer byte de la aceleración (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true); // Solicitar 6 bytes de datos
  rawAccelX = (Wire.read() << 8) | Wire.read();
  rawAccelY = (Wire.read() << 8) | Wire.read();
  rawAccelZ = (Wire.read() << 8) | Wire.read();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  if (initMPU6050()) {
    Serial.println("MPU6050 inicializado correctamente");
    Serial.println("\nComienza la calibración del acelerómetro:");
    Serial.println("Sigue las instrucciones en el monitor serial.");
    Serial.println("Asegúrate de que el sensor esté completamente quieto en cada orientación.");
    delay(1000);
  } else {
    Serial.println("Falló la inicialización del MPU6050");
    while (1);
  }

  Serial.println("\nColoca el sensor con el eje +X apuntando hacia abajo (1g). Presiona ENTER para continuar.");
  while (Serial.available() == 0); Serial.readStringUntil('\n'); delay(500);
  for (int i = 0; i < numReadings; i++) { readRawAccel(); sumAccelX[0] += rawAccelX; sumAccelY[0] += rawAccelY; sumAccelZ[0] += rawAccelZ; delay(2); }

  Serial.println("\nColoca el sensor con el eje -X apuntando hacia abajo (-1g). Presiona ENTER para continuar.");
  while (Serial.available() == 0); Serial.readStringUntil('\n'); delay(500);
  for (int i = 0; i < numReadings; i++) { readRawAccel(); sumAccelX[1] += rawAccelX; sumAccelY[1] += rawAccelY; sumAccelZ[1] += rawAccelZ; delay(2); }

  Serial.println("\nColoca el sensor con el eje +Y apuntando hacia abajo (1g). Presiona ENTER para continuar.");
  while (Serial.available() == 0); Serial.readStringUntil('\n'); delay(500);
  for (int i = 0; i < numReadings; i++) { readRawAccel(); sumAccelX[2] += rawAccelX; sumAccelY[2] += rawAccelY; sumAccelZ[2] += rawAccelZ; delay(2); }

  Serial.println("\nColoca el sensor con el eje -Y apuntando hacia abajo (-1g). Presiona ENTER para continuar.");
  while (Serial.available() == 0); Serial.readStringUntil('\n'); delay(500);
  for (int i = 0; i < numReadings; i++) { readRawAccel(); sumAccelX[3] += rawAccelX; sumAccelY[3] += rawAccelY; sumAccelZ[3] += rawAccelZ; delay(2); }

  Serial.println("\nColoca el sensor con el eje +Z apuntando hacia abajo (1g). Presiona ENTER para continuar.");
  while (Serial.available() == 0); Serial.readStringUntil('\n'); delay(500);
  for (int i = 0; i < numReadings; i++) { readRawAccel(); sumAccelX[4] += rawAccelX; sumAccelY[4] += rawAccelY; sumAccelZ[4] += rawAccelZ; delay(2); }

  Serial.println("\nColoca el sensor con el eje -Z apuntando hacia abajo (-1g). Presiona ENTER para continuar.");
  while (Serial.available() == 0); Serial.readStringUntil('\n'); delay(500);
  for (int i = 0; i < numReadings; i++) { readRawAccel(); sumAccelX[5] += rawAccelX; sumAccelY[5] += rawAccelY; sumAccelZ[5] += rawAccelZ; delay(2); }

  // Calcular los offsets del acelerómetro
  float accelOffsetX = (sumAccelX[0] + sumAccelX[1]) / (2.0 * numReadings);
  float accelOffsetY = (sumAccelY[2] + sumAccelY[3]) / (2.0 * numReadings);
  float accelOffsetZ = (sumAccelZ[4] + sumAccelZ[5]) / (2.0 * numReadings);

  // Calcular los factores de escala (asumiendo una escala ideal de +/- 16384 unidades por g)
  float accelScaleFactorX = (sumAccelX[0] - sumAccelX[1]) / (2.0 * 16384.0 * numReadings);
  float accelScaleFactorY = (sumAccelY[2] - sumAccelY[3]) / (2.0 * 16384.0 * numReadings);
  float accelScaleFactorZ = (sumAccelZ[4] - sumAccelZ[5]) / (2.0 * 16384.0 * numReadings);

  Serial.println("\nCalibración del acelerómetro completada:");
  Serial.print("Offset X: "); Serial.println(accelOffsetX);
  Serial.print("Offset Y: "); Serial.println(accelOffsetY);
  Serial.print("Offset Z: "); Serial.println(accelOffsetZ);
  Serial.print("Factor de Escala X: "); Serial.println(accelScaleFactorX);
  Serial.print("Factor de Escala Y: "); Serial.println(accelScaleFactorY);
  Serial.print("Factor de Escala Z: "); Serial.println(accelScaleFactorZ);
  Serial.println("\nCopia estos valores y úsalos en tu código principal.");
}

void loop() {
  // Este loop está vacío, la calibración se realiza solo una vez en el setup
}