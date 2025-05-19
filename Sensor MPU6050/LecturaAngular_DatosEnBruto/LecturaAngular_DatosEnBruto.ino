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

// Variables para almacenar las lecturas del sensor calibradas
float accelXCal, accelYCal, accelZCal;

// Función para inicializar el MPU6050 (sin cambios)
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Registro de Power Management 1
  Wire.write(0);    // Desactivar el modo de sleep
  if (Wire.endTransmission() != 0) {
    Serial.println("Error al inicializar el MPU6050");
    return false;
  }
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // Registro de Configuración del Acelerómetro
  Wire.write(0x00); // Rango de ±2g
  Wire.endTransmission();
  delay(100);
  return true;
}

// Función para leer los datos brutos del acelerómetro y aplicar la calibración
void leerAcelerometroCalibrado() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Registro del primer byte de la aceleración (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  int16_t rawAccelX = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelY = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelZ = (Wire.read() << 8) | Wire.read();

  accelXCal = (rawAccelX - accelOffsetX) / 16384.0 * accelScaleFactorX;
  accelYCal = (rawAccelY - accelOffsetY) / 16384.0 * accelScaleFactorY;
  accelZCal = (rawAccelZ - accelOffsetZ) / 16384.0 * accelScaleFactorZ;
}

// Función para calcular el ángulo de cabeceo sin filtro (usando lecturas calibradas)
float obtenerPitchSinFiltrarCalibrado() {
  leerAcelerometroCalibrado();
  float accelPitch = atan2(accelXCal, sqrt(accelYCal * accelYCal + accelZCal * accelZCal)) * 180 / PI;
  return accelPitch;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  if (initMPU6050()) {
    Serial.println("MPU6050 inicializado correctamente");
  } else {
    Serial.println("Falló la inicialización del MPU6050");
    while (1);
  }
}

void loop() {
  float anguloPitchSinFiltrar = obtenerPitchSinFiltrarCalibrado();
  Serial.print("Ángulo de Cabeceo Sin Filtrar (Calibrado): ");
  Serial.print(anguloPitchSinFiltrar);
  Serial.print(" | AccelX: "); Serial.print(accelXCal);
  Serial.print(" | AccelY: "); Serial.print(accelYCal);
  Serial.print(" | AccelZ: "); Serial.println(accelZCal);
  delay(10);
}