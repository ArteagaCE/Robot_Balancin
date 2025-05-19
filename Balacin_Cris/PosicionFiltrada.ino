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