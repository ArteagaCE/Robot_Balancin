// Pines de conexión al driver L298N
const int enA = 9;   // Pin Enable A (control de velocidad PWM)
const int in1 = 8;   // Pin de entrada 1 para el motor A
const int in2 = 7;   // Pin de entrada 2 para el motor A

void setup() {
  // Configura los pines como salidas
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Inicializa el motor detenido
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0); // 0% de velocidad
}

void loop() {
  // Gira el motor A en sentido horario
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Establece la velocidad del motor (0-255)
  int velocidad = 250; // Puedes ajustar este valor
  analogWrite(enA, velocidad);

  // El motor girará en sentido horario indefinidamente hasta que se cambie la lógica en el loop
}