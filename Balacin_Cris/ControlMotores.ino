//funcion para mover los motores en funcion del angulo del sensor
int moverMotores(){
  
  int anguloAbsoluto_INT = abs((int)anguloPitchFiltrado);

  velocidad = map(anguloAbsoluto_INT, 0, 90, 80, 255); // Ajusta el rango del angulo
  velocidad = constrain(velocidad, 0, 255);

  // Gira el motor A en sentido horario
  if (anguloPitchFiltrado>0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }

  // Gira el motor A en sentido antihorario
  else{
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);  
  }

  // Establece la velocidad del motor (0-255)
  analogWrite(enB, velocidad);
  return velocidad;
}