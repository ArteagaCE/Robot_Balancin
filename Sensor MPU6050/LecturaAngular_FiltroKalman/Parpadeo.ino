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
