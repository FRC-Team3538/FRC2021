constexpr uint8_t PIN_SENSOR = 12;

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(PIN_SENSOR, INPUT_PULLUP);

    Serial.begin(115200);  
}

void loop()
{
    bool beam = !digitalRead(PIN_SENSOR); 
    digitalWrite(LED_BUILTIN, beam);

    if(beam) 
        Serial.write('1');
    else 
        Serial.write('0');

    delay(10);
}
